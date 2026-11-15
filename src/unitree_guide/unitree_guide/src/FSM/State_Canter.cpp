/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 Simulation-only canter controller extensions.
***********************************************************************/
#include "FSM/State_Canter.h"
// QuadProg++ is included transitively by CtrlComponents.
#ifdef solve
#undef solve
#endif
#include <algorithm>
#include <cmath>

namespace {
double clip(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
Vec3 hip(int i) { return Vec3(i < 2 ? .1881 : -.1881, i % 2 ? .04675 : -.04675, 0); }
Vec3 limits() { return Vec3(23.7, 23.7, 35.55); }
} // namespace

State_Canter::State_Canter(CtrlComponents *c)
    : FSMState(c, FSMStateName::CANTER, "canter"),
      _gaitWave(_period, _duty, _bias), _gait(c, &_gaitWave) {}

State_Canter::~State_Canter() = default;

void State_Canter::enter() {
    _fault = _requested = false;
#ifndef COMPILE_WITH_SIMULATION
    fail("canter is simulation only");
    return;
#endif
    _stage = Stage::Waiting;
    _tau.setZero();
    _qGoal.setZero();
    _qdGoal.setZero();
    _optimizer.reset();
    _command.setZero();
    _position = _ctrlComp->estimator->getPosition();
    _velocity = _ctrlComp->estimator->getVelocity();
    _R0 = rotz(_lowState->getYaw());
    _R = _lowState->getRotMat();
    // Canter-only rigid model from the existing Unitree robot parameters.
    _mass = _ctrlComp->robotModel->getRobMass();
    _comBody = _ctrlComp->robotModel->getPcb();
    _inertia = _ctrlComp->robotModel->getRobInertial();
    updateComState();
    _reference = _comPosition;
    _feet = _ctrlComp->estimator->getFeetPos();
    _feetVel.setZero();
    _gait.restart();
    _lastTick = getSystemTime();
    _phaseTime = 0;
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllStance();
}

void State_Canter::exit() {
#ifdef COMPILE_WITH_SIMULATION
    _ctrlComp->estimator->setSimulationDt(_ctrlComp->dt);
#endif
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Canter::checkChange() {
    if (_fault || _lowState->userCmd == UserCommand::L2_B)
        return FSMStateName::PASSIVE;
    if (_lowState->userCmd == UserCommand::L2_A) {
        // Hand back to fixed stand near the four-foot support window, not flight.
        double phase = std::fmod(_phaseTime, _period) / _period;
        if (!active() || (_ctrlComp->contact->sum() == 4 && phase > .45))
            return FSMStateName::FIXEDSTAND;
    }
    return FSMStateName::CANTER;
}

#ifdef COMPILE_WITH_SIMULATION
void State_Canter::updateCanterWave() {
    const long long now = getSystemTime();
    if (now < _lastTick) {
        fail("simulation clock reset; restart controller");
        return;
    }
    _dt = clip((now - _lastTick) * 1e-6, .0002, .02);
    _lastTick = now;
    _ctrlComp->estimator->setSimulationDt(_dt);
    if (_requested && _stage == Stage::Waiting) {
        _stageEpoch = now;
        _stage = Stage::Settling;
    }
    _ctrlComp->contact->setOnes();
    _ctrlComp->phase->setConstant(.5);
    if (_stage == Stage::Waiting)
        return;
    if (_stage == Stage::Settling) {
        if ((now - _stageEpoch) * 1e-6 < .4)
            return;
        _stage = Stage::Running;
        _stageEpoch = now;
        _reference(2) = _height + (_R * _comBody)(2);
    }
    // Left lead: RR at 0, FR+RL at 0.15T, FL at 0.30T; flight at 0.82T..T.
    // u is elapsed time since this leg's touchdown; phase is local to stance/swing.
    _phaseTime = (now - _stageEpoch) * 1e-6;
    WaveGenerator::sampleContactPhase(_phaseTime, _period, _duty, _bias,
                                      *_ctrlComp->phase, *_ctrlComp->contact, true);
}
#endif

void State_Canter::footTrajectory() {
    _gait.setGait(_command.head<2>(), 0, _clearance);
    _gait.run(_feet, _feetVel);
}

Vec6 State_Canter::bodyWrench() {
    const double mass = _mass;
    const Vec3 gyro = _lowState->getGyroGlobal();
    const double stanceEnd = (_lead + _duty) * _period, flight = _period - stanceEnd;
    double z = active() ? _reference(2) : _height + (_R * _comBody)(2), vz = 0, az = 0;
    // Periodic vertical motion: constant upward acceleration in support,
    // ballistic acceleration -g in flight; z and dz match at both boundaries.
    if (active()) {
        double t = std::fmod(_phaseTime, _period);
        double v0 = -9.81 * flight * .5,
               as = 9.81 * flight / stanceEnd;
        if (t < stanceEnd) {
            z += v0 * t + .5 * as * t * t; //second eq of motion
            vz = v0 + as * t;
            az = as;
        } else {
            double f = t - stanceEnd;
            z += -v0 * f - 0.5*9.81 * f * f;
            vz = -v0 - 9.81 * f;
            az = -9.81;
        }
    }
    // Hold position during settling; integrate the velocity command once active.
    Vec3 desV = active() ? _command : Vec3::Zero();
    if (active())
        _reference.head<2>() += desV.head<2>() * _dt;
    Vec3 err = _reference - _comPosition;
    // Cartesian CoM PD: a*=a_ff + Kp*(p*-p) + Kd*(v*-v).
    Vec3 ddP;
    ddP(0) = (active() ? 20. : 100.) * err(0) +
             (active() ? 8. : 20.) * (desV(0) - _comVelocity(0));
    ddP(1) =
        (active() ? 50. : 100.) * err(1) + (active() ? 12. : 20.) * (desV(1) - _comVelocity(1));
    ddP(2) = az + 150 * (z - _comPosition(2)) + 20 * (vz - _comVelocity(2));
    if (!ddP.allFinite()) {
        fail("body acceleration");
        return Vec6::Zero();
    }
    
    for (int i = 0; i < 2; ++i)
        ddP(i) = clip(ddP(i), -4, 4);
    ddP(2) = clip(ddP(2), -9.81, 15);
    
    // SO(3) attitude PD: rotation-vector error and measured angular damping.
    Vec3 dW = 100 * rotMatToExp(_R0 * _R.transpose()) - 12 * gyro;
    
    if (!dW.allFinite()) {
        fail("angular acceleration");
        return Vec6::Zero();
    }
    
    for (int i = 0; i < 3; ++i)
        dW(i) = clip(dW(i), -60, 60);
    
        // Centroidal Newton-Euler wrench [m*(a*-g); I_world*alpha* + omega x I_world*omega].
    Vec6 requestedWrench;
    requestedWrench.head<3>() = mass * (ddP - Vec3(0, 0, -9.81));
    const Mat3 inertiaWorld = _R * _inertia * _R.transpose();
    requestedWrench.tail<3>() = inertiaWorld * dW + gyro.cross(inertiaWorld * gyro);
    
    return requestedWrench;
}

void State_Canter::allocateForces(const Vec6 &requestedWrench) {
    Vec34 feetFromCom = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::GLOBAL);
    std::array<Mat3, 4> forceToTorque;
    for (int i = 0; i < 4; ++i) {
        feetFromCom.col(i) -= _R * _comBody; // feet position relative to CoM
        forceToTorque[i] = _jacobians[i].transpose() * _R.transpose();
    }
    Vec6 weights;
    weights << 1, 1, 30, 100, 100, 100;
    Vec34 ground;
    if (!_optimizer.allocate(feetFromCom, *_ctrlComp->contact, forceToTorque, limits(), weights,
                             requestedWrench, ground)) {
        fail("force QP infeasible/non-finite");
        return;
    }
    // QP returns ground-on-robot forces; stance actuator foot forces use the opposite sign.
    Vec34 force = -ground;
    Vec34 measuredFeet = _ctrlComp->estimator->getFeetPos();
    Vec34 measuredVel = _ctrlComp->estimator->getFeetVel();
   
    // Swing uses Cartesian impedance F=400*(p*-p)+12*(v*-v), including during flight.
    for (int i = 0; i < 4; ++i)
        if (!(*_ctrlComp->contact)(i))
            force.col(i) = 400 * (_feet.col(i) - measuredFeet.col(i)) +
                           12 * (_feetVel.col(i) - measuredVel.col(i));
    if (!force.allFinite()) {
        fail("non-finite foot force");
        return;
    }
    //tau_ff = J(q)^T * F_body; reuse the Unitree Jacobians.
    const Vec34 forceBody = _R.transpose() * force;
    for (int i = 0; i < 4; ++i)
        _tau.segment<3>(3 * i) = _jacobians[i].transpose() * forceBody.col(i);
}


void State_Canter::jointTargets() {
    Vec34 p, v;
    for (int i = 0; i < 4; ++i) {
        p.col(i) = _R.transpose() * (_feet.col(i) - _position);
        // Project to the reachable workspace before Unitree position IK.
        Vec3 h = p.col(i) - hip(i);
        double yz = h.tail<2>().norm();
        if (yz < .10) {
            if (yz < 1e-8)
                h(2) = -.10; //zero safeguard
            else
                h.tail<2>() *= .10 / yz; //scale outward if too close.
        }
        yz = h.tail<2>().norm();
        
        if (yz > .38) {
            h.tail<2>() *= .38 / yz; //scale inward
            yz = .38;
        }

        double maxX = std::sqrt(.385 * .385 - yz * yz);
        if (std::abs(h(0)) > maxX) {
            h(0) = clip(h(0), -maxX, maxX);
        }
        p.col(i) = hip(i) + h;
        v.col(i) =
            _R.transpose() * (_feetVel.col(i) - _velocity) - _lowState->getGyro().cross(p.col(i));
    }
    _qGoal = _ctrlComp->robotModel->getQ(p, FrameType::BODY); //p and v in body frame

    for (int i = 0; i < 4; ++i) {
        _jacobians[i] = _ctrlComp->robotModel->getJaco(*_lowState, i);
        const Mat3 &J = _jacobians[i];
        // Damped inverse velocity kinematics: J^T*(J*J^T+lambda^2*I)^(-1)*v.
        _qdGoal.segment<3>(3 * i) =
            J.transpose() * (J * J.transpose() + .0004 * Mat3::Identity()).ldlt().solve(v.col(i));
    }
    if (!_qGoal.allFinite() || !_qdGoal.allFinite()) {
        fail("inverse kinematics");
        return;
    }
    for (int i = 0; i < 12; ++i) {
        int j = i % 3;
        const double lower[] = {-.863, -.686, -2.818}, upper[] = {.863, 4.501, -.888};
        _qGoal(i) = clip(_qGoal(i), lower[j] + .01, upper[j] - .01);
        _qdGoal(i) = clip(_qdGoal(i), -12, 12);
    }
}
// Control pipeline: estimate -> trajectory -> joint targets -> body wrench -> forces -> torque.
void State_Canter::run() {
    if (_fault) {
        fail("latched fault");
        return;
    }
    _position = _ctrlComp->estimator->getPosition();
    _velocity = _ctrlComp->estimator->getVelocity();
    _R = _lowState->getRotMat();
    if (!_position.allFinite() || !_velocity.allFinite() || !_R.allFinite() ||
        !_lowState->getQ().allFinite() || !_lowState->getQd().allFinite() ||
        !_lowState->getGyro().allFinite()) {
        fail("measured state");
        return;
    } //prevents NANs

    if (_lowState->getQd().cwiseAbs().maxCoeff() > 80) {
        fail("measured overspeed");
        return;
    }

    updateComState();
    if (!_comPosition.allFinite() || !_comVelocity.allFinite() || !_inertia.allFinite()) {
        fail("mass properties");
        return;
    }
    
    const Vec3 targetCommand = _R0 * Vec3(invNormalize(_lowState->userValue.ly, -.4, .4),
                                          -invNormalize(_lowState->userValue.lx, -.3, .3), 0);
    for (int i = 0; i < 2; ++i)
        _command(i) += clip(targetCommand(i) - _command(i), -.3 * _dt, .3 * _dt);
    _requested = _requested || targetCommand.head<2>().norm() > .03;
    footTrajectory();
    jointTargets();
    if (_fault)
        return;
    const Vec6 wrench = bodyWrench();
    if (_fault)
        return;
    allocateForces(wrench);
    if (_fault)
        return;
    if (!_tau.allFinite()) {
        fail("force/torque");
        return;
    }
    applyJointControl();
}
void State_Canter::applyJointControl() {
    const Vec12 q = vec34ToVec12(_lowState->getQ()), qd = vec34ToVec12(_lowState->getQd());
    for (int i = 0; i < 12; ++i) {
        bool swing = !(*_ctrlComp->contact)(i / 3);
        // Combined position/force control; PD is added explicitly before torque saturation.
        _tau(i) +=
            (swing ? 30. : .8) * (_qGoal(i) - q(i)) + (swing ? 1. : .8) * (_qdGoal(i) - qd(i));
        _tau(i) = clip(_tau(i), -limits()(i % 3), limits()(i % 3));
    }
    if (!_tau.allFinite()) {
        fail("total torque");
        return;
    }
    for (int i = 0; i < 12; ++i) {
        auto &m = _lowCmd->motorCmd[i];
        m.mode = 10;
        m.q = _qGoal(i);
        m.dq = _qdGoal(i);
        m.tau = _tau(i);
        // Motor PD is zero because the joint impedance term is already included in tau.
        m.Kp = 0;
        m.Kd = 0;
    }
}
void State_Canter::updateComState() {
    // A constant body-frame CoM rotates with the body; no articulated model is evaluated.
    _comPosition = _position + _R * _comBody;
    _comVelocity = _velocity + _lowState->getGyroGlobal().cross(_R * _comBody);
}

void State_Canter::fail(const char *stage) {
    if (!_fault) {
        std::cerr << "Canter failsafe: " << stage << std::endl;
    }
    _fault = true;
    _ctrlComp->ioInter->setPassive();
    for (auto &m : _lowCmd->motorCmd) {
        m.mode = 10;
        m.q = 0;
        m.dq = 0;
        m.tau = 0;
        m.Kp = 0;
        m.Kd = 8;
    }
}
