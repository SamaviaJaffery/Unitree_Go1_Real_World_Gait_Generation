
/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd.
 All rights reserved.

 Simulation-only gallop controller extensions.
 Inspired by Unitree Go1 locomotion architecture.
**********************************************************************/

#include "FSM/State_Gallop.h"

#ifdef solve
#undef solve
#endif

#include <algorithm>
#include <cmath>

namespace {

double clip(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

// Assumed Unitree leg ordering:
// 0 = FR, 1 = FL, 2 = RR, 3 = RL
Vec3 hip(int i) {
    return Vec3(
        i < 2 ? .1881 : -.1881,
        i % 2 ? .04675 : -.04675,
        0
    );
}

Vec3 limits() {
    return Vec3(23.7, 23.7, 35.55);
}

} // namespace


State_Gallop::State_Gallop(CtrlComponents *c)
    : FSMState(c, FSMStateName::GALLOP, "gallop"),
      _gaitWave(_period, _duty, _bias),
      _gait(c, &_gaitWave) {
}


State_Gallop::~State_Gallop() = default;


// ------------------------------------------------------------
// Enter
// ------------------------------------------------------------

void State_Gallop::enter() {

    _fault = _requested = false;

#ifndef COMPILE_WITH_SIMULATION
    fail("gallop is simulation only");
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

    // Go1 rigid-body properties.
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


// ------------------------------------------------------------
// Exit
// ------------------------------------------------------------

void State_Gallop::exit() {

#ifdef COMPILE_WITH_SIMULATION
    _ctrlComp->estimator->setSimulationDt(_ctrlComp->dt);
#endif

    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}


// ------------------------------------------------------------
// FSM transitions
// ------------------------------------------------------------

FSMStateName State_Gallop::checkChange() {

    if (_fault ||
        _lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }

    if (_lowState->userCmd == UserCommand::L2_A) {

        double phase =
            std::fmod(_phaseTime, _period) / _period;

        if (!active() ||
            (_ctrlComp->contact->sum() == 4 && phase > .45)) {

            return FSMStateName::FIXEDSTAND;
        }
    }

    return FSMStateName::GALLOP;
}


// ------------------------------------------------------------
// Gallop gait scheduling
// ------------------------------------------------------------

#ifdef COMPILE_WITH_SIMULATION

void State_Gallop::updateGallopWave() {

    const long long now = getSystemTime();

    if (now < _lastTick) {
        fail("simulation clock reset; restart controller");
        return;
    }

    _dt = clip(
        (now - _lastTick) * 1e-6,
        .0002,
        .02
    );

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

        _reference(2) =
            _height + (_R * _comBody)(2);
    }

    _phaseTime = (now - _stageEpoch) * 1e-6;

    // Left-leading rotary gallop:
    //
    // RR -> RL -> FR -> FL -> Flight
    //
    // Leg order:
    // FR = 0
    // FL = 1
    // RR = 2
    // RL = 3

    WaveGenerator::sampleContactPhase(
        _phaseTime,
        _period,
        _duty,
        _bias,
        *_ctrlComp->phase,
        *_ctrlComp->contact,
        true
    );
}

#endif


// ------------------------------------------------------------
// Foot trajectory
// ------------------------------------------------------------


void State_Gallop::footTrajectory() {

    _gait.setGait(
        _command.head<2>(),
        0,
        _clearance
    );

    _gait.run(_feet, _feetVel);

    const Vec3 forward = _R0.col(0);

    for (int i = 0; i < 4; ++i) {

        const bool front = (i < 2);

        if (!(*_ctrlComp->contact)(i)) {

            const double phase =
                clip((*_ctrlComp->phase)(i), 0.0, 1.0);

            const double shape = sin(M_PI * phase);

            const double lift = front ? .010 : .015;
            const double reach = front ? .020 : .010;

            _feet.col(i) += forward * (reach * shape);
            _feet(2, i) += lift * shape;

            const double swingTime =
                _gaitWave.getTswing();

            const double shapeVelocity =
                M_PI * cos(M_PI * phase) / swingTime;

            _feetVel.col(i) +=
                forward * (reach * shapeVelocity);

            _feetVel(2, i) +=
                lift * shapeVelocity;
        }
    }
}



// ------------------------------------------------------------
// Body wrench
// ------------------------------------------------------------

Vec6 State_Gallop::bodyWrench() {

    const double mass = _mass;

    const Vec3 gyro =
        _lowState->getGyroGlobal();

    // Last scheduled touchdown belongs to the leading
    // foreleg (FL), with offset 0.54T.
    const double stanceEnd =
        (_lead + _duty) * _period;

    const double flight =
        _period - stanceEnd;

    double z = active()
        ? _reference(2)
        : _height + (_R * _comBody)(2);

    double vz = 0;
    double az = 0;

    // Periodic vertical trajectory.
    // Support phase: controlled acceleration.
    // Flight phase: ballistic motion.

    if (active()) {

        double t =
            std::fmod(_phaseTime, _period);

        double v0 = -9.81 * flight * .5;

        double as =
            9.81 * flight / stanceEnd;

        if (t < stanceEnd) {

            z += v0 * t + .5 * as * t * t;

            vz = v0 + as * t;
            az = as;

        } else {

            double f = t - stanceEnd;

            z += -v0 * f - .5 * 9.81 * f * f;

            vz = -v0 - 9.81 * f;
            az = -9.81;
        }
    }

    // Horizontal CoM reference.

    Vec3 desV = active()
        ? _command
        : Vec3::Zero();

    if (active())
        _reference.head<2>() += desV.head<2>() * _dt;

    Vec3 err = _reference - _comPosition;

    Vec3 ddP;

    // Horizontal body stabilization.

    ddP(0) =
        (active() ? 20. : 100.) * err(0)
        + (active() ? 8. : 20.)
        * (desV(0) - _comVelocity(0));

    ddP(1) =
        (active() ? 50. : 100.) * err(1)
        + (active() ? 12. : 20.)
        * (desV(1) - _comVelocity(1));

    // Vertical trajectory tracking.

    ddP(2) =
        az
        + 150 * (z - _comPosition(2))
        + 20 * (vz - _comVelocity(2));

    if (!ddP.allFinite()) {
        fail("body acceleration");
        return Vec6::Zero();
    }

    for (int i = 0; i < 2; ++i)
        ddP(i) = clip(ddP(i), -4, 4);

    ddP(2) = clip(ddP(2), -9.81, 15);

    // Body attitude stabilization.

    Vec3 dW =
        100 * rotMatToExp(_R0 * _R.transpose())
        - 12 * gyro;

    if (!dW.allFinite()) {
        fail("angular acceleration");
        return Vec6::Zero();
    }

    for (int i = 0; i < 3; ++i)
        dW(i) = clip(dW(i), -60, 60);

    // Centroidal Newton-Euler wrench.

    Vec6 requestedWrench;

    requestedWrench.head<3>() =
        mass * (ddP - Vec3(0, 0, -9.81));

    const Mat3 inertiaWorld =
        _R * _inertia * _R.transpose();

    requestedWrench.tail<3>() =
        inertiaWorld * dW
        + gyro.cross(inertiaWorld * gyro);

    return requestedWrench;
}


// ------------------------------------------------------------
// Force allocation and Cartesian impedance
// ------------------------------------------------------------

void State_Gallop::allocateForces(
    const Vec6 &requestedWrench
) {

    Vec34 feetFromCom =
        _ctrlComp->robotModel->getFeet2BPositions(
            *_lowState,
            FrameType::GLOBAL
        );

    std::array<Mat3, 4> forceToTorque;

    for (int i = 0; i < 4; ++i) {

        feetFromCom.col(i) -= _R * _comBody;

        forceToTorque[i] =
            _jacobians[i].transpose() * _R.transpose();
    }

    Vec6 weights;
    weights << 1, 1, 30, 100, 100, 100;

    Vec34 ground;

    if (!_optimizer.allocate(
        feetFromCom,
        *_ctrlComp->contact,
        forceToTorque,
        limits(),
        weights,
        requestedWrench,
        ground
    )) {

        fail("force QP infeasible/non-finite");
        return;
    }

    // QP returns ground-on-robot forces.
    // Actuator foot forces use opposite sign.

    Vec34 force = -ground;

    Vec34 measuredFeet =
        _ctrlComp->estimator->getFeetPos();

    Vec34 measuredVel =
        _ctrlComp->estimator->getFeetVel();

    // Swing Cartesian impedance.
    // Also active during gallop flight.

    for (int i = 0; i < 4; ++i) {

        if (!(*_ctrlComp->contact)(i)) {

            force.col(i) =
                400 * (_feet.col(i) - measuredFeet.col(i))
                + 12 * (_feetVel.col(i) - measuredVel.col(i));
        }
    }

    if (!force.allFinite()) {
        fail("non-finite foot force");
        return;
    }

    // Cartesian force to joint torque.

    const Vec34 forceBody =
        _R.transpose() * force;

    for (int i = 0; i < 4; ++i) {

        _tau.segment<3>(3 * i) =
            _jacobians[i].transpose()
            * forceBody.col(i);
    }
}


// ------------------------------------------------------------
// Joint targets
// ------------------------------------------------------------

void State_Gallop::jointTargets() {

    Vec34 p, v;

    for (int i = 0; i < 4; ++i) {

        p.col(i) =
            _R.transpose() * (_feet.col(i) - _position);

        Vec3 h = p.col(i) - hip(i);

        double yz = h.tail<2>().norm();

        if (yz < .10) {

            if (yz < 1e-8)
                h(2) = -.10;
            else
                h.tail<2>() *= .10 / yz;
        }

        yz = h.tail<2>().norm();

        if (yz > .38) {
            h.tail<2>() *= .38 / yz;
            yz = .38;
        }

        double maxX =
            std::sqrt(.385 * .385 - yz * yz);

        if (std::abs(h(0)) > maxX)
            h(0) = clip(h(0), -maxX, maxX);

        p.col(i) = hip(i) + h;

        v.col(i) =
            _R.transpose() * (_feetVel.col(i) - _velocity)
            - _lowState->getGyro().cross(p.col(i));
    }

    _qGoal =
        _ctrlComp->robotModel->getQ(
            p,
            FrameType::BODY
        );

    for (int i = 0; i < 4; ++i) {

        _jacobians[i] =
            _ctrlComp->robotModel->getJaco(*_lowState, i);

        const Mat3 &J = _jacobians[i];

        _qdGoal.segment<3>(3 * i) =
            J.transpose()
            * (J * J.transpose()
               + .0004 * Mat3::Identity())
                  .ldlt()
                  .solve(v.col(i));
    }

    if (!_qGoal.allFinite() || !_qdGoal.allFinite()) {
        fail("inverse kinematics");
        return;
    }

    for (int i = 0; i < 12; ++i) {

        int j = i % 3;

        const double lower[] = {
            -.863, -.686, -2.818
        };

        const double upper[] = {
            .863, 4.501, -.888
        };

        _qGoal(i) =
            clip(_qGoal(i), lower[j] + .01, upper[j] - .01);

        _qdGoal(i) = clip(_qdGoal(i), -12, 12);
    }
}


// ------------------------------------------------------------
// Main control loop
// ------------------------------------------------------------

void State_Gallop::run() {

    if (_fault) {
        fail("latched fault");
        return;
    }

    _position = _ctrlComp->estimator->getPosition();
    _velocity = _ctrlComp->estimator->getVelocity();

    _R = _lowState->getRotMat();

    if (!_position.allFinite() ||
        !_velocity.allFinite() ||
        !_R.allFinite() ||
        !_lowState->getQ().allFinite() ||
        !_lowState->getQd().allFinite() ||
        !_lowState->getGyro().allFinite()) {

        fail("measured state");
        return;
    }

    if (_lowState->getQd().cwiseAbs().maxCoeff() > 80) {
        fail("measured overspeed");
        return;
    }

    updateComState();

    if (!_comPosition.allFinite() ||
        !_comVelocity.allFinite() ||
        !_inertia.allFinite()) {

        fail("mass properties");
        return;
    }

    // User velocity command.

    const Vec3 targetCommand =
        _R0 * Vec3(
            invNormalize(_lowState->userValue.ly, -.4, .4),
            -invNormalize(_lowState->userValue.lx, -.3, .3),
            0
        );

    for (int i = 0; i < 2; ++i) {

        _command(i) += clip(
            targetCommand(i) - _command(i),
            -.3 * _dt,
            .3 * _dt
        );
    }

    _requested =
        _requested || targetCommand.head<2>().norm() > .03;

    // Control pipeline.

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


// ------------------------------------------------------------
// Combined position and force control
// ------------------------------------------------------------

void State_Gallop::applyJointControl() {

    const Vec12 q =
        vec34ToVec12(_lowState->getQ());

    const Vec12 qd =
        vec34ToVec12(_lowState->getQd());

    for (int i = 0; i < 12; ++i) {

        bool swing =
            !(*_ctrlComp->contact)(i / 3);

        _tau(i) +=
            (swing ? 30. : .8) * (_qGoal(i) - q(i))
            + (swing ? 1. : .8) * (_qdGoal(i) - qd(i));

        _tau(i) =
            clip(_tau(i), -limits()(i % 3), limits()(i % 3));
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

        m.Kp = 0;
        m.Kd = 0;
    }
}


// ------------------------------------------------------------
// CoM state
// ------------------------------------------------------------

void State_Gallop::updateComState() {

    _comPosition =
        _position + _R * _comBody;

    _comVelocity =
        _velocity
        + _lowState->getGyroGlobal().cross(_R * _comBody);
}


// ------------------------------------------------------------
// Fault handling
// ------------------------------------------------------------

void State_Gallop::fail(const char *stage) {

    if (!_fault) {
        std::cerr << "Gallop failsafe: "
                  << stage << std::endl;
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
