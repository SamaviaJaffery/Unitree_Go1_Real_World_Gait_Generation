/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 Simulation-only canter controller extensions.
***********************************************************************/
#include "FSM/State_Canter.h"
// QuadProg++ is included transitively by CtrlComponents.
#ifdef solve
#undef solve
#endif
#include "common/CentroidalModel.h"
#include <algorithm>
#include <cmath>
namespace {
double clip(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
Vec3 hip(int i) { return Vec3(i < 2 ? .1881 : -.1881, i % 2 ? .04675 : -.04675, 0); }
Vec3 limits() { return Vec3(23.7, 23.7, 35.55); }
} // namespace
State_Canter::State_Canter(CtrlComponents *c) : FSMState(c, FSMStateName::CANTER, "canter") {}
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
    if (!_model) {
        std::string xml;
        if (!ros::param::get("/robot_description", xml)) {
            fail("missing simulation URDF");
            return;
        }
        std::unique_ptr<CentroidalModel> model(new CentroidalModel);
        if (!model->initialize(xml)) {
            fail("invalid simulation URDF");
            return;
        }
        _model = std::move(model);
    }
    massProperties();
    _reference = _comPosition;
    _anchors = _ctrlComp->estimator->getFeetPos();
    _start = _end = _anchors;
    _feet = _anchors;
    _feetVel.setZero();
    _previous.setOnes();
    _nominal << .1881, .16, -.16, -.1881, -.13, .11, -.11, .13, 0, 0, 0, 0;
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
    _phaseTime = (now - _stageEpoch) * 1e-6;
    const double t = std::fmod(_phaseTime, _period);
    for (int i = 0; i < 4; ++i) {
        const double u = std::fmod(t + _period - _bias(i) * _period, _period);
        (*_ctrlComp->contact)(i) = u < _duty * _period;
        (*_ctrlComp->phase)(i) = u < _duty * _period
                                     ? u / (_duty * _period)
                                     : (u - _duty * _period) / ((1 - _duty) * _period);
        if (_phaseTime < _bias(i) * _period)
            (*_ctrlComp->phase)(i) = _phaseTime / (_bias(i) * _period);
    }
}
#endif
void State_Canter::feet() {
    auto &contact = *_ctrlComp->contact;
    auto &phase = *_ctrlComp->phase;
    for (int i = 0; i < 4; ++i) {
        if (contact(i)) {
            if (!_previous(i))
                _anchors.col(i) = _end.col(i);
            _feet.col(i) = _anchors.col(i);
            _feetVel.col(i).setZero();
        } else {
            if (_previous(i)) {
                _start.col(i) = _anchors.col(i);
                Vec3 target = _comPosition + _R0 * _nominal.col(i);
                if (active()) {
                    target.head<2>() += _comVelocity.head<2>() * (_duty * _period * .5) +
                                        .03 * (_comVelocity.head<2>() - _command.head<2>());
                }
                target(2) = _anchors(2, i);
                _end.col(i) = target;
            }
            // Initially unloaded feet wait for their first touchdown. Their
            // startup swing begins at zero and ends at the configured beat.
            double p = phase(i), duration = (1 - _duty) * _period;
            if (_phaseTime < _bias(i) * _period) {
                duration = _bias(i) * _period;
                p = _phaseTime / duration;
                phase(i) = p;
            }
            const double a = 2 * M_PI * p;
            Vec3 delta = _end.col(i) - _start.col(i);
            _feet.col(i) = _start.col(i) + delta * (a - std::sin(a)) / (2 * M_PI);
            _feetVel.col(i) = delta * (1 - std::cos(a)) / duration;
            _feet(2, i) += _clearance * (1 - std::cos(a)) * .5;
            _feetVel(2, i) = _clearance * M_PI * std::sin(a) / duration;
        }
    }
    _previous = contact;
}
void State_Canter::forces() {
    const double mass = _mass;
    const Vec3 gyro = _lowState->getGyroGlobal();
    const double stanceEnd = (_lead + _duty) * _period, flight = _period - stanceEnd;
    double z = active() ? _reference(2) : _height + (_R * _comBody)(2), vz = 0, az = 0;
    if (active()) {
        double t = std::fmod(_phaseTime, _period), v0 = -9.81 * flight * .5,
               as = 9.81 * flight / stanceEnd;
        if (t < stanceEnd) {
            z += v0 * t + .5 * as * t * t;
            vz = v0 + as * t;
            az = as;
        } else {
            double f = t - stanceEnd;
            z += -v0 * f - 4.905 * f * f;
            vz = -v0 - 9.81 * f;
            az = -9.81;
        }
    }
    // Hold position during settling; integrate the velocity command once active.
    Vec3 desV = active() ? _command : Vec3::Zero();
    if (active())
        _reference.head<2>() += desV.head<2>() * _dt;
    Vec3 err = _reference - _comPosition;
    Vec3 ddP;
    ddP.head<2>() = (active() ? 20. : 100.) * err.head<2>() +
                    (active() ? 8. : 20.) * (desV.head<2>() - _comVelocity.head<2>());
    ddP(1) =
        (active() ? 50. : 100.) * err(1) + (active() ? 12. : 20.) * (desV(1) - _comVelocity(1));
    ddP(2) = az + 150 * (z - _comPosition(2)) + 20 * (vz - _comVelocity(2));
    if (!ddP.allFinite()) {
        fail("body acceleration");
        return;
    }
    for (int i = 0; i < 2; ++i)
        ddP(i) = clip(ddP(i), -4, 4);
    ddP(2) = clip(ddP(2), -9.81, 15);
    Vec3 dW = 100 * rotMatToExp(_R0 * _R.transpose()) - 12 * gyro;
    if (!dW.allFinite()) {
        fail("angular acceleration");
        return;
    }
    for (int i = 0; i < 3; ++i)
        dW(i) = clip(dW(i), -60, 60);
    Vec34 feetFromCom = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::GLOBAL);
    std::array<Mat3, 4> forceToTorque;
    for (int i = 0; i < 4; ++i) {
        feetFromCom.col(i) -= _R * _comBody;
        forceToTorque[i] = _jacobians[i].transpose() * _R.transpose();
    }
    Vec6 requestedWrench;
    requestedWrench.head<3>() = mass * (ddP - Vec3(0, 0, -9.81));
    requestedWrench.tail<3>() =
        _R * _inertia * _R.transpose() * dW + gyro.cross(_R * _inertia * _R.transpose() * gyro);
    Vec6 weights;
    weights << 1, 1, 30, 100, 100, 100;
    Vec34 ground;
    if (!_optimizer.allocate(feetFromCom, *_ctrlComp->contact, forceToTorque, limits(), weights,
                             requestedWrench, ground)) {
        fail("force QP infeasible/non-finite");
        return;
    }
    Vec34 force = -ground;
    Vec34 measuredFeet = _ctrlComp->estimator->getFeetPos();
    Vec34 measuredVel = _ctrlComp->estimator->getFeetVel();
    for (int i = 0; i < 4; ++i)
        if (!(*_ctrlComp->contact)(i))
            force.col(i) = 400 * (_feet.col(i) - measuredFeet.col(i)) +
                           12 * (_feetVel.col(i) - measuredVel.col(i));
    if (!force.allFinite()) {
        fail("non-finite foot force");
        return;
    }
    const Vec34 forceBody = _R.transpose() * force;
    for (int i = 0; i < 4; ++i)
        _tau.segment<3>(3 * i) = _jacobians[i].transpose() * forceBody.col(i);
}
void State_Canter::joints() {
    Vec34 p, v;
    for (int i = 0; i < 4; ++i) {
        p.col(i) = _R.transpose() * (_feet.col(i) - _position);
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
        double maxX = std::sqrt(.385 * .385 - yz * yz);
        if (std::abs(h(0)) > maxX) {
            h(0) = clip(h(0), -maxX, maxX);
        }
        p.col(i) = hip(i) + h;
        v.col(i) =
            _R.transpose() * (_feetVel.col(i) - _velocity) - _lowState->getGyro().cross(p.col(i));
    }
    _qGoal = _ctrlComp->robotModel->getQ(p, FrameType::BODY);
    for (int i = 0; i < 4; ++i) {
        _jacobians[i] = _ctrlComp->robotModel->getJaco(*_lowState, i);
        const Mat3 &J = _jacobians[i];
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
    }
    if (_lowState->getQd().cwiseAbs().maxCoeff() > 80) {
        fail("measured overspeed");
        return;
    }
    massProperties();
    if (!_comPosition.allFinite() || !_comVelocity.allFinite() || !_inertia.allFinite()) {
        fail("mass properties");
        return;
    }
    const Vec3 targetCommand = _R0 * Vec3(invNormalize(_lowState->userValue.ly, -.4, .4),
                                          -invNormalize(_lowState->userValue.lx, -.3, .3), 0);
    for (int i = 0; i < 2; ++i)
        _command(i) += clip(targetCommand(i) - _command(i), -.3 * _dt, .3 * _dt);
    _requested = _requested || targetCommand.head<2>().norm() > .03;
    feet();
    joints();
    if (_fault)
        return;
    forces();
    if (_fault)
        return;
    if (!_tau.allFinite()) {
        fail("force/torque");
        return;
    }
    const Vec12 q = vec34ToVec12(_lowState->getQ()), qd = vec34ToVec12(_lowState->getQd());
    for (int i = 0; i < 12; ++i) {
        bool swing = !(*_ctrlComp->contact)(i / 3);
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
        m.Kp = 0;
        m.Kd = 0;
    }
}
void State_Canter::massProperties() {
    if (!_model)
        return;
    _model->update(vec34ToVec12(_lowState->getQ()), vec34ToVec12(_lowState->getQd()));
    _mass = _model->mass();
    _comBody = _model->position();
    _inertia = _model->inertia();
    _comPosition = _position + _R * _comBody;
    _comVelocity =
        _velocity + _lowState->getGyroGlobal().cross(_R * _comBody) + _R * _model->velocity();
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
