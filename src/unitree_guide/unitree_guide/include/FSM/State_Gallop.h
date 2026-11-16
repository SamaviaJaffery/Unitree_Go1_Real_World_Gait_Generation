
/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 Simulation-only gallop controller extensions.
***********************************************************************/
#ifndef GALLOP_H
#define GALLOP_H

#include "FSM/FSMState.h"
#include "control/ContactForceOptimizer.h"
#include "Gait/GaitGenerator.h"

// Experimental simulation-only gallop controller.
// Other FSM states retain their original controllers.
class State_Gallop : public FSMState {

public:
    explicit State_Gallop(CtrlComponents *ctrlComp);
    ~State_Gallop();

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

#ifdef COMPILE_WITH_SIMULATION
    void updateGallopWave();
#endif

private:

    // Controller functions
    void fail(const char *stage);
    void footTrajectory();
    Vec6 bodyWrench();
    void allocateForces(const Vec6 &wrench);
    void jointTargets();
    void applyJointControl();
    void updateComState();

    // Robot center of mass properties
    Vec3 _comBody = Vec3::Zero();
    Vec3 _comPosition = Vec3::Zero();
    Vec3 _comVelocity = Vec3::Zero();

    Mat3 _inertia = Mat3::Identity();
    double _mass = 12;

    // Gallop execution stages
    enum class Stage {
        Waiting,
        Settling,
        Running
    };

    bool active() const {
        return _stage == Stage::Running;
    }

    Stage _stage = Stage::Waiting;

    bool _fault = false;
    bool _requested = false;

    long long _lastTick = 0;
    long long _stageEpoch = 0;

    double _dt = .002;
    double _phaseTime = 0;

    // Gallop gait parameters
    // Leg order: FR, FL, RR, RL.
    const double _period = .45;
    const double _duty = .60;
    const double _lead = .25;

    const double _height = .34;
    const double _clearance = .040;

    const double _rearClearance = .055;
    const double _frontClearance = .045;

    const double _rearReach = .045;
    const double _frontReach = .075;

    Vec4 _bias = Vec4(.16, .24, 0.0, .08);

    // Dedicated gait timing generator.
    // Does not modify the shared gait configuration.
    WaveGenerator _gaitWave;

    // Cartesian foot trajectory generator
    GaitGenerator _gait;

    // Body state and desired reference
    Vec3 _position = Vec3::Zero();
    Vec3 _velocity = Vec3::Zero();
    Vec3 _reference = Vec3::Zero();

    Vec3 _command = Vec3::Zero();

    // Body orientation
    RotMat _R = RotMat::Identity();
    RotMat _R0 = RotMat::Identity();

    // Cartesian foot positions and velocities
    Vec34 _feet = Vec34::Zero();
    Vec34 _feetVel = Vec34::Zero();

    // Leg Jacobians
    std::array<Mat3, 4> _jacobians;

    // Ground reaction force optimizer
    ContactForceOptimizer _optimizer;

    // Joint-space commands
    Vec12 _tau = Vec12::Zero();
    Vec12 _qGoal = Vec12::Zero();
    Vec12 _qdGoal = Vec12::Zero();
};

#endif // GALLOP_H
