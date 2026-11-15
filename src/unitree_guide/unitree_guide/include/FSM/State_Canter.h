/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 Simulation-only canter controller extensions.
***********************************************************************/
#ifndef CANTER_H
#define CANTER_H

#include "FSM/FSMState.h"
#include "control/ContactForceOptimizer.h"
#include "Gait/GaitGenerator.h"

// Experimental simulation controller. Other states retain their original controller.
class State_Canter : public FSMState {

  public:
    explicit State_Canter(CtrlComponents *ctrlComp);
    ~State_Canter();
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;
#ifdef COMPILE_WITH_SIMULATION
    void updateCanterWave();
#endif

private:
    void fail(const char *stage);
    void footTrajectory();
    Vec6 bodyWrench();
    void allocateForces(const Vec6 &wrench);
    void jointTargets();
    void applyJointControl();
    void updateComState();

    Vec3 _comBody = Vec3::Zero();
    Vec3 _comPosition = Vec3::Zero(), _comVelocity = Vec3::Zero();
    Mat3 _inertia = Mat3::Identity();
    double _mass = 12;
    
    enum class Stage { Waiting, Settling, Running };
    bool active() const { return _stage == Stage::Running; }
    Stage _stage = Stage::Waiting;
    bool _fault = false, _requested = false;
    long long _lastTick = 0, _stageEpoch = 0;
    double _dt = .002, _phaseTime = 0;

    // Same working parameters. Leg order throughout: FR, FL, RR, RL.
    const double _period = .24, _duty = .52, _lead = .30;
    const double _height = .34, _clearance = .024;
    Vec4 _bias = Vec4(.15, .30, 0, .15);
    // Dedicated durations for the reused trajectory planner; shared gait timing is untouched.
    WaveGenerator _gaitWave;
    GaitGenerator _gait;
    
    Vec3 _position = Vec3::Zero(), _velocity = Vec3::Zero(), _reference = Vec3::Zero();
    Vec3 _command = Vec3::Zero();
    
    RotMat _R = RotMat::Identity(), _R0 = RotMat::Identity();
    
    Vec34 _feet = Vec34::Zero(), _feetVel = Vec34::Zero();
    
    std::array<Mat3, 4> _jacobians;
    ContactForceOptimizer _optimizer;
    Vec12 _tau = Vec12::Zero(), _qGoal = Vec12::Zero(), _qdGoal = Vec12::Zero();
};
#endif
