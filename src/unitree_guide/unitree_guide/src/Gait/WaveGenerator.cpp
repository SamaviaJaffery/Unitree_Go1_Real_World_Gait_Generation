/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{

    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
}

WaveGenerator::~WaveGenerator()
{
}

void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status)
{

    calcWave(_phase, _contact, status);

    if (status != _statusPast)
    {
        if (_switchStatus.sum() == 0)
        {
            _switchStatus.setOnes();
        }
        calcWave(_phasePast, _contactPast, _statusPast);
        // two special case
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0)
    {
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i))
            {
                _switchStatus(i) = 0;
            }
            else
            {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;
        }
    }

    phaseResult = _phase;
    contactResult = _contact;
}

float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

float WaveGenerator::getT()
{
    return _period;
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)
    {
        _passT = (double)(getSystemTime() - _startT) * 1e-6;
        sampleContactPhase(_passT, _period, _stRatio, _bias, phase, contact);
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
}

//for canter
void WaveGenerator::sampleContactPhase(double elapsed, double period, double stanceRatio,
                                      const Vec4 &bias, Vec4 &phase, VecInt4 &contact,
                                      bool startup)
{
    for (int i = 0; i < 4; ++i) {
                const double cycle = fmod(elapsed + period - period * bias(i), period) / period;
        contact(i) = cycle < stanceRatio;
        phase(i) = contact(i) ? cycle / stanceRatio : (cycle - stanceRatio) / (1 - stanceRatio);
        
        // Before the first biased touchdown, a startup swing begins at phase zero.
        const double firstTouchdown = bias(i) * period;
        if (startup && elapsed < firstTouchdown)
            phase(i) = elapsed / firstTouchdown;
    }
}
