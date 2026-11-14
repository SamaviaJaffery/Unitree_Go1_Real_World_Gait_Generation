/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWPASSFILTER
#define LOWPASSFILTER

class LPFilter{
public:
    LPFilter(double samplePeriod, double cutFrequency);
    ~LPFilter();
    void addValue(double newValue);
    double getValue();
    void clear();
    void setSamplePeriod(double period, double frequency = 3.0) {
        _weight = 1.0 / (1.0 + 1.0 / (6.283185307179586 * period * frequency));
    }
private:
    double _weight;
    double _pastValue;
    bool _start;
};

#endif  // LOWPASSFILTER
