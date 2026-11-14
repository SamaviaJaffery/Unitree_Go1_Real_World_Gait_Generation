#ifndef UNITREE_CONTACT_FORCE_OPTIMIZER_H
#define UNITREE_CONTACT_FORCE_OPTIMIZER_H
#include "common/mathTypes.h"
#include <array>

// Constrained centroidal wrench allocation. Unlike the legacy BalanceCtrl,
// supports flight, a normal-force ceiling, and actuator-aware force constraints.
// Explicit inputs keep the optimizer independent of gait planning and ROS.
class ContactForceOptimizer {
  public:
    void reset() { _previous.setZero(); }
    bool allocate(const Vec34 &feetFromCom, const VecInt4 &contact,
                  const std::array<Mat3, 4> &forceToTorque, const Vec3 &effortLimits,
                  const Vec6 &weights, const Vec6 &requested, Vec34 &ground);

  private:
    Vec12 _previous = Vec12::Zero();
};
#endif
