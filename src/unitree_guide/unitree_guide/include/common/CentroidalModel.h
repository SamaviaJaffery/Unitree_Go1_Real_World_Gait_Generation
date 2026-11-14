#ifndef UNITREE_CENTROIDAL_MODEL_H
#define UNITREE_CENTROIDAL_MODEL_H

#include "common/mathTypes.h"
#include <urdf/model.h>
#include <vector>

// Articulated mass properties in the base frame. The immutable URDF topology,
// joint lookup and inertial transforms are cached once, outside the control loop.
// Only the simulation canter instantiates this model; QuadrupedRobot still owns
// foot kinematics and actuator-force mapping for every gait.
class CentroidalModel {
  public:
    bool initialize(const std::string &xml);
    void update(const Vec12 &q, const Vec12 &qd);
    double mass() const { return _mass; }
    const Vec3 &position() const { return _position; }
    const Vec3 &velocity() const { return _velocity; }
    const Mat3 &inertia() const { return _inertia; }

  private:
    struct Link {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int parent = -1, joint = -1;
        Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d inertial = Eigen::Isometry3d::Identity();
        Vec3 axis = Vec3::Zero();
        double mass = 0;
        Mat3 inertia = Mat3::Zero();
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        Vec3 velocity = Vec3::Zero(), angular = Vec3::Zero();
        Vec3 com = Vec3::Zero(), comVelocity = Vec3::Zero();
        Mat3 rotatedInertia = Mat3::Zero();
    };
    void addLink(urdf::LinkConstSharedPtr link, int parent, urdf::JointConstSharedPtr joint);
    std::vector<Link, Eigen::aligned_allocator<Link>> _links;
    double _mass = 0;
    Vec3 _position = Vec3::Zero(), _velocity = Vec3::Zero();
    Mat3 _inertia = Mat3::Zero();
};
#endif
