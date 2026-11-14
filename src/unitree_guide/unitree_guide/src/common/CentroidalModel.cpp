#include "common/CentroidalModel.h"

namespace {
Eigen::Isometry3d transform(const urdf::Pose &p) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    double x, y, z, w;
    p.rotation.getQuaternion(x, y, z, w);
    t.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
    t.translation() = Vec3(p.position.x, p.position.y, p.position.z);
    return t;
}
} // namespace

bool CentroidalModel::initialize(const std::string &xml) {
    urdf::Model model;
    _links.clear();
    _mass = 0;
    if (!model.initString(xml))
        return false;
    addLink(model.getRoot(), -1, nullptr);
    return std::isfinite(_mass) && _mass > 0;
}

void CentroidalModel::addLink(urdf::LinkConstSharedPtr link, int parent,
                              urdf::JointConstSharedPtr joint) {
    static const char *names[] = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    Link node;
    node.parent = parent;
    if (joint) {
        node.origin = transform(joint->parent_to_joint_origin_transform);
        node.axis = Vec3(joint->axis.x, joint->axis.y, joint->axis.z);
        for (int i = 0; i < 12; ++i)
            if (joint->name == names[i])
                node.joint = i;
    }
    if (link->inertial && link->inertial->mass > 0) {
        const auto &in = link->inertial;
        node.mass = in->mass;
        node.inertial = transform(in->origin);
        node.inertia << in->ixx, in->ixy, in->ixz, in->ixy, in->iyy, in->iyz, in->ixz, in->iyz,
            in->izz;
        _mass += node.mass;
    }
    const int index = _links.size();
    _links.push_back(node);
    for (const auto &child : link->child_links)
        addLink(child, index, child->parent_joint);
}

void CentroidalModel::update(const Vec12 &q, const Vec12 &qd) {
    _position.setZero();
    _velocity.setZero();
    for (auto &link : _links) {
        if (link.parent >= 0) {
            const auto &parent = _links[link.parent];
            link.pose = parent.pose * link.origin;
            link.velocity = parent.velocity + parent.angular.cross(link.pose.translation() -
                                                                   parent.pose.translation());
            link.angular = parent.angular;
            if (link.joint >= 0) {
                link.angular += link.pose.linear() * link.axis * qd(link.joint);
                link.pose.linear() = link.pose.linear() *
                                     Eigen::AngleAxisd(q(link.joint), link.axis).toRotationMatrix();
            }
        }
        if (link.mass == 0)
            continue;
        const Vec3 offset = link.pose.linear() * link.inertial.translation();
        link.com = link.pose.translation() + offset;
        link.comVelocity = link.velocity + link.angular.cross(offset);
        const Mat3 rotation = link.pose.linear() * link.inertial.linear();
        link.rotatedInertia = rotation * link.inertia * rotation.transpose();
        _position += link.mass * link.com;
        _velocity += link.mass * link.comVelocity;
    }
    _position /= _mass;
    _velocity /= _mass;
    _inertia.setZero();
    for (const auto &link : _links)
        if (link.mass > 0) {
            const Vec3 d = link.com - _position;
            _inertia += link.rotatedInertia +
                        link.mass * (d.squaredNorm() * Mat3::Identity() - d * d.transpose());
        }
}
