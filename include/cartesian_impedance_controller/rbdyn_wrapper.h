#pragma once

#include <vector>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <SpaceVecAlg/Conversions.h>
#include <mc_rbdyn_urdf/urdf.h>
#include <Eigen/Geometry>

class rbdyn_wrapper
{
public:
  struct EefState
  {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
  };

  void init_rbdyn(const std::string &urdf_string, const std::string &end_effector)
  {
    // Convert URDF to RBDyn
    _rbdyn_urdf = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);

    _rbd_indices.clear();

    for (size_t i = 0; i < _rbdyn_urdf.mb.nrJoints(); i++)
    {
      if (_rbdyn_urdf.mb.joint(i).type() != rbd::Joint::Fixed)
        _rbd_indices.push_back(i);
    }

    for (size_t i = 0; i < _rbdyn_urdf.mb.nrBodies(); i++)
    {
      if (_rbdyn_urdf.mb.body(i).name() == end_effector)
      {
        _ef_index = i;
        return;
      }
    }

    _R_control_root.setIdentity();
    
    throw std::runtime_error("Index for end effector link " + end_effector + " not found in URDF. Aborting.");
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

    rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

    _update_urdf_state(rbdyn_urdf, q, dq);

    // Compute jacobian
    rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());

    // // TO-DO: Check if we need this
    rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
    rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);

    if (_cf_index != 0) { //0 is root link
      
      sva::PTransformd tf_control_frame = rbdyn_urdf.mbc.bodyPosW.at(_cf_index);
      _R_control_root = sva::conversions::toHomogeneous(tf_control_frame).topLeftCorner<3,3>();
    }

    return jac.jacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc);
  }

  EefState perform_fk(const Eigen::VectorXd &q) const
  {
    mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = _rbdyn_urdf;

    Eigen::VectorXd q_low = Eigen::VectorXd::Ones(_rbd_indices.size());
    Eigen::VectorXd q_high = q_low;

    for (size_t i = 0; i < _rbd_indices.size(); i++)
    {
      size_t index = _rbd_indices[i];
      q_low(i) = rbdyn_urdf.limits.lower[rbdyn_urdf.mb.joint(index).name()][0];
      q_high(i) = rbdyn_urdf.limits.upper[rbdyn_urdf.mb.joint(index).name()][0];
    }

    rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);

    for (size_t i = 0; i < _rbd_indices.size(); i++)
    {
      size_t rbd_index = _rbd_indices[i];
      double jt = q[i];
      // wrap in [-pi,pi]
      jt = _wrap_angle(jt);
      // enforce limits
      if (jt < q_low(i))
        jt = q_low(i);
      if (jt > q_high(i))
        jt = q_high(i);

      rbdyn_urdf.mbc.q[rbd_index][0] = jt;
    }

    rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);

    sva::PTransformd tf = rbdyn_urdf.mbc.bodyPosW[_ef_index];

    Eigen::Matrix4d eig_tf = sva::conversions::toHomogeneous(tf);
    Eigen::Vector3d trans = eig_tf.col(3).head(3);
    Eigen::Matrix3d rot_mat = eig_tf.block(0, 0, 3, 3);
    Eigen::Quaterniond quat = Eigen::Quaterniond(rot_mat).normalized();

    return {trans, quat};
  }

  int n_joints() const
  {
    return _rbd_indices.size();
  }

  std::string root_link() const
  {
    return _rbdyn_urdf.mb.body(0).name();
  }

  Eigen::Matrix3d get_R_control_frame() const {
    return _R_control_root;
  }

  void set_control_frame(const std::string& control_frame) {

    for (size_t i = 0; i < _rbdyn_urdf.mb.nrBodies(); i++)
    {
      if (_rbdyn_urdf.mb.body(i).name() == control_frame)
      {
        _cf_index = i;
        return;
      }
    }
    throw std::runtime_error("Index for control frame link " + control_frame + " not found in URDF. Aborting.");
  }

private:
  void _update_urdf_state(mc_rbdyn_urdf::URDFParserResult &rbdyn_urdf, const Eigen::VectorXd &q,
                          const Eigen::VectorXd &dq)
  {
    for (size_t i = 0; i < _rbd_indices.size(); i++)
    {
      size_t rbd_index = _rbd_indices[i];

      if (q.size() > i)
        rbdyn_urdf.mbc.q[rbd_index][0] = q[i];
      if (dq.size() > i)
        rbdyn_urdf.mbc.alpha[rbd_index][0] = dq[i];
    }
  }

  double _wrap_angle(const double &angle) const
  {
    double wrapped;
    if ((angle <= M_PI) && (angle >= -M_PI))
    {
      wrapped = angle;
    }
    else if (angle < 0.0)
    {
      wrapped = std::fmod(angle - M_PI, 2.0 * M_PI) + M_PI;
    }
    else
    {
      wrapped = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    }
    return wrapped;
  }

  mc_rbdyn_urdf::URDFParserResult _rbdyn_urdf;
  std::vector<size_t> _rbd_indices;
  size_t _ef_index;
  size_t _cf_index;
  Eigen::Matrix3d _R_control_root;
};