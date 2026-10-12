#ifndef CARTESIAN_IMPEDANCE_CONTROLLER__TRANSFORM_WRENCH_HPP_
#define CARTESIAN_IMPEDANCE_CONTROLLER__TRANSFORM_WRENCH_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cartesian_impedance_controller
{

/*! \brief Transforms a 6D wrench (force and torque) using the provided transform.
 * Only rotation is applied to both force and torque vectors (translation does not affect forces).
 * \param[in,out] wrench 6D wrench vector [fx, fy, fz, tx, ty, tz]
 * \param[in] transform Transform to apply
 */
inline void transformWrench(Eigen::Matrix<double, 6, 1> *wrench,
                            const geometry_msgs::msg::Transform &transform)
{
  tf2::Vector3 force_in((*wrench)(0), (*wrench)(1), (*wrench)(2));
  tf2::Vector3 torque_in((*wrench)(3), (*wrench)(4), (*wrench)(5));
  tf2::Transform tf;
  tf2::fromMsg(transform, tf);
  tf2::Vector3 force_out = tf.getBasis() * force_in;
  tf2::Vector3 torque_out = tf.getBasis() * torque_in;
  (*wrench)(0) = force_out.x();
  (*wrench)(1) = force_out.y();
  (*wrench)(2) = force_out.z();
  (*wrench)(3) = torque_out.x();
  (*wrench)(4) = torque_out.y();
  (*wrench)(5) = torque_out.z();
}

/*! \brief Transforms a 6D wrench (force and torque) using the provided TransformStamped.
 * Only rotation is applied to both force and torque vectors (translation does not affect forces).
 * \param[in,out] wrench 6D wrench vector [fx, fy, fz, tx, ty, tz]
 * \param[in] transform TransformStamped to apply
 */
inline void transformWrench(Eigen::Matrix<double, 6, 1> *wrench,
                            const geometry_msgs::msg::TransformStamped &transform)
{
  transformWrench(wrench, transform.transform);
}

/*! \brief Transforms a WrenchStamped message using the provided TransformStamped.
 * Only rotation is applied to both force and torque vectors (translation does not affect forces).
 * The returned message frame_id is set to transform.header.frame_id and stamp to wrench.header.stamp.
 * \param[in] wrench Input WrenchStamped message
 * \param[in] transform TransformStamped to apply
 * \return Transformed WrenchStamped message
 */
inline geometry_msgs::msg::WrenchStamped transformWrench(
    const geometry_msgs::msg::WrenchStamped &wrench,
    const geometry_msgs::msg::TransformStamped &transform)
{
  Eigen::Matrix<double, 6, 1> F;
  F << wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z,
       wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z;
  transformWrench(&F, transform.transform);
  geometry_msgs::msg::WrenchStamped out;
  out.header.stamp = wrench.header.stamp;
  out.header.frame_id = transform.header.frame_id;
  out.wrench.force.x = F(0);
  out.wrench.force.y = F(1);
  out.wrench.force.z = F(2);
  out.wrench.torque.x = F(3);
  out.wrench.torque.y = F(4);
  out.wrench.torque.z = F(5);
  return out;
}

} // namespace cartesian_impedance_controller

#endif // CARTESIAN_IMPEDANCE_CONTROLLER__TRANSFORM_WRENCH_HPP_
