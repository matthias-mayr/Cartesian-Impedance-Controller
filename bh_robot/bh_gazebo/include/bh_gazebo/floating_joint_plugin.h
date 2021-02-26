#ifndef _FLOATING_JOINT_PLUGIN_HH_
#define _FLOATING_JOINT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "tf/transform_listener.h"

namespace gazebo
{
  /// \brief A plugin to sync a floating joint with ROS tf
  class FloatingJointPlugin : public ModelPlugin
  {
    public:
    FloatingJointPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    void SetPose(physics::LinkPtr parent, physics::LinkPtr child);

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private:
    void InjectTf();

    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointers to links.
    std::string parent_frame;
    physics::LinkPtr child;

    // Callback object that tells Gazebo which method to call.
    event::ConnectionPtr callback;

    // Object that accesses the ROS TF tree.
    std::shared_ptr<tf::TransformListener> tfListener;

    // Handle to the ROS node that is created by this plugin.
    ros::NodeHandlePtr nodeHandle;
  };

}
#endif