#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "bh_gazebo/floating_joint_plugin.h"
#include "tf/transform_listener.h"

namespace gazebo
{
    // Register the plugin with the Gazebo server.
    GZ_REGISTER_MODEL_PLUGIN(FloatingJointPlugin)

    void FloatingJointPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      ROS_INFO_STREAM("The floating joint plugin is attach to model[" << _model->GetName() << "]");

      // Safety check
      if (_model->GetJointCount() == 0)
      {
        ROS_INFO_STREAM("Abort, Invalid joint count, floating joint plugin not loaded");
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      if (!_sdf->HasElement("child"))
      {
        ROS_ERROR_STREAM("Abort, Must provide the child link name.");
        return;
      }
      if (!_sdf->HasElement("parent"))
      {
        ROS_ERROR_STREAM("Abort, Must provide the parent link name.");
        return;
      }
      //Find the link
      this->parent_frame = _sdf->Get<std::string>("parent");
      auto child_name = _sdf->Get<std::string>("child");
      this->child = this->model->GetLink(child_name);
      ROS_INFO_STREAM("Injecting tf between [" << this->parent_frame << ", "<< child_name << "]");

      if (! this->child)
      {
          ROS_ERROR_STREAM("Abort, Missing link " << child_name << " in model. Existing links are: ");
          for(auto l : this->model->GetLinks())
              ROS_ERROR_STREAM(l->GetName());
          return;
      }

      this->nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle());
      this->tfListener = std::make_shared<tf::TransformListener>(*(this->nodeHandle));
      this->callback = event::Events::ConnectWorldUpdateBegin(std::bind(&FloatingJointPlugin::InjectTf, this));
    }

    void FloatingJointPlugin::InjectTf()
    {
        // Retrieve the link pose.
        tf::StampedTransform transform;
        try
        {
            this->tfListener->lookupTransform(this->parent_frame, this->child->GetName(), ros::Time(0), transform);
        }
        catch (const tf::TransformException & exception)
        {
            ROS_ERROR_STREAM_THROTTLE(2.0, "Failed to update link \"" << child->GetName() << "\": " << exception.what() << ".");
        }
        // Set the link pose.
        tf::Vector3 pos(transform.getOrigin());
        tf::Quaternion rot(transform.getRotation());
        ignition::math::Pose3d pose(pos.getX(), pos.getY(), pos.getZ(), rot.getW(), rot.getX(), rot.getY(), rot.getZ());
        this->child->SetWorldPose(pose);
    }
}
