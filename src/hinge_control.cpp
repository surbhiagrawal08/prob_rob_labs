#include <hinge_control.h>

namespace gazebo {
    void HingeControlPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized()) {
            ROS_ERROR("ROS is not initialized.");
            return;
        }

        this->model = _model;
        this->joint = this->model->GetJoint("hinge");

        this->rosNode.reset(new ros::NodeHandle(""));
        this->torqueSub = this->rosNode->subscribe<std_msgs::Float64>(
                "/" + this->model->GetName() + "/torque", 10,
                &HingeControlPlugin::torqueCallback, this);
    }

    void HingeControlPlugin::torqueCallback(
        const std_msgs::Float64::ConstPtr& msg) {
        double torque = msg->data;
        this->joint->SetForce(0, torque);
    }

    GZ_REGISTER_MODEL_PLUGIN(HingeControlPlugin)
}
