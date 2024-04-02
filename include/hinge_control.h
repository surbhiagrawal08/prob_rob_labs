#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo {
    class HingeControlPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
        void torqueCallback(const std_msgs::Float64::ConstPtr& msg);

    private:
        physics::ModelPtr model;
        physics::JointPtr joint;
        ros::NodeHandlePtr rosNode;
        ros::Subscriber torqueSub;
    };

}
