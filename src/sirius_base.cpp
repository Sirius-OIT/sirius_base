#include "sirius_base/sirius_base.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace sirius_base
{
    SiriusBase::SiriusBase(const rclcpp::NodeOptions & options) : Node("sirius_base_node", options)
    {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("wheel_odom", rclcpp::QoS(10).best_effort(), std::bind(&SiriusBase::pose_callback, this, std::placeholders::_1));
        
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&SiriusBase::process, this));
    }

    void SiriusBase::pose_callback(geometry_msgs::msg::Pose::SharedPtr data){
        pose = *data;
        RCLCPP_INFO(this->get_logger(),"data_X : %f", data->position.x);
        RCLCPP_INFO(this->get_logger(),"pose_X : %f", pose.position.x);
    }

    void SiriusBase::process(void){
        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = this->get_clock()->now();
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_footprint";
        odometry.pose.pose = pose;

        odometry_publisher_->publish(odometry);
    }

    SiriusBase::~SiriusBase(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(sirius_base::SiriusBase)