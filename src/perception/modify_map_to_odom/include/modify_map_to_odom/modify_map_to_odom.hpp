#ifndef MODIFY_MAP_TO_ODOM__MODIFY_MAP_TO_ODOM_HPP
#define MODIFY_MAP_TO_ODOM__MODIFY_MAP_TO_ODOM_HPP

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

namespace modify_map_to_odom
{
    class ModifyMapToOdom : public rclcpp::Node
    {
        public:
        explicit ModifyMapToOdom(const rclcpp::NodeOptions & options);
        ~ModifyMapToOdom();

        private:
        double yaw;
        double pitch;
        double roll;
        double x;
        double y;

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

        Eigen::Isometry3d map_to_odom;

        rclcpp::TimerBase::SharedPtr map_to_odom_publish_timer;
        rclcpp::TimerBase::SharedPtr listen_keyboard_timer;

        std::mutex map_to_odom_lock;

        void map_to_odom_publish_callback();
        void set_nonblocking_mode(bool enable);
        void listen_keyboard();
        void display();
    };
} // namespace ModifyMapToOdom



#endif