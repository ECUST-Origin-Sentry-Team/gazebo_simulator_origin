#include "modify_map_to_odom/modify_map_to_odom.hpp"

#include <tf2/impl/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <termios.h>
#include <fcntl.h>

namespace modify_map_to_odom
{
    ModifyMapToOdom::ModifyMapToOdom(const rclcpp::NodeOptions & options)
    : Node("modify_map_to_odom",options)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        set_nonblocking_mode(1);

        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        map_to_odom = Eigen::Isometry3d();
        x = 0.88;
        y = -0.816;
        map_to_odom.translation() = Eigen::Vector3d(
            x,y,0
        );

        tf2::Quaternion q;
        yaw = 0.0;
        pitch = 0.0;
        roll = 0.0;
        q.setRPY(roll,pitch,yaw);
        map_to_odom.linear() = Eigen::Quaterniond(
            q.w(),q.x(),q.y(),q.z()
        ).toRotationMatrix();
        map_to_odom_publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),std::bind(&ModifyMapToOdom::map_to_odom_publish_callback,this)
        );
        listen_keyboard_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),std::bind(&ModifyMapToOdom::listen_keyboard,this)
        );
    }

    ModifyMapToOdom::~ModifyMapToOdom()
    {
        set_nonblocking_mode(0);
    }

    void ModifyMapToOdom::map_to_odom_publish_callback()
    {
        map_to_odom_lock.lock();

        map_to_odom.translation() = Eigen::Vector3d(
            x,y,0
        );
        tf2::Quaternion q;
        q.setRPY(roll,pitch,yaw);
        map_to_odom.linear() = Eigen::Quaterniond(
            q.w(),q.x(),q.y(),q.z()
        ).toRotationMatrix();

        geometry_msgs::msg::TransformStamped tf_map_to_odom = tf2::eigenToTransform(map_to_odom);

        map_to_odom_lock.unlock();

        tf_map_to_odom.header.stamp = this->now();
        tf_map_to_odom.header.frame_id = "map";
        tf_map_to_odom.child_frame_id = "odom";
        broadcaster->sendTransform(tf_map_to_odom);
    }

    void ModifyMapToOdom::set_nonblocking_mode(bool enable)
    {
        static struct termios oldt, newt;
        if (enable)
        {
            tcgetattr(STDIN_FILENO,&oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            newt.c_cc[VMIN] = 0;
            newt.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        }
        else
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        }
    }

    void ModifyMapToOdom::listen_keyboard()
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0)
        {
            switch (c)
            {
                case 'w':
                map_to_odom_lock.lock();
                x += 0.05;
                map_to_odom_lock.unlock();
                display();
                break;
                case 's':
                map_to_odom_lock.lock();
                x -= 0.05;
                map_to_odom_lock.unlock();
                display();
                break;
                case 'a':
                map_to_odom_lock.lock();
                y += 0.05;
                map_to_odom_lock.unlock();
                display();
                break;
                case 'd':
                map_to_odom_lock.lock();
                y -= 0.05;
                map_to_odom_lock.unlock();
                display();
                break;
                case 'q':
                map_to_odom_lock.lock();
                yaw += 0.017;
                map_to_odom_lock.unlock();
                display();
                break;
                case 'e':
                map_to_odom_lock.lock();
                yaw -= 0.017;
                map_to_odom_lock.unlock();
                display();
                break;
            }
            tcflush(STDIN_FILENO, TCIFLUSH);
        }
    }

    void ModifyMapToOdom::display()
    {
        char buffer[200];
        sprintf(buffer, "x: %lf, y: %lf, yaw: %lf",x,y,yaw);
        RCLCPP_INFO(this->get_logger(),buffer);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(modify_map_to_odom::ModifyMapToOdom)