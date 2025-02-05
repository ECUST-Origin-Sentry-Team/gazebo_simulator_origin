#include "referee_pub/referee_publisher.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <QSharedMemory>
#include <gz/gui/qt.h>
using namespace std::chrono_literals;

std::map<std::string, int> referee_dict{
    {"remain_hp", 400},
    {"max_hp", 400},
    {"game_type", 1},
    {"game_progress", 4},
    {"stage_remain_time", 419},
    {"coin_remaining_num", 400},
    {"bullet_remaining_num_17mm", 300},
    {"red_1_hp", 500},
    {"red_2_hp", 250},
    {"red_3_hp", 400},
    {"red_4_hp", 400},
    {"red_5_hp", 400},
    {"red_7_hp", 600},
    {"red_outpost_hp", 1500},
    {"red_base_hp", 5000},
    {"blue_1_hp", 500},
    {"blue_2_hp", 250},
    {"blue_3_hp", 400},
    {"blue_4_hp", 400},
    {"blue_5_hp", 400},
    {"blue_7_hp", 600},
    {"blue_outpost_hp", 1500},
    {"blue_base_hp", 5000},
    {"rfid_status", 1},
};

referee_publisher::referee_publisher()
    : Node("referee_publisher"), count_(0)
{
    for (auto iter = referee_dict.begin(); iter != referee_dict.end(); ++iter)
    {
        this->declare_parameter<int>(iter->first, iter->second);
        this->get_parameter(iter->first, referee_dict[iter->first]);
    }
    
    publisher_ = this->create_publisher<referee_msg::msg::Referee>("Referee", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&referee_publisher::timer_callback, this));
}

referee_publisher::~referee_publisher()
{
    if (shared_memory.isAttached())
    {
        shared_memory.detach();
    }
}

void referee_publisher::memory(std::string msg_name)
{
    shared_memory.setKey(QString::fromStdString(std::string(msg_name)));
    if (!shared_memory.attach())
    {
        return;
    }
    if (!shared_memory.constData())
    {
        return;
    }
    shared_memory.lock();
    memcpy(&referee_dict[msg_name], shared_memory.constData(), sizeof(int));
    shared_memory.unlock();
}

void referee_publisher::timer_callback()
{
    for (auto iter = referee_dict.begin(); iter != referee_dict.end(); ++iter)
    {
        referee_publisher::memory(iter->first);
    }

    auto message = referee_msg::msg::Referee();
    message.remain_hp = referee_dict["remain_hp"];
    message.max_hp = referee_dict["max_hp"];
    message.game_type = referee_dict["game_type"];
    message.game_progress = referee_dict["game_progress"];
    message.stage_remain_time = referee_dict["stage_remain_time"];
    message.coin_remaining_num = referee_dict["coin_remaining_num"];
    message.bullet_remaining_num_17mm = referee_dict["bullet_remaining_num_17mm"];
    message.red_1_hp = referee_dict["red_1_hp"];
    message.red_2_hp = referee_dict["red_2_hp"];
    message.red_3_hp = referee_dict["red_3_hp"];
    message.red_4_hp = referee_dict["red_4_hp"];
    message.red_5_hp = referee_dict["red_5_hp"];
    message.red_7_hp = referee_dict["red_7_hp"];
    message.red_outpost_hp = referee_dict["red_outpost_hp"];
    message.red_base_hp = referee_dict["red_base_hp"];
    message.blue_1_hp = referee_dict["blue_1_hp"];
    message.blue_2_hp = referee_dict["blue_2_hp"];
    message.blue_3_hp = referee_dict["blue_3_hp"];
    message.blue_4_hp = referee_dict["blue_4_hp"];
    message.blue_5_hp = referee_dict["blue_5_hp"];
    message.blue_7_hp = referee_dict["blue_7_hp"];
    message.blue_outpost_hp = referee_dict["blue_outpost_hp"];
    message.blue_base_hp = referee_dict["blue_base_hp"];
    message.rfid_status = referee_dict["rfid_status"];

    // 发布消息
    publisher_->publish(message);
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<referee_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}