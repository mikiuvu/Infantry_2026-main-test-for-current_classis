#ifndef VISION_SERIAL_DRIVER_HPP
#define VISION_SERIAL_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vision_interfaces/msg/auto_aim.hpp>
#include <vision_interfaces/msg/robot.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "avgFilter.hpp"
#include "packet.h"

using namespace drivers::serial_driver;
using namespace std::chrono_literals;

class serial_driver_node : public rclcpp::Node
{
public:
    /*
    @brief 串口驱动节点构造函数
    @param[in] device_name 串口名称
    @param[in] node_name 节点名称
    */
    serial_driver_node(std::string device_name, std::string node_name);

    /*@brief 串口驱动节点析构函数*/
    ~serial_driver_node();

private:
    /*@brief 串口重启回调函数*/
    void serial_reopen_callback();

    /*@brief 串口读取线程函数*/
    void serial_read_thread();

    /*@brief 串口写入函数*/
    void serial_write(uint8_t *data, size_t len);

    /*
    @brief 自瞄回调函数
    @param vMsg 自瞄信息
    */

    void auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg);

    /*@brief 机器人状态回调函数*/
    void robot_callback();

    visionArray *vArray;
    robotArray *rArray;
    avgFilter muzzleSpeedFilter;
    bool isOpen = false;
    std::string *dev_name;
    std::thread serialReadThread;
    SerialPortConfig *portConfig;
    IoContext ctx;
    SerialDriver serialDriver = SerialDriver(ctx);

    // Param client to set detect_color
    using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    bool initial_set_param_ = false;
    uint8_t previous_receive_color_ = 0;
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    ResultFuturePtr set_param_future_;


    // Broadcast tf from odom to gimbal_link
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr reopenTimer;
    rclcpp::TimerBase::SharedPtr publishTimer;
    rclcpp::Publisher<vision_interfaces::msg::Robot>::SharedPtr publisher;
    rclcpp::Subscription<vision_interfaces::msg::AutoAim>::SharedPtr autoAimSub;
};

#endif // VISION_SERIAL_DRIVER_HPP
