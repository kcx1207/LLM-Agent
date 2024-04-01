# PX4与ROS2通信联合测试(已成功)
[PX4官方例程](https://docs.px4.io/v1.13/en/ros/ros2_offboard_control.html)

## 准备
1. ROS2环境
2. PX4编译环境
3. 下载px4_ros_com和px4_msgs功能包到ROS2工作空间中
4. 查看PX4和px4_msgs中关于vehicle_command消息是否生效（true）
5. 尝试使用ROS2节点控制无人机切换offboard模式并解锁起飞


## 尝试使用ROS2节点控制无人机切换offboard模式并解锁起飞
相关命令：
```
make px4_sitl_rtps gazebo
micrortps_agent -t UDP
$ source workspace_directory/install/setup.bash
$ ros2 run px4_ros_com offboard_control
```
如果PX4编译出现问题大部分是因为submodule没有完全下载成功，可以参考代理配置出现问题的解决方法并按照以下命令解决：
```
make distclean
git submodule update --init --recursive
make px4_sitl_rtps gazebo
```



## 问题：代理配置出问题导致无法更新PX4 submodule
```
usr@usr:~$ git clone https://github.com/eProsima/Micro-CDR.git
正克隆到 'Micro-CDR'...
fatal: 无法访问 'https://github.com/eProsima/Micro-CDR.git/'：Failed to connect to 127.0.0.1 port 7890: 拒绝连接
usr@usr:~$ git config --list
    http.proxy=http://127.0.0.1:7890
    https.proxy=http://127.0.0.1:7890
usr@usr:~$ git config --global --unset http.proxy
usr@usr:~$ git config --global --unset https.proxy
usr@usr:~$ git clone https://github.com/eProsima/Micro-CDR.git
正克隆到 'Micro-CDR'...
remote: Enumerating objects: 1765, done.
remote: Counting objects: 100% (89/89), done.
remote: Compressing objects: 100% (64/64), done.
remote: Total 1765 (delta 31), reused 76 (delta 24), pack-reused 1676
接收对象中: 100% (1765/1765), 734.19 KiB | 1.65 MiB/s, 完成.
处理 delta 中: 100% (955/955), 完成.
```



## 问题：ROS2节点发送解锁消息之后，无法解锁
```
INFO  [commander] Failsafe mode activated   
INFO  [navigator] RTL HOME activated    
WARN  [PreFlightCheck] Mode not suitable for takeoff
```
解决方案：
```
https://github.com/PX4/px4_ros_com/issues/111

QGC中设置参数：
COM_RCL_EXCEPT=4；
使其支持offboard模式
```




## 新建一个run_demo_takeoff.sh脚本来运行demo
```
#!/bin/bash

export ROS_VERSION=2
export ROS_DISTRO=foxy
export PX4AUTOPILOTHOME=/home/usr/PX4-Autopilot

# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PX4AUTOPILOTHOME/build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PX4AUTOPILOTHOME/Tools/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$PX4AUTOPILOTHOME/Tools/sitl_gazebo

# Launch Gazebo simulation with PX4 RTPS client
#open a new terminal named PX4 SITL RTPS Gazebo
#source the ros2 workspace and execute the command of gazebo starting
gnome-terminal --title="PX4 SITL RTPS Gazebo" -- /bin/bash -c 'source ~/ROS-LLM/install/setup.bash; cd $PX4AUTOPILOTHOME; make px4_sitl_rtps gazebo_iris'


#waiting for gazebo starting
sleep 12

#open a new terminal named microRTPS Agents;  
#source the ros2 workspace and eaecute the micrortps command;
gnome-terminal --title="microRTPS Agents" -- /bin/bash -c 'source ~/ROS-LLM//install/setup.bash; micrortps_agent -t UDP'


sleep 3
gnome-terminal --title="ROS2 Node" -- /bin/bash -c 'source ~/ROS-LLM//install/setup.bash;ros2 run px4_ros_com offboard_control'
```

执行命令：
```
./run_demo_takeoff.sh
```





## offboard.cpp文件注释

这个.cpp实现了名为offboard_control的ROS2节点的创建；  

节点功能：
1. 创建并初始化三个发布者，用于发布`OffboardControlMode`、`TrajectorySetpoint`和`VehicleCommand`消息。
2. 创建了一个订阅者`timesync_sub_`，用于订阅`px4_msgs::msg::Timesync`消息，并获取一个公共时间戳。
3. 具体流程：
   10s之前： 
   发布`OffboardControlMode`消息，通知飞行器进入离板控制模式。
   发布`TrajectorySetpoint`消息，提供飞行器的轨迹设定点。   
   因为offboard模式必须和设置路径点一起操作！！！    
   10s的时候解锁；


代码源码地址：/home/usr/ROS-LLM/src/px4_ros_com/src/examples/offboard/offboard_control.cpp
```
/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
#else
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
#endif

        // get common timestamp
        timesync_sub_ =
            this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
                [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                    timestamp_.store(msg->timestamp);
                });

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

                    // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

                 // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() const;
    void disarm() const;

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint() const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                     float param2 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {
    TrajectorySetpoint msg{};
    msg.timestamp = timestamp_.load();
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = -5.0;
    msg.yaw = -3.14; // [-PI:PI]

    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
                          float param2) const {
    VehicleCommand msg{};
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);/*初始化节点*/
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}

```


## 尝试更改.cpp文件设置目标点为（10,0，-5）
```
    msg.x = 10.0;
    msg.y = 0.0;
    msg.z = -5.0;
```
重新构建工作空间使功能包中的更改生效。执行命令：  
```
cd ROS_LLM
colcon build
cd ..
./run_demo_takeoff
```

## 尝试直接通过外部传入位置和姿态参数飞行
代码修改：
```
class OffboardControl : public rclcpp::Node {
public:
    OffboardControl(float x, float y, float z, float yaw) : Node("offboard_control"),
        x_(x),  
        y_(y),  
        z_(z),  
        yaw_(yaw) {}

……

void OffboardControl::publish_trajectory_setpoint() const {
    TrajectorySetpoint msg{};
    msg.timestamp = timestamp_.load();
    msg.x = x_;
    msg.y = y_;
    msg.z = -z_;
    msg.yaw = -yaw_; // [-PI:PI]

    trajectory_setpoint_publisher_->publish(msg);
}

……

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);/*初始化节点*/
    // 解析命令行参数  
    if (argc != 5) {  
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Incorrect number of arguments provided. Usage: ./your_node x y z yaw");  
        return 1;  
    }  
  
    float x = std::atof(argv[1]);  
    float y = std::atof(argv[2]);  
    float z = std::atof(argv[3]);  
    float yaw = std::atof(argv[4]);  
    rclcpp::spin(std::make_shared<OffboardControl>(x, y, z, yaw));

    rclcpp::shutdown();
    return 0;
}

```

修改.sh脚本文件
```
gnome-terminal --title="ROS2 Node" -- /bin/bash -c 'source ~/ROS-LLM//install/setup.bash;ros2 run px4_ros_com offboard_control 10 0 5 3.14'
```

重新构建工作空间使功能包中的更改生效。执行命令：  
```
cd ROS_LLM
colcon build
cd ..
./run_demo_takeoff
```