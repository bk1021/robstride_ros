#include "motor_ros2/motor_cfg.h"
#include <rclcpp/node.hpp>
#include <thread>
#include <unistd.h>
#include <vector> 
#include <memory> 
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include "stdint.h"
#include <atomic>
#include <iostream>

class MotorControlSample : public rclcpp::Node
{
public:
    MotorControlSample() : 
    rclcpp::Node("motor_control_set_node"),
    motor(RobStrideMotor("/dev/ttyUSB0", 0x00, 0x7F, 5))
    {
        motor.enable_motor();

        usleep(1000);
        worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);
    }

    ~MotorControlSample()
    {
        motor.Disenable_Motor(0);
        running_ = false;               // 停止线程
        if (worker_thread_.joinable())
            worker_thread_.join();      // 等待线程结束

    }

    void excute_loop()
    {
        motor.Set_ZeroPos();
        usleep(1000);
        float position = 6.28f;
        float velocity = 0.0f;
        long long print_counter = 0;
        while (running_ && rclcpp::ok())
        {
            // 自定义循环逻辑
            // 依次为速度，运控，位置模式, 电流，CSP位置
             auto start_time = std::chrono::high_resolution_clock::now();

            auto [position_feedback, velocity_feedback, torque, temperature] =
                motor.send_motion_command(0.0, position, velocity, 0.5f, 0.5f);
            // motor.RobStrite_Motor_PosPP_control(position, velocity);
            // motor.RobStrite_Motor_Current_control(0.0f);
            // motor.send_velocity_mode_command(velocity);
            // motor.RobStrite_Motor_PosCSP_control(velocity, position);

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

            if (print_counter++ % 100 == 0) {
                std::cout << "[STATUS] Duration: " << duration_us << " us, "
                    << "Position Feedback: " << position_feedback
                    << ", Velocity Feedback: " << velocity_feedback
                    << ", Torque: " << torque
                    << ", Temperature: " << temperature << std::endl;
                motor.Get_RobStrite_Motor_parameter(0x7019);
                motor.Get_RobStrite_Motor_parameter(0x701B);
                std::cout << "MechPos: " << motor.drw.mechPos.data << std::endl;
                std::cout << "mechVel: " << motor.drw.mechVel.data << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));  // loop rate

        }
    }

private:
    std::thread worker_thread_;
    std::atomic<bool> running_ = true;

    RobStrideMotor motor;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<MotorControlSample>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(controller);

    executor.spin();

    rclcpp::shutdown();

    return 0;

}