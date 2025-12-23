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

// #include "motor_ros2/motor_cfg.h"
// #include <rclcpp/node.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <thread>
// #include <atomic>
// #include <iostream>
// #include <chrono>
// #include <mutex> // Required for thread safety
// #include <limits> // Required for numeric_limits

// class MotorControlSample : public rclcpp::Node
// {
// public:
//     MotorControlSample(std::string port, float start_pos, float target_vel)
//     : rclcpp::Node("motor_control_set_node"),
//       motor(port, 0x00, 0x7F, 5),
//       target_pos_(start_pos),
//       target_vel_(target_vel)
//     {
//         motor.enable_motor();
//         usleep(1000);

//         // 1. Start the High-Speed Control Loop (Background)
//         worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);

//         // 2. Start the Interactive Input Loop (Foreground)
//         input_thread_  = std::thread(&MotorControlSample::input_loop, this);
//     }

//     ~MotorControlSample()
//     {
//         running_ = false;

//         if (worker_thread_.joinable())
//             worker_thread_.join();

//         // std::cin blocks, so we cannot easily "join" the input thread if it's waiting for input.
//         // We detach it so the program can exit cleanly without hanging.
//         if (input_thread_.joinable())
//             input_thread_.detach();
//     }

//     // --- NEW: Interactive Input Loop ---
//     void input_loop()
//     {
//         float new_pos;
//         std::cout << "\n=============================================" << std::endl;
//         std::cout << " INTERACTIVE MODE: Type a position (rad) and hit ENTER" << std::endl;
//         std::cout << "=============================================\n" << std::endl;

//         while (running_ && rclcpp::ok())
//         {
//             std::cout << ">>> Enter Target: " << std::flush;

//             // This line BLOCKS until you type something
//             if (std::cin >> new_pos) {
//                 // Lock mutex to safely update the shared variable
//                 {
//                     std::lock_guard<std::mutex> lock(pos_mutex_);
//                     target_pos_ = new_pos;
//                 }
//                 std::cout << "[CMD] Updating Target to: " << new_pos << " rad" << std::endl;
//             }
//             else {
//                 // Handle invalid input (like letters)
//                 std::cin.clear();
//                 std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//                 std::cout << "[Ignored] Invalid input. Please enter a number." << std::endl;
//             }
//         }
//     }

//     void excute_loop()
//     {
//         motor.Set_ZeroPos();
//         usleep(1000);
//         long long print_counter = 0;

//         while (running_ && rclcpp::ok())
//         {
//             auto start_time = std::chrono::high_resolution_clock::now();

//             // 3. Read the Shared Target safely
//             float current_target;
//             {
//                 std::lock_guard<std::mutex> lock(pos_mutex_);
//                 current_target = target_pos_;
//             }

//             // Send command to motor
//             auto [position_feedback, velocity_feedback, torque, temperature] =
//             motor.RobStrite_Motor_PosPP_control(current_target, target_vel_);

//             auto end_time = std::chrono::high_resolution_clock::now();
//             auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

//             // 4. Reduced print frequency (every 2000 loops = ~2 seconds)
//             //    so it doesn't interrupt your typing too much.
//             if (print_counter++ % 2000 == 0) {
//                 std::cout << "\n[STATUS] Pos: " << position_feedback
//                           << " | Vel: " << velocity_feedback
//                           << " | Temp: " << temperature << std::endl;

//                 // Print prompt again so user knows where to type
//                 std::cout << ">>> Enter Target: " << std::flush;
//             }
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         }
//     }

// private:
//     std::thread worker_thread_;
//     std::thread input_thread_;    // Thread for keyboard input
//     std::atomic<bool> running_ = true;
//     std::mutex pos_mutex_;        // Protects target_pos_

//     float target_pos_;
//     float target_vel_;

//     RobStrideMotor motor;
// };

// int main(int argc, char **argv)
// {
//     // Default Values
//     std::string serial_port = "/dev/ttyUSB0";
//     float position = 0.0f; // Start at 0
//     float velocity = 5.0f; // Slower default for safety

//     if (argc > 1) serial_port = argv[1];
//     if (argc > 2) position = std::atof(argv[2]);
//     if (argc > 3) velocity = std::atof(argv[3]);

//     rclcpp::init(argc, argv);
//     auto controller = std::make_shared<MotorControlSample>(serial_port, position, velocity);
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(controller);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }
