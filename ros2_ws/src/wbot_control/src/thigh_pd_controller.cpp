#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ThighPDController : public rclcpp::Node
{
public:
  ThighPDController() : Node("thigh_pd_controller")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&ThighPDController::joint_state_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "大腿 PD 控制器已升级！注入超级力矩，尝试站立...");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    int left_leg_idx = -1;
    int right_leg_idx = -1;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "left_leg_joint") { left_leg_idx = i; }
      if (msg->name[i] == "right_leg_joint") { right_leg_idx = i; }
    }

    if (left_leg_idx == -1 || right_leg_idx == -1) {
      return; 
    }

    // ================= 核心修改：温柔一点，并加上限幅 =================
    // 因为惯量极小，Kp 不能用 300，先降到 20 试试水
    double kp = 5.0; 
    double kd = 0.1;  // 阻尼也同步降低
    
    double target_pos = 0.5; 

    double left_pos = msg->position[left_leg_idx];
    double left_vel = msg->velocity[left_leg_idx];
    double left_effort = kp * (target_pos - left_pos) + kd * (0.0 - left_vel);

    double right_pos = msg->position[right_leg_idx];
    double right_vel = msg->velocity[right_leg_idx];
    double right_effort = kp * (target_pos - right_pos) + kd * (0.0 - right_vel);

    // 【新增护城河】：限制最大扭矩不超过 5.0 Nm！
    double max_torque = 1.0;
    if (left_effort > max_torque) left_effort = max_torque;
    if (left_effort < -max_torque) left_effort = -max_torque;
    if (right_effort > max_torque) right_effort = max_torque;
    if (right_effort < -max_torque) right_effort = -max_torque;

    // ================= 发送力矩 =================
    auto command_msg = std_msgs::msg::Float64MultiArray();
    command_msg.data = {left_effort, right_effort, 0.0, 0.0}; 
    publisher_->publish(command_msg);

    // ================= 关键：添加实时打印 (每 0.5 秒打印一次) =================
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "当前姿态 -> 左腿: [%.2f] 右腿: [%.2f] | 输出扭矩 -> 左: [%.1f Nm] 右: [%.1f Nm]",
      left_pos, right_pos, left_effort, right_effort);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThighPDController>());
  rclcpp::shutdown();
  return 0;
}