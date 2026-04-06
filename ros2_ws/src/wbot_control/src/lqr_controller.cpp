#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LQRController : public rclcpp::Node
{
public:
  LQRController() : Node("lqr_controller")
  {
    last_time_ = this->get_clock()->now();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&LQRController::cmd_vel_callback, this, std::placeholders::_1));

    // 发布力矩指令
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
    
    // 订阅编码器和 IMU
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&LQRController::joint_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&LQRController::imu_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "正规军 LQR 大脑已启动！正在尝试接管平衡...");
  }

private:
  // ====== 新增：控制指令订阅 ======
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // 动态目标值
  double current_target_dx_ = 0.0;
  double current_target_dpsi_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ====== 全局状态变量 ======
  double pitch_angle_ = 0.0;
  double pitch_rate_ = 0.0;
  
  double left_leg_pos_ = 0.0, left_leg_vel_ = 0.0;
  double right_leg_pos_ = 0.0, right_leg_vel_ = 0.0;
  double left_wheel_pos_ = 0.0, left_wheel_vel_ = 0.0;
  double right_wheel_pos_ = 0.0, right_wheel_vel_ = 0.0;

  // ====== 新增：LQR 积分变量 ======
  double integral_dx_ = 0.0;
  double integral_dpsi_ = 0.0;
  rclcpp::Time last_time_; // 用于计算 dt

  // ====== 新增：数据锁标志位 ======
  bool imu_data_received_ = false;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;
    
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch_angle_ = std::copysign(M_PI / 2.0, sinp); 
    else
        pitch_angle_ = std::asin(sinp);

    pitch_rate_ = msg->angular_velocity.y;
    imu_data_received_ = true; 
  }

  // ====== 新增：遥控指令回调 ======
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
      // 提取键盘发来的前进速度 (线速度 X) 和 转向速度 (角速度 Z)
      current_target_dx_ = msg->linear.x;
      current_target_dpsi_ = msg->angular.z;
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    int lt_idx = -1, rt_idx = -1, lw_idx = -1, rw_idx = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "left_leg_joint") lt_idx = i;
      if (msg->name[i] == "right_leg_joint") rt_idx = i;
      if (msg->name[i] == "left_wheel_joint") lw_idx = i;
      if (msg->name[i] == "right_wheel_joint") rw_idx = i;
    }
    
    if (lt_idx == -1 || rt_idx == -1 || lw_idx == -1 || rw_idx == -1) return; 

    // ====== 电子手刹 (采用安全参数) ======
    if (!imu_data_received_) {
        double brake_kp = 5.0;  // 降为安全刚度
        double brake_kd = 0.1;
        double wheel_brake_kd = 0.5; 

        double temp_target_thigh = 0.0; 
        double lt_eff = brake_kp * (temp_target_thigh - msg->position[lt_idx]) + brake_kd * (0.0 - msg->velocity[lt_idx]);
        double rt_eff = brake_kp * (temp_target_thigh - msg->position[rt_idx]) + brake_kd * (0.0 - msg->velocity[rt_idx]);
        double lw_eff = wheel_brake_kd * (0.0 - msg->velocity[lw_idx]); 
        double rw_eff = wheel_brake_kd * (0.0 - msg->velocity[rw_idx]);

        auto command_msg = std_msgs::msg::Float64MultiArray();
        command_msg.data = {lt_eff, rt_eff, lw_eff, rw_eff}; 
        publisher_->publish(command_msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "等待 IMU 数据，安全电子手刹已激活...");
        return; 
    }

    left_leg_pos_ = msg->position[lt_idx]; left_leg_vel_ = msg->velocity[lt_idx];
    right_leg_pos_ = msg->position[rt_idx]; right_leg_vel_ = msg->velocity[rt_idx];
    left_wheel_vel_ = msg->velocity[lw_idx];
    right_wheel_vel_ = msg->velocity[rw_idx];

    // ====== 算法模块 1：大腿 PD 支撑控制 ======
    double kp_thigh = 5.0; 
    double kd_thigh = 0.1;  
    double target_thigh = 0.0; 

    double left_leg_effort = kp_thigh * (target_thigh - left_leg_pos_) + kd_thigh * (0.0 - left_leg_vel_);
    double right_leg_effort = kp_thigh * (target_thigh - right_leg_pos_) + kd_thigh * (0.0 - right_leg_vel_);

    // ====== 算法模块 2：正规军 LQR 控制 ======
    double r = 0.01830; 
    double d = 0.07902; 

    // 当前状态提取
    double dx = (left_wheel_vel_ + right_wheel_vel_) / 2.0 * r; 
    double dtheta = pitch_rate_;
    double dpsi = (right_wheel_vel_ - left_wheel_vel_) * r / d;
    double theta = pitch_angle_;

    // 目标状态
    double target_dx = current_target_dx_;   
    double target_dpsi = current_target_dpsi_; 
    double target_theta = 0.0;

    // 积分项计算
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    if (std::abs(theta) < 0.3) {
        if (dt > 0.0 && dt < 0.1) {
            integral_dx_ += (target_dx - dx) * dt;
            integral_dpsi_ += (target_dpsi - dpsi) * dt;
        }
    } else {
        integral_dx_ = 0.0;
        integral_dpsi_ = 0.0;
    }
    last_time_ = current_time;

    // 积分限幅
    if (integral_dx_ > 2.0) integral_dx_ = 2.0;
    if (integral_dx_ < -2.0) integral_dx_ = -2.0;
    if (integral_dpsi_ > 2.0) integral_dpsi_ = 2.0;
    if (integral_dpsi_ < -2.0) integral_dpsi_ = -2.0;

    // LQR 核心公式 u = -Kx (直接采用你跑出来的矩阵结果，暴力展开防止符号错误)
    double u_L = -( -0.7726 * (dx - target_dx) 
                  - 0.0645 * dtheta 
                  - 0.0946 * (dpsi - target_dpsi) 
                  - 0.9696 * (theta - target_theta) 
                  + 1.6921 * integral_dx_ 
                  + 0.0772 * integral_dpsi_ );

    double u_R = -( -0.7726 * (dx - target_dx) 
                  - 0.0645 * dtheta 
                  + 0.0946 * (dpsi - target_dpsi) 
                  - 0.9696 * (theta - target_theta) 
                  + 1.6921 * integral_dx_ 
                  - 0.0772 * integral_dpsi_ );

    double left_wheel_effort  = u_L;
    double right_wheel_effort = u_R;

    // 轮子力矩绝对安全限幅 (先卡死在 1.5 Nm)
    double max_effort = 4.5; 
    if (left_wheel_effort > max_effort) left_wheel_effort = max_effort;
    if (left_wheel_effort < -max_effort) left_wheel_effort = -max_effort;
    if (right_wheel_effort > max_effort) right_wheel_effort = max_effort;
    if (right_wheel_effort < -max_effort) right_wheel_effort = -max_effort;

    // ====== 3. 组合指令并下发 ======
    auto command_msg = std_msgs::msg::Float64MultiArray();
    command_msg.data = {left_leg_effort, right_leg_effort, left_wheel_effort, right_wheel_effort};
    publisher_->publish(command_msg);

    // 打印状态 (如果终端显示的不是 U_L 和 U_R，说明新代码没编译进去！)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
      "Pitch: [%.3f] rad | dx: [%.2f] m/s | U_L: [%.2f] U_R: [%.2f] Nm",
      theta, dx, left_wheel_effort, right_wheel_effort);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LQRController>());
  rclcpp::shutdown();
  return 0;
}