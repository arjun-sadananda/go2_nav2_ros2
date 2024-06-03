// #include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>


class JointPublisher : public rclcpp::Node
{ 
public:
    JointPublisher()
      : Node("joint_publisher"), count_(0){
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/go2_joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this));
      }
private:
    void timer_callback(){
      auto message = trajectory_msgs::msg::JointTrajectory();

      message.joint_names.push_back("FR_hip_joint");
      message.joint_names.push_back("FL_hip_joint");
      message.joint_names.push_back("RR_hip_joint");
      message.joint_names.push_back("RL_hip_joint");

      message.joint_names.push_back("FR_thigh_joint");
      message.joint_names.push_back("FL_thigh_joint");
      message.joint_names.push_back("RR_thigh_joint");
      message.joint_names.push_back("RL_thigh_joint");

      message.joint_names.push_back("FR_calf_joint");
      message.joint_names.push_back("FL_calf_joint");
      message.joint_names.push_back("RR_calf_joint");
      message.joint_names.push_back("RL_calf_joint");
      // Hip Limits
      // 1.0472       center = 0
      // -1.0472      range   = +- 1.047

      // Thigh Limits
      // 3.4907       center = 0.95995
      // -1.5708      range  = +- 2.53075

      // Calf Limits
      // -0.83776
      // -2.7227

      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      double position_hip   = 1.047*(1-cos(count_*0.1));
      double position_thigh = 0.95995 + 2.53075*(1-cos(count_*0.1));
      double position_calf  = -1.78023 + 0.94247*(1-cos(count_*0.1));
      point.positions.push_back(position_hip);
      point.positions.push_back(position_hip);
      point.positions.push_back(position_hip);
      point.positions.push_back(position_hip);
      point.positions.push_back(position_thigh);
      point.positions.push_back(position_thigh);
      point.positions.push_back(position_thigh);
      point.positions.push_back(position_thigh);
      point.positions.push_back(position_calf);
      point.positions.push_back(position_calf);
      point.positions.push_back(position_calf);
      point.positions.push_back(position_calf);
      point.time_from_start = rclcpp::Duration::from_seconds(1.0);

      message.points.push_back(point);
      publisher_->publish(message);

      RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f', '%f'", position_hip, position_thigh, position_calf);
      count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;

  // printf("hello world go2_description package\n");
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<JointPublisher>());
  rclcpp::shutdown();
  return 0;
}
