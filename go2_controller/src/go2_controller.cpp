// #include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>
#include "go2_controller/geometry/geometry.h"


class Go2Controller : public rclcpp::Node
{ 
private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    size_t count_;
    int    mode_;
    float req_vel_x, req_vel_y, req_ang_vel_z;
    double loop_rate = 200.0;
    rclcpp::Clock clock_;
    // rclcpp::Time& time
public:
    Go2Controller(): 
      Node("joint_publisher"), 
      count_(0), 
      mode_(1),
      clock_(*this->get_clock())
    {
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel/smooth", 10, std::bind(&Go2Controller::cmd_vel_callback, this,  std::placeholders::_1));
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/go2_joint_trajectory_controllers/joint_trajectory", 10);
          
        std::chrono::milliseconds period(static_cast<int>(1000/loop_rate));
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(period), std::bind(&Go2Controller::controller, this));
    }
private:
    void controller(){
      float target_joint_positions[12];
      geometry::Transformation target_foot_positions[4];
      bool foot_contacts[4];

      //check
      // body_controller_.poseCommand(target_foot_positions, req_pose_);
      unsigned long int go2_time;
      go2_time = clock_.now().nanoseconds()/1000ul;

      velocity_command(target_foot_positions, go2_time);
      kinematics_.inverse(target_joint_positions, target_foot_positions);

      // publishFootContacts_(foot_contacts); (if there is not feedback on the robot)
      publish_joints(target_joint_positions);

      // Hip Limits
      // 1.0472       center = 0
      // -1.0472      range   = +- 1.047

      // Thigh Limits
      // 3.4907       center = 0.95995
      // -1.5708      range  = + 2.53075 is max forward for all thighs - 2.53075 max back.

      // Calf Limits
      // -0.83776     center = -1.78023
      // -2.7227

      static double position_hip, position_thigh, position_calf, position_thigh_back;

    }

    void publish_joints(float target_joints[12]){
      auto message = trajectory_msgs::msg::JointTrajectory();

      // "FR_hip_joint", "FL_hip_joint", "RR_hip_joint","RL_hip_joint",
      // "FR_thigh_joint","FL_thigh_joint","RR_thigh_joint","RL_thigh_joint",
      // "FR_calf_joint","FL_calf_joint","RR_calf_joint","RL_calf_joint"

        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = clock_.now();
        joints_cmd_msg.header.stamp.sec = 0;
        joints_cmd_msg.header.stamp.nanosec = 0;

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

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = rclcpp::Duration::from_seconds(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);

        // RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f', '%f'", position_hip, position_thigh, position_calf);
        RCLCPP_INFO(this->get_logger(), "Publishing");

    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        req_vel_x = msg->linear.x;
        req_vel_y = msg->linear.y;
        req_ang_vel_z = msg->angular.z;
    }

    float capVelocities(float velocity, float min_velocity, float max_velocity)
    {
        return ((velocity)<(min_velocity)?(min_velocity):((velocity)>(max_velocity)?(max_velocity):(velocity)));
    }

    void velocity_command(geometry::Transformation (&foot_positions)[4], unsigned long int go2_time)
    {
        //limit all velocities to user input
        req_vel_x     = req_vel_x; //capVelocities(req_vel.linear.x, -base_->gait_config.max_linear_velocity_x, base_->gait_config.max_linear_velocity_x);
        req_vel_y     = req_vel_y; //capVelocities(req_vel.linear.y, -base_->gait_config.max_linear_velocity_y, base_->gait_config.max_linear_velocity_y);
        req_ang_vel_z = req_ang_vel_z; //capVelocities(req_vel.angular.z, -base_->gait_config.max_angular_velocity_z, base_->gait_config.max_angular_velocity_z);
        
        float tangential_velocity = req_ang_vel_z * base_->lf.center_to_nominal();
        float velocity =  sqrtf(pow(req_vel_x, 2) + pow(req_vel_y + tangential_velocity, 2));
        
        //calculate optimal distance to hop based
        float step_x = raibertHeuristic(base_->gait_config.stance_duration, req_vel.linear.x);
        float step_y = raibertHeuristic(base_->gait_config.stance_duration, req_vel.linear.y);
        float step_theta = raibertHeuristic(base_->gait_config.stance_duration, tangential_velocity);
        
        //calculate the angle from leg when zero to optimal distance to hop
        float theta = sinf((step_theta / 2) / base_->lf.center_to_nominal()) * 2;

        float step_lengths[4] = {0.0f,0.0f,0.0f,0.0f};
        float trajectory_rotations[4] = {0.0f,0.0f,0.0f,0.0f};    
        float sum_of_steps = 0.0f;

        for(unsigned int i = 0; i < 4; i++)
        {
            //get the step length and foot trajectory rotation for each leg
            transformLeg(step_lengths[i], trajectory_rotations[i], *base_->legs[i], step_x, step_y, theta);
            // get the average calculated step lengths since the gait generator has only one parameter for step length
            sum_of_steps += step_lengths[i];
        }

        //create a saw tooth signal gen so the trajectory planner knows whether it should swing or stride
        phase_generator.run(velocity, sum_of_steps / 4.0f, time);

        for(unsigned int i = 0; i < 4; i++)
        {
            //get the point in the swing/stride equation as a function of the sawtooth signal's magnitude for each leg
            trajectory_planners_[i]->generate(foot_positions[i], step_lengths[i], trajectory_rotations[i], phase_generator.swing_phase_signal[i], phase_generator.stance_phase_signal[i]);
        }
    }
};

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;

  // printf("hello world go2_description package\n");
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Go2Controller>());
  rclcpp::shutdown();
  return 0;
}
