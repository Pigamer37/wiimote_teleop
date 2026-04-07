// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define _USE_MATH_DEFINES
#include <cmath>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>

#define IMG_WIDTH 1024
#define IMG_HEIGHT 768

class ControllerHandler : public rclcpp::Node {
 public:
  ControllerHandler() : Node("controller_handler") {
    RCLCPP_INFO(this->get_logger(),"Initializing controller handler...");
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
    op_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedbackArray>(
        "/joy/set_feedback", 10);
    // set LED 4 to on as feedback that the node is running
    sensor_msgs::msg::JoyFeedbackArray feedback_msg;
    feedback_msg.array.resize(4);
    for (size_t i = 0; i < feedback_msg.array.size()-1; i++) {
      feedback_msg.array[i].type = feedback_msg.array[i].TYPE_LED;
      feedback_msg.array[i].id = i;  // LED 1-3
      feedback_msg.array[i].intensity = 0;  // off
    }
    feedback_msg.array[3].type = feedback_msg.array[3].TYPE_LED;
    feedback_msg.array[3].id = 0;  // LED 1
    feedback_msg.array[3].intensity = 1.0;  // on
    op_feedback_pub_->publish(feedback_msg);
    // get robot description
    auto robot_param = rclcpp::Parameter();
    this->declare_parameter("robot_description",
                            rclcpp::ParameterType::PARAMETER_STRING);
    this->get_parameter("robot_description", robot_param);
    auto robot_description = robot_param.as_string();

    // get end link name
    auto end_link_param = rclcpp::Parameter();
    this->declare_parameter("end_link_name",
                            rclcpp::ParameterType::PARAMETER_STRING);
    this->get_parameter("end_link_name", end_link_param);
    auto end_link_name = end_link_param.as_string();

    // get whether the gripper is controlled
    auto gripper_param = rclcpp::Parameter();
    this->declare_parameter("gripper",
                            rclcpp::ParameterType::PARAMETER_BOOL);
    this->get_parameter("gripper", gripper_param);
    gripper = gripper_param.as_bool();

    // get whether we are using wiimote classic controller
    auto classic_param = rclcpp::Parameter();
    this->declare_parameter("classic",
                            rclcpp::ParameterType::PARAMETER_BOOL);
    this->get_parameter("classic", classic_param);
    classic = classic_param.as_bool();

    // create kinematic chain
    kdl_parser::treeFromString(robot_description, robot_tree_);
    robot_tree_.getChain("base_link", end_link_name, chain_);
    // create KDL solvers
    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    current_joint_positions_ = KDL::JntArray(chain_.getNrOfJoints());
    RCLCPP_INFO(this->get_logger(),"Declared KDL solvers!");
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ControllerHandler::update_joint_state, this,
                  std::placeholders::_1));

    // subscribe to controller state topic after all the setup is done to avoid processing messages before we're ready
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerHandler::topic_callback, this,
                  std::placeholders::_1));

    if (gripper) {
      grip_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gripper_controller/commands", 10);
    } else {
      grip_publisher_ = nullptr;
    }
    // set LED 1 to on as feedback that the node is configured and ready to receive messages
    for (size_t i = 0; i < feedback_msg.array.size(); i++) {
      feedback_msg.array[i].type = feedback_msg.array[i].TYPE_LED;
      feedback_msg.array[i].id = i;  // LED 1-4
      feedback_msg.array[i].intensity = 0;  // off
    }
    feedback_msg.array[3].type = feedback_msg.array[3].TYPE_LED;
    feedback_msg.array[3].id = 0;  // LED 1
    feedback_msg.array[3].intensity = 1.0;  // on
    op_feedback_pub_->publish(feedback_msg);
    RCLCPP_INFO(this->get_logger(),"Finished setting up controller_handler");
  }

 private:
  // TODO: get current joint positions from robot state topic
  void update_joint_state(const sensor_msgs::msg::JointState& msg) {
    for (size_t i = 0; i < msg.name.size(); i++) {
      for (size_t j = 0; j < chain_.getNrOfJoints(); j++) {
        // match joint names from msg to joint names in kinematic chain to get
        // current joint positions in the correct order for the IK solver
        if (msg.name[i] == chain_.getSegment(j).getJoint().getName()) {
          current_joint_positions_(j) = msg.position[i];
          break;
        }
      }
    }
  }
  void topic_callback(const sensor_msgs::msg::Joy& msg) const {
    KDL::Frame desired_pose = get_desired_pose(msg);
    auto desired_joint_positions = KDL::JntArray(chain_.getNrOfJoints());
    // inverse kinematics
    ik_solver_->CartToJnt(current_joint_positions_, desired_pose,
                          desired_joint_positions);

    std_msgs::msg::Float64MultiArray articular_pose_msg;
    articular_pose_msg.data.resize(chain_.getNrOfJoints());
    // copy to msg
    std::memcpy(articular_pose_msg.data.data(),
                desired_joint_positions.data.data(),
                articular_pose_msg.data.size() *
                    sizeof(double));  // flat64 is equivalent to double

    publisher_->publish(articular_pose_msg);
    // handle tool (button 7 is right trigger)
    if (gripper) {
      std_msgs::msg::Float64MultiArray gripper_pose_msg;
      if ((msg.buttons[7]==1 && !classic) || (msg.buttons[9]==1 && classic)) {  // close
        gripper_pose_msg.data = std::vector<double>{0.0,0.0,M_PI/2,M_PI/2,
                                                    0.0,0.0,M_PI/2,M_PI/2,
                                                    0.0,0.0,M_PI/2,M_PI/2};
        grip_publisher_->publish(gripper_pose_msg);
      } else if ((msg.buttons[5]==1 && !classic) || (msg.buttons[10]==1 && classic)) { // open
        gripper_pose_msg.data = std::vector<double>{0.0,0.0,0.0,0.0,
                                                    0.0,0.0,0.0,0.0,
                                                    0.0,0.0,0.0,0.0};
        grip_publisher_->publish(gripper_pose_msg);
      }
    }
  }
  KDL::Frame get_desired_pose(const sensor_msgs::msg::Joy& msg) const{
    KDL::Frame curr_cartesian_pose;
    fk_solver_->JntToCart(current_joint_positions_, curr_cartesian_pose);
    // deal with origin
    float axis_multiplier = 0.2, rot_multiplier = 0.5;
    float x_inc = 0, y_inc = 0, z_inc = 0, x_rot = 0, y_rot = 0, z_rot = 0;
    if(!classic) {
      x_inc = msg.axes[0] * axis_multiplier;
      y_inc = msg.axes[1] * axis_multiplier;
      if (msg.buttons[4]==1 && msg.buttons[6]==0) { // Left shoulder button
        z_inc = 0.75 * axis_multiplier; // go up
      } else if (msg.buttons[4]==0 && msg.buttons[6]==1) { // Right shoulder
        z_inc = -0.75 * axis_multiplier; // go down
      }

      // deal with orientation
      y_rot = msg.axes[3] * rot_multiplier;
      x_rot = msg.axes[2] * rot_multiplier;
      if (msg.buttons[1]==0 && msg.buttons[2]==1) {
        z_rot = 0.75 * rot_multiplier; // roll counter-clockwise (from robot)
      } else if (msg.buttons[1]==1 && msg.buttons[2]==0) {
        z_rot = -0.75 * rot_multiplier; // roll clockwise (from robot)
      }
    } else {
      if (msg.buttons[12]==1 && msg.buttons[13]==0) { // Left on D-pad
        x_inc = -0.75 * axis_multiplier; // go left
      } else if (msg.buttons[12]==0 && msg.buttons[13]==1) { // Right on D-pad
        x_inc = 0.75 * axis_multiplier; // go right
      }

      if (msg.buttons[11]==1 && msg.buttons[14]==0) { // Up on D-pad
        y_inc = 0.75 * axis_multiplier; // go forwards
      } else if (msg.buttons[11]==0 && msg.buttons[14]==1) { // Down on D-pad
        y_inc = -0.75 * axis_multiplier; // go backwards
      }

      if (msg.buttons[4]==1 && msg.buttons[5]==0) { // Left shoulder button
        z_inc = 0.75 * axis_multiplier; // go up
      } else if (msg.buttons[4]==0 && msg.buttons[5]==1) { // Right shoulder
        z_inc = -0.75 * axis_multiplier; // go down
      }

      // deal with orientation
      if (msg.buttons[1]==1 && msg.buttons[2]==0) { // B button
        x_rot = 0.75 * rot_multiplier;
      } else if (msg.buttons[1]==0 && msg.buttons[2]==1) { // X button
        x_rot = -0.75 * rot_multiplier;
      }

      if (msg.buttons[3]==1 && msg.buttons[0]==0) { // Y button
        y_rot = 0.75 * rot_multiplier;
      } else if (msg.buttons[3]==0 && msg.buttons[0]==1) { // A button
        y_rot = -0.75 * rot_multiplier;
      }

      if (msg.buttons[6]==0 && msg.buttons[7]==1) {
        z_rot = 0.75 * rot_multiplier; // roll counter-clockwise (from robot)
      } else if (msg.buttons[6]==1 && msg.buttons[7]==0) {
        z_rot = -0.75 * rot_multiplier; // roll clockwise (from robot)
      }
    }

    KDL::Vector origin = curr_cartesian_pose.p;
    origin.x(origin.x() + x_inc);
    origin.y(origin.y() + y_inc);
    origin.z(origin.z() + z_inc);
    
    KDL::Rotation rotation = curr_cartesian_pose.M;
    rotation.DoRotY(y_rot);
    rotation.DoRotX(x_rot);
    rotation.DoRotZ(z_rot);

    return KDL::Frame(rotation, origin);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grip_publisher_;
  bool gripper;
  bool classic;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr op_feedback_pub_;
  KDL::Tree robot_tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray current_joint_positions_;
  KDL::Frame desired_pose_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerHandler>());
  rclcpp::shutdown();
  return 0;
}
