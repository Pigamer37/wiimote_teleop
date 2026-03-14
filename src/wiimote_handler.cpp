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
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <wiimote_msgs/msg/state.hpp>

#define IMG_WIDTH 1024
#define IMG_HEIGHT 768

cv::Mat Rot90X = (cv::Mat_<double>(3, 3) << 
1, 0, 0,
0, 0,-1,   // 0, cos(M_PI / 2), -sin(M_PI / 2),
0, 1, 0);  // 0, sin(M_PI / 2), cos(M_PI / 2));
cv::Mat intrinsicCoeffs = (cv::Mat_<double>(3, 3) << 
1700, 0,    IMG_WIDTH / 2.0,
0,    1700, IMG_HEIGHT / 2.0,
0,    0,    1);
std::vector<cv::Point3f> objectPoints = {
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

KDL::Frame calc_desired_pose(std::vector<cv::Point2f> imagePoints) {
  cv::Mat R, rvec, tvec;
  // https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
  cv::solvePnP(objectPoints, imagePoints, intrinsicCoeffs, cv::noArray(), rvec,
               tvec);
  cv::Rodrigues(rvec, R);  // from vector to matrix
  // cv:Mat fullTransMat
  // R.copyTo(fullTransMat.rowRange(0, 3).colRange(0, 3));
  // tvec.copyTo(fullTransMat.rowRange(0, 3).col(3));
  // transMatInv = fullTransMat.inv();
  // from
  // https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp
  R = R.t();         // rotation of inverse
  tvec = -R * tvec;  // translation of inverse

  cv::Mat T = cv::Mat::eye(4, 4, R.type());     // T is 4x4
  T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1;  // copies R into T
  T(cv::Range(0, 3), cv::Range(3, 4)) = tvec * 1;
  // from X->right, Y->down, Z->forward to X->right, Y->forward, Z->up
  T = T * Rot90X;  // rotate to match the robot's coordinate system

  KDL::Vector translationVec =
      KDL::Vector(T.at<double>(0, 3), T.at<double>(1, 3), T.at<double>(2, 3));
  KDL::Rotation rotationMat =
      KDL::Rotation(T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2),
                    T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2),
                    T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2));

  return KDL::Frame(rotationMat, translationVec);
}

class WiimoteHandler : public rclcpp::Node {
 public:
  WiimoteHandler() : Node("wiimote_handler") {
    subscription_ = this->create_subscription<wiimote_msgs::msg::State>(
        "/wiimote/state", 10,
        std::bind(&WiimoteHandler::topic_callback, this,
                  std::placeholders::_1));
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&WiimoteHandler::update_joint_state, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_commands", 10);
    op_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedbackArray>(
        "/joy/set_feedback", 10);
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

    // create kinematic chain
    kdl_parser::treeFromString(robot_description, robot_tree_);
    robot_tree_.getChain("base_link", end_link_name, chain_);
    // create KDL solvers
    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
    current_joint_positions_ = KDL::JntArray(chain_.getNrOfJoints());
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
  void topic_callback(const wiimote_msgs::msg::State& msg) const {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    std::vector<cv::Point2f> imagePoints;
    for (auto irCoords : msg.ir_tracking) {
      if (irCoords.x == msg.INVALID_FLOAT)
        break;  // invalid point, stop processing further points
      else {
        RCLCPP_INFO(this->get_logger(), "IR point: x=%f, y=%f", irCoords.x,
                    irCoords.y);
        // emplace points so (0,0) is top left and (IMG_WIDTH, IMG_HEIGHT) is
        // bottom right
        imagePoints.emplace_back(IMG_WIDTH - 1 - irCoords.x,
                                 IMG_HEIGHT - 1 - irCoords.y);
      }
    }
    // msg.buttons[msg.MSG_BTN_A] // A button bool
    // msg.buttons[msg.MSG_BTN_B] // B button bool
    // check that we got the max number of points (4) and that they are valid
    // before proceeding
    KDL::Frame desired_pose;
    auto desired_joint_positions = KDL::JntArray(chain_.getNrOfJoints());
    if (imagePoints.size() != 4) {
      RCLCPP_WARN(this->get_logger(), "Lesser than 4 IR points: %zu",
                  imagePoints.size());
      // TODO: Calculate the new pose based on previous and accelerometer data
      // instead of just using the previous pose
      desired_joint_positions = current_joint_positions_;
    } else {
      desired_pose = calc_desired_pose(imagePoints);
      // inverse kinematics
      ik_solver_->CartToJnt(current_joint_positions_, desired_pose,
                            desired_joint_positions);
    }

    std_msgs::msg::Float64MultiArray articular_pose_msg;
    articular_pose_msg.data.resize(chain_.getNrOfJoints());
    // copy to msg
    std::memcpy(articular_pose_msg.data.data(),
                desired_joint_positions.data.data(),
                articular_pose_msg.data.size() *
                    sizeof(double));  // flat64 is equivalent to double

    publisher_->publish(articular_pose_msg);
  }
  rclcpp::Subscription<wiimote_msgs::msg::State>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr op_feedback_pub_;
  KDL::Tree robot_tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  KDL::JntArray current_joint_positions_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WiimoteHandler>());
  rclcpp::shutdown();
  return 0;
}
