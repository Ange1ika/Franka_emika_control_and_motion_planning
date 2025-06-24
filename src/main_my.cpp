#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();



        // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    
    // Desired pose offset
    Eigen::Matrix4d desired_pose{{}};

    // Set Cartesian stiffness and damping
    const double translational_stiffness = 150.0;
    const double rotational_stiffness = 10.0;

    Eigen::MatrixXd stiffness(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) = translational_stiffness * Eigen::Matrix3d::Identity();
    stiffness.bottomRightCorner(3, 3) = rotational_stiffness * Eigen::Matrix3d::Identity();

    Eigen::MatrixXd damping(6, 6);
    damping.setZero();
    damping.topLeftCorner(3, 3) = 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
    damping.bottomRightCorner(3, 3) = 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();

    bool initialized = false;

    auto control_callback = [&](
        const franka::RobotState& state, franka::Duration /* period */) -> franka::Torques {
      
      // Convert state to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());

      if (!initialized) {
        // Get initial pose as target
        std::array<double, 16> pose_array = model.pose(franka::Frame::kEndEffector, state);
        desired_pose = Eigen::Map<const Eigen::Matrix4d>(pose_array.data());
        initialized = true;
        std::cout << "Desired pose:\n" << desired_pose << std::endl;

      }
      

      // Get current pose
      std::array<double, 16> current_pose_array = model.pose(franka::Frame::kEndEffector, state);
      Eigen::Matrix4d current_pose = Eigen::Map<const Eigen::Matrix4d>(current_pose_array.data());
      std::cout << "Curr_pose " << current_pose_array << 
 
      // Get Jacobian
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

      // Position error
      Eigen::Vector3d pos_error = desired_pose.block<3,1>(0,3) - current_pose.block<3,1>(0,3);

      // Orientation error
      Eigen::Matrix3d R_des = desired_pose.block<3,3>(0,0);
      Eigen::Matrix3d R_cur = current_pose.block<3,3>(0,0);
      Eigen::Vector3d rot_error = 0.5 * (R_des.transpose() * R_cur - R_cur.transpose() * R_des)
                                        .diagonal();  // Approximate

      // Full pose error [pos; rot]
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) = pos_error;
      error.tail(3) = rot_error;

      // Cartesian velocity = J * dq
      Eigen::Matrix<double, 6, 1> cartesian_vel = jacobian * dq;

      // Compute wrench
      Eigen::Matrix<double, 6, 1> wrench = -stiffness * error - damping * cartesian_vel;

      // Map wrench to joint torques
      Eigen::VectorXd tau_task = jacobian.transpose() * wrench;

      // Add dynamics compensation
      std::array<double, 7> coriolis_array = model.coriolis(state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::VectorXd tau = tau_task + coriolis;

      std::array<double, 7> tau_command;
      Eigen::VectorXd::Map(&tau_command[0], 7) = tau;

      return tau_command;
    };

    // Start control loop
    robot.control(control_callback);

  } catch (const franka::Exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
