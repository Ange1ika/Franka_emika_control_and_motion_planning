// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <ruckig/ruckig.hpp>


#include "examples_common.h"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
  std::ofstream log_file("../data/trajectory_log.csv");
  log_file << "time,px,py,pz,qx,qy,qz,qw\n";

  ///init ruckig
  ruckig::Ruckig<3> otg(0.01); 
  ruckig::InputParameter<3> input;
  ruckig::OutputParameter<3> output;


  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();


    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());
    Eigen::Quaterniond orientation_init = orientation_d; // target orientation
    Eigen::Quaterniond orientation_target = Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) * orientation_init;
    double orientation_progress = 0.0;  // ranges from 0.0 to 1.0
    const double orientation_step = 0.0002;  // increment per control loop (adjust speed)
    std::cout << "Initial orientation: " << orientation_init << std::endl;


    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // std::cout << "Initial position: " << input.current_position << std::endl;

    input.current_position = {position_d[0], position_d[1], position_d[2]};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
  
    input.target_position = {0.1, 0.3, 0.2};
    input.target_velocity = {0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0}; 

    input.max_velocity = {0.01, 0.01, 0.01};
    input.max_acceleration = {0.1, 0.1, 0.1};
    input.max_jerk = {1.0, 1.0, 1.0};


    ///////////////////////////// CALLBACK FUNCTION /////////////////////////////
    
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      //std:: cout << "coriolis array\n" << coriolis_array << std::endl;

      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      // //position_d[2] += 0.0001; // add a constant offset in z direction
      
      // const double max_step = 0.000001;

      

      // // Update x position
      // if (position_d[1] < 0.5) { 
      //   position_d[1] += max_step;
      // }
      // otg.update(input, output);
      // //cout << "position_d: " << position_d.transpose() << std::endl;
      // std::cout << "position: " << position.transpose() << std::endl;
      // std::cout << "orientation: " << orientation.coeffs().transpose() << std::endl;
      ruckig::Result result = otg.update(input, output);

      if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
        std::cerr << "Ruckig failed!" << std::endl;
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
      }

      position_d[0] = output.new_position[0];
      position_d[1] = output.new_position[1];
      position_d[2] = output.new_position[2];

      input.current_position = output.new_position;
      input.current_velocity = output.new_velocity;
      input.current_acceleration = output.new_acceleration;

      // double time_sec = duration.toSec();
      // log_file << time_sec << "," 
      //         << position.x() << "," << position.y() << "," << position.z() << ","
      //         << orientation.x() << "," << orientation.y() << "," << orientation.z() << "," << orientation.w() 
      //         << "\n";


      // double angle_rad = 1.0 * M_PI / 180.0;  // small step 1°
      // Eigen::AngleAxisd delta_rotation(angle_rad, Eigen::Vector3d::UnitX());
      // Eigen::Quaterniond q_delta(delta_rotation);

      // orientation_d = q_delta * orientation_d;
      if (orientation_progress < 1.0) {
          orientation_progress += orientation_step;
          orientation_progress = std::min(1.0, orientation_progress);
          orientation_d = orientation_init.slerp(orientation_progress, orientation_target);
        }


      // double orientation_error = orientation_d.angularDistance(orientation_target);


      // if (orientation_error < 0.01) {
      //   return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
      // }

      // if (position_d[1] > 0.3) {
      //   return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
      // }

      // std::cout << "Iteration: " << i << std::endl;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      bool pos_reached = result == ruckig::Result::Finished;
      bool ori_reached = (orientation_progress >= 1.0);

      if (pos_reached && ori_reached) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
      }

      return tau_d_array;
     // if ()>
     // return frankaMotionFinished(franka::Torques(0,0,0,0,));
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  log_file.close();

  //std::cout << "orientation progress: " << orientation.coeffs() << std::endl;

  return 0;
}


// quat slerp(quat q1, quat q2, float t) {
//     float angle = acos(dotProduct(q1, q2));
//     float denom = sin(angle);
//     //check if denom is zero
//     return (q1*sin((1-t)*angle)+q2*sin(t*angle))/denom;
// }