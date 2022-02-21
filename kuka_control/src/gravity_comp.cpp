#include <ros/ros.h>
#include <iostream>
#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_msgs/ControlMode.h>
#include <iiwa_ros/command/joint_position.hpp>


/**
 * Callback for when motion command is finished. Publish a finished flag.
**/
void finishedMotionCallback() {
	std::cout << "Moved to position" << std::endl;
	return;
}


int main(int argc, char** argv) { 

	std::cout << "Running gravity_comp" << std::endl;
	// return 0;

	ros::init(argc, argv, "grav_comp");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.2);

	ros::ServiceClient client = nh.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
	iiwa_msgs::ConfigureControlMode config;
	std::cout << "1" << std::endl;

	// Change to position control to get it to the right place
	/**
	config.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;	

	iiwa_msgs::CartesianControlModeLimits limits;
	iiwa_msgs::CartesianQuantity max_path_deviation;
	iiwa_msgs::CartesianQuantity max_control_force;
	iiwa_msgs::CartesianQuantity max_cartesian_velocity;

	limits.max_path_deviation = max_path_deviation;
	limits.max_control_force = max_control_force;
	limits.max_control_force_stop = true;
	limits.max_cartesian_velocity = max_cartesian_velocity;

	std::cout << "2" << std::endl;

	if (client.call(config)) {
	    if(!config.response.success)
	        ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
	    else
	        ROS_INFO_STREAM("SmartServo Service successfully called.");
	}
	else {
	    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
	}


	iiwa_ros::command::JointPosition joint_command;
	joint_command.init("iiwa");
	iiwa_msgs::JointPosition initial_joints;
	initial_joints.position.a1 = 0.1;
	initial_joints.position.a2 = 0.33;
	initial_joints.position.a3 = 0.25;
	initial_joints.position.a4 = -0.6;
	initial_joints.position.a5 = 0.35;
	initial_joints.position.a6 = 0.7;
	initial_joints.position.a7 = 0.12;

	int iter = 0;

	while (ros::ok() && iter < 3) {
	    joint_command.setPosition(initial_joints);
		std::cout << iter << std::endl;
		ros::spinOnce();
		loop_rate.sleep();	
		iter ++;
	}

	// return 0;
	**/

	// Initiate joint impedance control for grav comp
	iiwa_msgs::ConfigureControlMode joint_impedance_config;

	iiwa_msgs::JointImpedanceControlMode joint_impedance;
	iiwa_msgs::JointQuantity stiffness;
	float sn = 0.0;
	stiffness.a1 = sn;
	stiffness.a2 = sn;
	stiffness.a3 = sn;
	stiffness.a4 = sn;
	stiffness.a5 = sn;
	stiffness.a6 = sn;
	stiffness.a7 = sn;

	iiwa_msgs::JointQuantity damping;
	damping.a1 = 0.7;
	damping.a2 = 0.7;
	damping.a3 = 0.7;
	damping.a4 = 0.7;
	damping.a5 = 0.7;
	damping.a6 = 0.7;
	damping.a7 = 0.7;

	joint_impedance_config.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;	
	joint_impedance.joint_stiffness = stiffness;
	joint_impedance.joint_damping = damping;
	joint_impedance_config.request.joint_impedance = joint_impedance;
	
	if (client.call(joint_impedance_config)) {
	    if(!joint_impedance_config.response.success)
	        ROS_ERROR_STREAM("Joint impedance config failed, Java error: " << joint_impedance_config.response.error);
	    else
	        ROS_INFO_STREAM("Joint Impedance SmartServo Service successfully called.");
	}
	else {
	    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
	}


	std::cout << "Activate comp mode" << std::endl;
	 do 
	 {
	   std::cout << '\n' << "Press a key to continue...";
	 } while (std::cin.get() != '\n');

	// Change back to position control 
	ros::Duration(0.5).sleep();

	if (client.call(config)) {
	    if(!config.response.success)
	        ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
	    else
	        ROS_INFO_STREAM("SmartServo Service successfully called.");
	}
	else {
	    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
	}


	return 0;
}
