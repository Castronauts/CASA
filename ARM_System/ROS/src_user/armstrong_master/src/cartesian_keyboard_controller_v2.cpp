#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include <moveit/kinematic_constraints/utils.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>

#include <time.h>
#include <stdio.h>
#include <cmath>
#include <signal.h>

float control_signals [3] = {0.0, 0.0, 0.0};
bool controls_updated = false;

void updateControlSignal(const geometry_msgs::Point::ConstPtr& msg)
{
	if(control_signals[0] != msg->x || control_signals[1] != msg->y || control_signals[2] != msg->z)
	{
		control_signals[0] = msg->x;
		control_signals[1] = msg->y;
		control_signals[2] = msg->z;
		controls_updated = true;
	}
}

void moveToSpecifiedPose(moveit::planning_interface::MoveGroupInterface * group, int position_setting)
{
	if (position_setting == 0)
	{
		group->setNamedTarget("Claw_Start");
		group->move();
	}

	sleep(3.0);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface * planning, moveit::planning_interface::MoveGroupInterface * move)
{
	//Creating ground collision object being 2 meter box from center of robot
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move->getPlanningFrame();
	collision_object.id = "ground";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 2.0; //X
	primitive.dimensions[1] = 2.0; //Y
	primitive.dimensions[2] = 0.05; //Z

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0; //Specify towards the center
	box_pose.position.y = 0;
	box_pose.position.z = 0;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	planning->addCollisionObjects(collision_objects);
}


int main(int argc, char **argv)
{
	//ROS Declarations
	ros::init(argc, argv, "cartesian_keyboard_controller");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4); //Have 4 cores total but only need 2 for updateControlSignal and main loop
	spinner.start();

	//Move It Declarations
	static const std::string PLANNING_GROUP = "Arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	robot_trajectory::RobotTrajectory rt_planner(move_group.getRobotModel(), PLANNING_GROUP);
	trajectory_processing::IterativeParabolicTimeParameterization time_planner;

	//Controller Topic Signal
	ros::Subscriber control_sub = nh.subscribe("control_signal", 1, updateControlSignal);

	//Poses needed to be saved
	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::PoseStamped ideal_pose;
	geometry_msgs::PoseStamped current_pose;

	//Add initial collision objects
	addCollisionObjects(&planning_scene_interface, &move_group);

	//Save initial behavior
	moveToSpecifiedPose(&move_group, 0); //Move to initial claw arm position Position 0
	target_pose = move_group.getCurrentPose();
	ideal_pose = move_group.getCurrentPose();

	//ROS Wait
	ros::Rate r(10);

	//Controls updated tracker
	int pressed_released = 0; //Odd is pressed, Even is released

	while(ros::ok())
	{
		/*if (controls_updated)
		{
			pressed_released++;
			if(pressed_released % 2 == 0) //This means controls updated second time then released
			{
				ROS_ERROR("Released");
				//Get current position for target
				current_pose = move_group.getCurrentPose();
				target_pose.pose.position.x = current_pose.pose.position.x;
				target_pose.pose.position.y = current_pose.pose.position.y;
				target_pose.pose.position.z = current_pose.pose.position.z;

				pressed_released = 0;
			}
			else
			{
				ROS_ERROR("Pressed");
			}

			controls_updated = false;
		}*/
		
		//Keep Planning X direction until released
		while(control_signals[0] != 0)
		{
			ROS_ERROR("X Movement Only");

			//Distance of Movements
			double distance = 0.02;

			//Set Target Pose X Direction
			target_pose.pose.position.x = target_pose.pose.position.x + control_signals[0]*distance; //Only move x direction

			//Set waypoints for which to compute path
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(target_pose.pose);
			moveit_msgs::RobotTrajectory trajectory;
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			//Compute cartesian path
			double ret = move_group.computeCartesianPath(waypoints, (distance*0.5), 0, trajectory);

			//Execute trajectory synchronously 
			my_plan.trajectory_ = trajectory;
			move_group.execute(my_plan);
			sleep(0.1);
		}

		//Plan Y direction until released
		while(control_signals[1] != 0)
		{
			ROS_ERROR("Y Movement Only");

			//Distance of Movements
			double distance = 0.02;

			//Set Target Pose X Direction
			target_pose.pose.position.y = target_pose.pose.position.y + control_signals[1]*distance; //Only move y direction

			//Set waypoints for which to compute path
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(target_pose.pose);
			moveit_msgs::RobotTrajectory trajectory;
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			//Compute cartesian path
			double ret = move_group.computeCartesianPath(waypoints, (distance*0.5), 0, trajectory);

			//Execute trajectory synchronously 
			my_plan.trajectory_ = trajectory;
			move_group.execute(my_plan);
			sleep(0.1);
		}

		//Plan Z direciton until released
		while(control_signals[2] != 0)
		{
			ROS_ERROR("Z Movement Only");

			//Distance of Movements
			double distance = 0.02;

			//Set Target Pose Z Direction
			target_pose.pose.position.z = target_pose.pose.position.z + control_signals[2]*distance; //Only move z direction
			target_pose.pose.position.y = target_pose.pose.position.y + 0.001; //Offset drift

			//Set waypoints for which to compute path
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(target_pose.pose);
			moveit_msgs::RobotTrajectory trajectory;
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			//Compute cartesian path
			double ret = move_group.computeCartesianPath(waypoints, (distance*0.5), 0, trajectory);

			//Execute trajectory synchronously 
			my_plan.trajectory_ = trajectory;
			move_group.execute(my_plan);
			sleep(0.1);
		}

		//ROS Syntax
		r.sleep();
	}
	
	return 0;
}