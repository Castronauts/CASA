#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "dynamixel_msgs/JointState.h"

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

//---------------------------------------------------------------------------------------------------------------------------------------------
//Global Variables
//---------------------------------------------------------------------------------------------------------------------------------------------
float control_signals [3] = {0.0, 0.0, 0.0};
float gripper_signal = 0.0;
float gripper_load = 0.0;

//---------------------------------------------------------------------------------------------------------------------------------------------
//Function Declarations
//---------------------------------------------------------------------------------------------------------------------------------------------
void updateControlSignal(const geometry_msgs::Point::ConstPtr& msg)
{
	if(control_signals[0] != msg->x || control_signals[1] != msg->y || control_signals[2] != msg->z)
	{
		control_signals[0] = msg->x;
		control_signals[1] = msg->y;
		control_signals[2] = msg->z;
	}
}

void updateGripperSignal(const geometry_msgs::Point::ConstPtr & msg)
{
	if (gripper_signal != msg->x)
	{
		gripper_signal = msg->x; //1.0 means open, -1.0 means close
	}
}

void updateGripperLoad(const dynamixel_msgs::JointState msg)
{
	gripper_load = msg.load;
}

void moveToSpecifiedPose(moveit::planning_interface::MoveGroupInterface * group, int position_setting)
{
	if (position_setting == 0)
	{
		group->setNamedTarget("Claw_Start");
		group->move();
	}
	if (position_setting == 1)
	{
		group->setNamedTarget("Shelf_Start");
		group->move();
	}

	sleep(3.0);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface * planning, moveit::planning_interface::MoveGroupInterface * move)
{
	//Creating ground collision object being 2 x 2 x 0.04 meter box from center of robot
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move->getPlanningFrame();
	collision_object.id = "ground";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 2.0; //X
	primitive.dimensions[1] = 2.0; //Y
	primitive.dimensions[2] = 0.03; //Z

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

	sleep(2.0);
}

void screenTrajectory(moveit_msgs::RobotTrajectory* orig_traj)
{
	//tune this to prevent major configuration space changes (which sometimes don't properly screen collisions)
	float distance_cutoff = 0.5;
	std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator first = orig_traj->joint_trajectory.points.begin() + 0;
	std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator last = orig_traj->joint_trajectory.points.begin() + orig_traj->joint_trajectory.points.size();
	int i;
	for(i = 0; i < orig_traj->joint_trajectory.points.size() - 1; ++i)
	{
		float dist_sq = 0.0;
		int j;
		for(j = 0; j < orig_traj->joint_trajectory.points[i].positions.size(); ++j)
		{
			float dist_ = orig_traj->joint_trajectory.points[i+1].positions[j] - orig_traj->joint_trajectory.points[i].positions[j];
			dist_sq += dist_*dist_;
		}
		if(dist_sq > distance_cutoff)
		{
			int size_diff = orig_traj->joint_trajectory.points.size() - (i+1);
			ROS_ERROR("Clipping [%d] points out of trajectory", size_diff);
			last = orig_traj->joint_trajectory.points.begin() + (i+1);
			break;
		}
	}
	std::vector<trajectory_msgs::JointTrajectoryPoint> new_points(first, last);
	orig_traj->joint_trajectory.points = new_points;
}

void updateMovement(double distance, int setting, moveit::planning_interface::MoveGroupInterface * move, geometry_msgs::PoseStamped * target)
{
	//Set Target Pose Based on setting
	if (setting == 0) //X Setting
	{
		target->pose.position.x = target->pose.position.x + control_signals[0]*distance; //Only move x direction
	}
	if (setting == 1) //Y Setting
	{
		target->pose.position.y = target->pose.position.y + control_signals[1]*distance; //Only move y direction
	}
	if (setting == 2) //Z Setting
	{
		target->pose.position.z = target->pose.position.z + control_signals[2]*distance; //Only move z direction

		if (target->pose.position.z < 0.06) //Don't want it to get stuck in low position towards the ground so giving it a buffer
		{
			distance = distance * 2; //Offset what came down
			target->pose.position.z = target->pose.position.z + distance;
		}
	}

	//Set waypoints for which to compute path
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target->pose);
	moveit_msgs::RobotTrajectory trajectory;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	//Compute cartesian path
	double ret = move->computeCartesianPath(waypoints, (distance*0.5), 0, trajectory);

	//Once it returns zero need to keep it moving. Set target to current because it thinks its something based on target set above.
	//This returns zero when the arm cannot physically move the arm to that spot
	//Need to put extra warnings here to let user know
	//FIXME!!!!!!!!!!!!!!!!!
	if (ret == 0.0)
	{
		ROS_ERROR("Stuck");

		geometry_msgs::PoseStamped current_pose = move->getCurrentPose();

		target->pose.position.x = current_pose.pose.position.x;
		target->pose.position.y = current_pose.pose.position.y;
		target->pose.position.z = current_pose.pose.position.z;
	}

	//Check to make sure no crazy movements
	//screenTrajectory(&trajectory);

	//Execute trajectory synchronously 
	my_plan.trajectory_ = trajectory;
	move->execute(my_plan);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//Main Function
//---------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	//ROS Declarations
	ros::init(argc, argv, "cartesian_keyboard_controller");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(3); //Need 3 for updateControlSignal, updateGripperSignal, updateGripperLoad
	spinner.start();

	//Move It Declarations
	static const std::string PLANNING_GROUP = "Arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	robot_trajectory::RobotTrajectory rt_planner(move_group.getRobotModel(), PLANNING_GROUP);
	trajectory_processing::IterativeParabolicTimeParameterization time_planner;

	//Controller Topic Signals
	ros::Subscriber control_sub = nh.subscribe("control_signal", 10, updateControlSignal); //10 buffer message size might be better response time than 1
	ros::Subscriber gripper_sub_signal = nh.subscribe("gripper_signal", 10, updateGripperSignal);
	ros::Subscriber gripper_sub_load = nh.subscribe("/dual_gripper_controller/state", 10, updateGripperLoad);

	//Gripper publisher
	ros::Publisher gripper_pub = nh.advertise<std_msgs::Float64>("/dual_gripper_controller/command", 10);

	//Poses needed to be saved
	geometry_msgs::PoseStamped target_pose;

	//Add initial collision objects
	addCollisionObjects(&planning_scene_interface, &move_group);

	//Save initial behavior
	moveToSpecifiedPose(&move_group, 1); //Move to Position 1
	target_pose = move_group.getCurrentPose();

	//ROS Wait
	ros::Rate r(20);

	//Control Signal Settings
	int setting = 0; //0-X, 1-Y, 2-Z
	double distance = 0.015; //Meters

	//Gripper Settings
	std_msgs::Float64 gripper_distance;
	gripper_distance.data = -1.0;
	gripper_pub.publish(gripper_distance);

	while(ros::ok())
	{
		//First check gripper signal before anything
		while ((gripper_signal == 1.0) && (gripper_distance.data >= -1.0)) //Open
		{
			gripper_distance.data = gripper_distance.data - 0.01;
			gripper_pub.publish(gripper_distance);
			ros::Duration(0.02).sleep();
		}

		while((gripper_signal == -1.0) && (gripper_load >= -0.20) && (gripper_distance.data <= -0.1)) //Close Load Sensitive
		{
			gripper_distance.data = gripper_distance.data + 0.01;
			gripper_pub.publish(gripper_distance);
			ros::Duration(0.02).sleep();
		}

		//Put diagonal signal detection here before single direction


		//Keep Planning X direction until released
		while(control_signals[0] != 0)
		{
			//X Setting
			setting = 0;

			updateMovement(distance, setting, &move_group, &target_pose);
		}

		//Plan Y direction until released
		while(control_signals[1] != 0)
		{
			//Y Setting
			setting = 1;

			updateMovement(distance, setting, &move_group, &target_pose);
		}

		//Plan Z direciton until released
		while(control_signals[2] != 0)
		{
			//Z Setting
			setting = 2;

			updateMovement(distance, setting, &move_group, &target_pose);
		}

		//ROS Syntax
		r.sleep();
	}
	
	return 0;
}