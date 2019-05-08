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

float control_signals [3] = {0.0, 0.0, 0.0};
bool controls_updated = false;
int last_movement_axis = -1;
int trajectory_status = 0;

const int TRAJECTORY_INACTIVE = 0;
const int TRAJECTORY_PENDING = 1;
const int TRAJECTORY_ACTIVE = 2;


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

void updateTrajectoryGoal(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
	trajectory_status = TRAJECTORY_ACTIVE;
}

void updateTrajectoryStatus(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg)
{
	trajectory_status = TRAJECTORY_INACTIVE;
}

void getTarget(geometry_msgs::Pose* target_pose, geometry_msgs::Pose ideal_pose)
{
	target_pose->position.x = ideal_pose.position.x;
	target_pose->position.y = ideal_pose.position.y;
	target_pose->position.z = ideal_pose.position.z;
	if(control_signals[0] != 0.0)
	{
		target_pose->position.x += control_signals[0]*0.4;
		last_movement_axis = 0;
	}
	if(control_signals[1] != 0.0)
	{
		target_pose->position.y += control_signals[1]*0.4;
		last_movement_axis = 1;
	}
	if(control_signals[2] != 0.0)
	{
		target_pose->position.z += control_signals[2]*0.4;
		last_movement_axis = 2;
	}
}

void updateIdealPose(geometry_msgs::Pose* ideal_pose, geometry_msgs::Pose current_pose)
{
	if(last_movement_axis == 0)
	{
		ideal_pose->position.x = current_pose.position.x;
	} else if (last_movement_axis == 1)
	{
		ideal_pose->position.y = current_pose.position.y;
	} else if (last_movement_axis == 2)
	{
		ideal_pose->position.z = current_pose.position.z;
	}
	last_movement_axis = -1;
}

void screenTrajectory(moveit_msgs::RobotTrajectory* orig_traj)
{
	//tune this to prevent major configuration space changes (which sometimes don't properly screen collisions)
	float distance_cutoff = 3.0;
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

void moveToSpecifiedPose(moveit::planning_interface::MoveGroupInterface * group)
{
	group->setNamedTarget("Claw_Start");
	group->move();

	sleep(3.0);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface * planning, moveit::planning_interface::MoveGroupInterface * move)
{
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
	box_pose.position.x = 0;
	box_pose.position.y = 0;
	box_pose.position.z = 0;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	planning->addCollisionObjects(collision_objects);
	//move->attachObject(collision_object.id); Attaches to End Effector
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_keyboard_controller");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(3);
	spinner.start();

	static const std::string PLANNING_GROUP = "Arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	robot_trajectory::RobotTrajectory rt_planner(move_group.getRobotModel(), PLANNING_GROUP);
	trajectory_processing::IterativeParabolicTimeParameterization time_planner;

	ros::Subscriber control_sub = nh.subscribe("control_signal", 1, updateControlSignal);
	ros::Subscriber traj_received = nh.subscribe("armstrong_controller/follow_joint_trajectory/goal", 1, updateTrajectoryGoal);
	ros::Subscriber status_sub = nh.subscribe("armstrong_controller/follow_joint_trajectory/result", 1, updateTrajectoryStatus);

	addCollisionObjects(&planning_scene_interface, &move_group);

	moveToSpecifiedPose(&move_group); //Move to initial claw arm position

	ros::Rate r(20); //update check for new controls at 20 hz
	geometry_msgs::PoseStamped t_pose;
	geometry_msgs::PoseStamped ideal_pose;
	geometry_msgs::PoseStamped c_pose;

	//Save initial behavior
	t_pose = move_group.getCurrentPose();
	ideal_pose = move_group.getCurrentPose();

	while(ros::ok())
	{
		if(controls_updated) 
		{
			move_group.stop(); 
			c_pose = move_group.getCurrentPose();
			updateIdealPose(&(ideal_pose.pose), c_pose.pose);

			if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0))
			{
				//set target based on current pose and control inputs
				getTarget(&(t_pose.pose), ideal_pose.pose);
				
				// set waypoints for which to compute path
			    std::vector<geometry_msgs::Pose> waypoints;
			    waypoints.push_back(t_pose.pose);
				moveit_msgs::RobotTrajectory trajectory;
				moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			    // compute cartesian path
			    double ret = move_group.computeCartesianPath(waypoints, 0.01, 0, trajectory); //the two magic numbers here are allowed distance between points (meters) and configuration space jump distance (units?)
				
				//Set TimeStamps or pseudo velocity and acceleration
				rt_planner.setRobotTrajectoryMsg(*(move_group.getCurrentState()), trajectory);
				time_planner.computeTimeStamps(rt_planner, 0.05, 0.05); //Trajectory, velocity, acceleration
				rt_planner.getRobotTrajectoryMsg(trajectory);

				//Check jumps
				screenTrajectory(&trajectory);

				//Our way of sending trajectory to physical arm controllers
				my_plan.trajectory_ = trajectory;
				move_group.asyncExecute(my_plan); //This will plan whole trajectory while the user is pressing down the controls
				trajectory_status = TRAJECTORY_PENDING;
			} 
			else
			{
				// set waypoints for which to compute path
			    std::vector<geometry_msgs::Pose> waypoints;
			    waypoints.push_back(ideal_pose.pose);
				moveit_msgs::RobotTrajectory trajectory;
				moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			    // compute cartesian path
			    double ret = move_group.computeCartesianPath(waypoints, 0.01, 0, trajectory); //the two magic numbers here are allowed distance between points (meters) and configuration space jump distance (units?)
				screenTrajectory(&trajectory);
				ROS_ERROR("Fixing Path");

				//Our way of sending trajectory to physical arm controllers
				my_plan.trajectory_ = trajectory;
				move_group.execute(my_plan); //This will plan whole trajectory while the user is pressing down the controls
			}
			controls_updated = false;
		}

		r.sleep();
	}
	move_group.stop();

	return 0;
}