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
float arlo_signals [2] = {0.0, 0.0};
bool arlo_set = false;
float gripper_signal = 0.0;
float gripper_load = 0.0;
std_msgs::String arlo_speed;
int control_change = 0;

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
		control_change = 1;
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

void updateArloSpeed(const std_msgs::String msg)
{
	arlo_speed.data = msg.data; 
}

void updateArloControl(const geometry_msgs::Point::ConstPtr & msg)
{
	if(arlo_signals[0] != msg->x || arlo_signals[1] != msg->y)
	{
		arlo_signals[0] = msg->x;
		arlo_signals[1] = msg->y;
		arlo_set = true;
	}
}

void moveToSpecifiedPose(moveit::planning_interface::MoveGroupInterface * group, int position_setting)
{
	if (position_setting == 0)
	{
		group->setNamedTarget("Claw_Start");
		group->move();
	}

	sleep(5.0);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface * planning, moveit::planning_interface::MoveGroupInterface * move)
{
	//Creating ground collision object being 2 x 2 x 0.03 meter box from center of robot
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

	sleep(2.0);
}

void updateTargetPose(geometry_msgs::PoseStamped * target, moveit::planning_interface::MoveGroupInterface * move)
{
	//Get what state it is left at
	geometry_msgs::PoseStamped temp_pose;
	/*temp_pose = move->getCurrentPose();

	//Make sure the orientation stays the same
	temp_pose.pose.orientation = target->pose.orientation;

	//Move to that orientation
	move->setPoseTarget(temp_pose.pose);
	move->move();*/

	//Once moved then set the target position to begin again
	temp_pose = move->getCurrentPose();
	target->pose.position = temp_pose.pose.position;
}

void getTargetPosition(int * setting, geometry_msgs::PoseStamped * target)
{
	if(control_signals[0] != 0.0)
	{
		target->pose.position.x = control_signals[0]*0.2;
		*setting = 0;
	}
	else if(control_signals[1] != 0.0)
	{
		if (control_signals[1] > 0)
		{
			target->pose.position.y = 0.54;
		}
		else
		{
			target->pose.position.y = 0.36;
		}
		*setting = 1;
	}
	else if(control_signals[2] != 0.0)
	{
		if (control_signals[2] > 0)
		{
			target->pose.position.z = 0.35;
		}
		else
		{
			target->pose.position.z = 0.06;
		}
		*setting = 2;
	}
}

void computeStraightTrajectory(geometry_msgs::PoseStamped * target, moveit::planning_interface::MoveGroupInterface * move, double * ret, moveit_msgs::RobotTrajectory * trajectory)
{
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target->pose);

	*ret = move->computeCartesianPath(waypoints, 0.01, 0, *trajectory);
}



//---------------------------------------------------------------------------------------------------------------------------------------------
//Main Function
//---------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	//ROS Declarations
	ros::init(argc, argv, "cartesian_keyboard_controller");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2); //2 working for all signals
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
	ros::Subscriber arlo_sub = nh.subscribe("speed_arlo", 10, updateArloSpeed);
	ros::Subscriber arlo_keys = nh.subscribe("arlo_signal", 10, updateArloControl);

	//Publisher topics
	ros::Publisher gripper_pub = nh.advertise<std_msgs::Float64>("/dual_gripper_controller/command", 10);
	ros::Publisher arlo_pub = nh.advertise<std_msgs::String>("arlo_wheels", 10);

	//Poses needed to be saved
	geometry_msgs::PoseStamped target_pose;

	//Save initial behavior
	moveToSpecifiedPose(&move_group, 0); //Move to position 0
	target_pose = move_group.getCurrentPose();  

	//Add initial collision objects
	addCollisionObjects(&planning_scene_interface, &move_group);

	//ROS Wait
	ros::Rate r(20);

	//Control Signal Settings
	int setting = -1; //0-X, 1-Y, 2-Z

	//Gripper Settings
	std_msgs::Float64 gripper_distance;
	std_msgs::String arlo_string;
	gripper_distance.data = 0.0;
	arlo_string.data = "rst\r";
	arlo_speed.data = 10.0;
	gripper_pub.publish(gripper_distance);
	arlo_pub.publish(arlo_string);

	while(ros::ok())
	{
		//Check Arlo Signals
		if(arlo_set)
		{
			arlo_set = false;

			if (arlo_signals[0] == 0 && arlo_signals[1] == 0)
			{
				arlo_string.data = "go 0 0\r";
				arlo_pub.publish(arlo_string);
			}
			else
			{
				//Check Forward or Backward
				if (arlo_signals[0] > 0) //Forward
				{
					arlo_string.data = "gospd " + arlo_speed.data + " " + arlo_speed.data + "\r";
					arlo_pub.publish(arlo_string);
				}
				else if(arlo_signals[0] < 0) //Backward
				{
					arlo_string.data = "gospd -" + arlo_speed.data + " -" + arlo_speed.data + "\r";
					arlo_pub.publish(arlo_string);
				}

				//Check Left or Right
				if (arlo_signals[1] > 0) //Left
				{
					arlo_string.data = "gospd -" + arlo_speed.data + " " + arlo_speed.data + "\r";
					arlo_pub.publish(arlo_string);
				}
				else if (arlo_signals[1] < 0) //Right
				{
					arlo_string.data = "gospd " + arlo_speed.data + " -" + arlo_speed.data + "\r";
					arlo_pub.publish(arlo_string);
				}
			}
		}

		//Check gripper signal
		while (gripper_signal == 1.0) //Open
		{
			gripper_distance.data = gripper_distance.data - 0.01;
			gripper_pub.publish(gripper_distance);
			ros::Duration(0.02).sleep();
		}

		while((gripper_signal == -1.0) && (gripper_load >= -0.15)) //Close Load Sensitive
		{
			gripper_distance.data = gripper_distance.data + 0.01;
			gripper_pub.publish(gripper_distance);
			ros::Duration(0.02).sleep();
		}

		//Check arm movement/joystick
		if (control_change)
		{
			control_change = 0;
			move_group.stop();
			
			//Correct position and move it if any misalignments
			ros::Duration(0.1).sleep(); //Wait so move group can calculate its position quick enough
			updateTargetPose(&(target_pose), &(move_group));
			
			if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0))
			{
				double ret; //Return number from compute
				moveit::planning_interface::MoveGroupInterface::Plan my_plan; //Plan for final move
				moveit_msgs::RobotTrajectory trajectory; //Straight path trajectory, might need constraints

				//Set correct target position based on which control signal is set
				getTargetPosition(&setting, &target_pose);

				//Compute straight trajectory and set it in trajectory and set ret (i.e shouldnt be 0)
				computeStraightTrajectory(&target_pose, &move_group, &ret, &trajectory);

				if (ret != 0) //Need to reposition arm to keep moving
				{
					ROS_ERROR("RET[%lf]", ret);
					//Set TimeStamps or pseudo velocity and acceleration
					rt_planner.setRobotTrajectoryMsg(*(move_group.getCurrentState()), trajectory);
					time_planner.computeTimeStamps(rt_planner, 0.05, 0.05); //Trajectory, velocity, acceleration
					rt_planner.getRobotTrajectoryMsg(trajectory);

					//Set plan and execute
					my_plan.trajectory_ = trajectory;
					move_group.asyncExecute(my_plan);
				}
			}

		}

		//ROS Syntax
		r.sleep();
	}
	
	return 0;
}