#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Int64.h>
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

#define forward_limit 0.50
#define backward_limit 0.36
#define left_right_limit 0.2
#define up_limit 0.3
#define down_limit 0.06

//---------------------------------------------------------------------------------------------------------------------------------------------
//Global Variables
//---------------------------------------------------------------------------------------------------------------------------------------------
float control_signals [3] = {0.0, 0.0, 0.0};
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
		target->pose.position.x = control_signals[0]*left_right_limit;
		*setting = 0;
	}
	else if(control_signals[1] != 0.0)
	{
		if (control_signals[1] > 0)
		{
			target->pose.position.y = forward_limit;
		}
		else
		{
			target->pose.position.y = backward_limit;
		}
		*setting = 1;
	}
	else if(control_signals[2] != 0.0)
	{
		if (control_signals[2] > 0)
		{
			target->pose.position.z = up_limit;
		}
		else
		{
			target->pose.position.z = down_limit;
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

	//Arm publisher for gui and xbox
	ros::Publisher x_gui = nh.advertise<std_msgs::Int64>("x_gui", 10);
	ros::Publisher y_gui = nh.advertise<std_msgs::Int64>("y_gui", 10);
	ros::Publisher z_gui = nh.advertise<std_msgs::Int64>("z_gui", 10);
	ros::Publisher xbox_vibrate = nh.advertise<std_msgs::Int64>("xbox_vibrate", 10);

	//Poses needed to be saved
	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::PoseStamped current_pose;

	//Save initial behavior
	moveToSpecifiedPose(&move_group, 0); //Move to position 0
	target_pose = move_group.getCurrentPose();

	//Add initial collision objects
	addCollisionObjects(&planning_scene_interface, &move_group);

	//ROS Wait
	ros::Rate r(20);

	//Control Signal Settings
	int setting = -1; //0-X, 1-Y, 2-Z
	std_msgs::Int64 gui_msg;
	double y_conversion = forward_limit - backward_limit;
	double x_conversion = left_right_limit*2.0;
	double z_conversion = up_limit - down_limit;
	double temp = 0.0;
	int gui_control = 0;
	bool xbox_sent = false;

	while(ros::ok())
	{
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
					//Set TimeStamps or pseudo velocity and acceleration
					rt_planner.setRobotTrajectoryMsg(*(move_group.getCurrentState()), trajectory);
					time_planner.computeTimeStamps(rt_planner, 0.1, 0.1); //Trajectory, velocity, acceleration
					rt_planner.getRobotTrajectoryMsg(trajectory);

					//Set plan and execute
					my_plan.trajectory_ = trajectory;
					move_group.asyncExecute(my_plan);
				}
			}
			else
			{
				//So it doesn't keep publishing to gui for no reason
				setting = -1;
				xbox_sent = false;
			}

		}

		//Constantly send updated position to gui
		current_pose = move_group.getCurrentPose();

		if (setting == 0) //Update x gui
		{
			temp = current_pose.pose.position.x;
			temp = temp + left_right_limit; //Accounting for negative
			gui_control = (int)((temp/(left_right_limit*2)) * 100.0);

			if ((gui_control > 99 or gui_control < 1))
			{
				gui_msg.data = 1; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.5).sleep();
				xbox_sent = true;
			}

			gui_msg.data = gui_control;
			x_gui.publish(gui_msg);
		}
		else if (setting == 1) //Update y gui
		{
			temp = current_pose.pose.position.y;
			gui_control = (int)(((temp-backward_limit)/y_conversion) * 100.0);

			if ((gui_control > 99 or gui_control < 1))
			{
				gui_msg.data = 1; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.5).sleep();
				xbox_sent = true;
			}

			gui_msg.data = gui_control;
			y_gui.publish(gui_msg);
		}
		else if (setting == 2) //Update z gui
		{
			temp = current_pose.pose.position.z;
			gui_control = (int)(((temp-down_limit)/z_conversion) * 100);

			if ((gui_control > 99 or gui_control < 1))
			{
				gui_msg.data = 1; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.5).sleep();
				xbox_sent = true;
			}

			gui_msg.data = gui_control;
			z_gui.publish(gui_msg);
		}

		//ROS Syntax
		r.sleep();
	}

	return 0;
}
