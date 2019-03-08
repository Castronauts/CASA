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

#define forward_limit 0.48
#define backward_limit 0.42
#define left_right_limit 0.08
#define up_limit 0.3
#define down_limit 0.06
#define MAX_LOAD 0.37
#define PLANNING_GROUP "Arm"

//Arm Controller Object
class ArmController
{
	public:
		ArmController():
			spinner(3),
			move_group(PLANNING_GROUP),
			rt_planner(move_group.getRobotModel(), PLANNING_GROUP)
		{
			//ROS
			spinner.start();
			setting = -1;
			control_signals [0] = 0.0;
			control_signals [1] = 0.0;
			control_signals [2] = 0.0;
			control_change = 0.0;
			stop_movement = -1;

			//Move It Declarations
			joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

			//Controller Topic Signals
			control_sub = nh.subscribe("control_signal", 10, &ArmController::updateControlSignal, this); //10 buffer message size might be better response time than 1
			joint_2 = nh.subscribe("joint2_controller/state", 10, &ArmController::checkJoint2Load, this);
			//joint_3 = nh.subscribe("joint3_controller/state", 10, &ArmController::checkJoint3Load, this);
			//joint_5 = nh.subscribe("joint5_controller/state", 10, &ArmController::checkJoint5Load, this);

			//Arm publisher for gui and xbox
			x_gui = nh.advertise<std_msgs::Int64>("x_gui", 10);
			y_gui = nh.advertise<std_msgs::Int64>("y_gui", 10);
			z_gui = nh.advertise<std_msgs::Int64>("z_gui", 10);

			xbox_vibrate = nh.advertise<std_msgs::Int64>("xbox_vibrate", 10);

			//Save initial behavior
			moveToSpecifiedPose(0); //Move to position 0
			target_pose = move_group.getCurrentPose();

			//Add initial collision objects
			addCollisionObjects();
		}

		void runROSLoop()
		{
			//ROS Wait
			ros::Rate r(50);

			//Control Signal Settings
			std_msgs::Int64 gui_msg;
			double y_conversion = forward_limit - backward_limit;
			double x_conversion = left_right_limit*2.0;
			double z_conversion = up_limit - down_limit;
			double temp = 0.0;
			int gui_control = 0;
			bool xbox_sent = false;

			while(!(ros::isShuttingDown()))
			{
				//Check arm movement/joystick
				if (control_change)
				{
					control_change = 0;
					move_group.stop();

					//Correct position and move it if any misalignments
					ros::Duration(0.1).sleep(); //Wait so move group can calculate its position quick enough
					updateTargetPose();

					if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0) && (stop_movement == -1))
					{
						double ret; //Return number from compute
						moveit::planning_interface::MoveGroupInterface::Plan my_plan; //Plan for final move
						moveit_msgs::RobotTrajectory trajectory; //Straight path trajectory, might need constraints

						//Set correct target position based on which control signal is set
						getTargetPosition();

						//Compute straight trajectory and set it in trajectory and set ret (i.e shouldnt be 0)
						computeStraightTrajectory(&ret, &trajectory);

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

					if ((gui_control >= 98 or gui_control <= 2))
					{
						//Vibrate setting
						gui_msg.data = 1;
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

					if ((gui_control >= 98 or gui_control <= 2))
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

					if ((gui_control >= 98 or gui_control <= 2))
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
		}

	private:
		float control_signals [3];
		int control_change;
		int stop_movement;
		int setting;

		//ROS Declarations
		ros::NodeHandle nh;
		ros::AsyncSpinner spinner; //2 working for all signals

		//Move It Declarations
		moveit::planning_interface::MoveGroupInterface move_group;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		const robot_state::JointModelGroup * joint_model_group;
		robot_trajectory::RobotTrajectory rt_planner;
		trajectory_processing::IterativeParabolicTimeParameterization time_planner;

		//Controller Topic Signals
		ros::Subscriber control_sub; //10 buffer message size might be better response time than 1
		ros::Subscriber joint_2;
		//ros::Subscriber joint_3;
		//ros::Subscriber joint_5;

		//Arm publisher for gui and xbox
		ros::Publisher x_gui;
		ros::Publisher y_gui;
		ros::Publisher z_gui;
		ros::Publisher xbox_vibrate;

		//Poses needed to be saved
		geometry_msgs::PoseStamped target_pose;
		geometry_msgs::PoseStamped current_pose;

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

		void moveToSpecifiedPose(int position_setting)
		{
			if (position_setting == 0)
			{
				move_group.setNamedTarget("Claw_Start");
				move_group.move();
			}

			sleep(5.0);
		}

		void addCollisionObjects()
		{
			//Creating ground collision object being 2 x 2 x 0.03 meter box from center of robot
			moveit_msgs::CollisionObject collision_object;
			collision_object.header.frame_id = move_group.getPlanningFrame();
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

			planning_scene_interface.addCollisionObjects(collision_objects);

			sleep(2.0);
		}

		void updateTargetPose()
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
			temp_pose = move_group.getCurrentPose();
			target_pose.pose.position = temp_pose.pose.position;
		}

		void getTargetPosition()
		{
			if(control_signals[0] != 0.0)
			{
				if (control_signals[0] > 0) //Right
				{
					target_pose.pose.position.x = control_signals[0]*left_right_limit;
				}
				else if(control_signals[0] < 0) //Left
				{
					target_pose.pose.position.x = control_signals[0]*left_right_limit;
				}
				setting = 0;
			}
			else if(control_signals[1] != 0.0)
			{
				if (control_signals[1] > 0) //Forward
				{
					target_pose.pose.position.y = forward_limit;
				}
				else if(control_signals[1] < 0) //Backward
				{
					target_pose.pose.position.y = backward_limit;
				}
				setting = 1;
			}
			else if(control_signals[2] != 0.0)
			{
				if (control_signals[2] > 0) //Up
				{
					target_pose.pose.position.z = up_limit;
				}
				else if(control_signals[2] < 0) //Down
				{
					target_pose.pose.position.z = down_limit;
				}
				setting = 2;
			}
		}

		void computeStraightTrajectory(double * ret, moveit_msgs::RobotTrajectory * trajectory)
		{
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(target_pose.pose);

			*ret = move_group.computeCartesianPath(waypoints, 0.01, 0, *trajectory);
		}

		int checkLoadCommand()
		{
			int signal = 5; //Default to down as thats what we dont want to push

			if(control_signals[0] != 0.0)
			{
				if (control_signals[0] > 0) //Right
				{
					signal = 0;
				}
				else //Left
				{
					signal = 1;
				}
			}
			else if(control_signals[1] != 0.0)
			{
				if (control_signals[1] > 0)//Forward
				{
					signal = 2;
				}
				else //Backward
				{
					signal = 3;
				}
			}
			else if(control_signals[2] != 0.0)
			{
				if (control_signals[2] > 0) //Up
				{
					signal = 4;
				}
				else //Down
				{
					signal = 5;
				}
			}

			return signal;
		}

		void correctLoadPosition()
		{
			if(stop_movement == 0) //Right
			{
				target_pose.pose.position.x = target_pose.pose.position.x - 0.02; //Move left then
			}
			else if (stop_movement == 1) //Left
			{
				target_pose.pose.position.x = target_pose.pose.position.x + 0.02; //Move right then
			}
			else if (stop_movement == 2) //Forward
			{
				target_pose.pose.position.y = target_pose.pose.position.y - 0.02; //Move backward then
			}
			else if (stop_movement == 3) //Backward
			{
				target_pose.pose.position.y = target_pose.pose.position.y + 0.02; //Move forward then
			}
			else if (stop_movement == 4) //Up
			{
				target_pose.pose.position.z = target_pose.pose.position.z - 0.1; //Move down then
			}
			else if (stop_movement == 5) //Down
			{
				target_pose.pose.position.z = target_pose.pose.position.z + 0.1; //Move up then
			}

		}

		void checkJoint2Load(dynamixel_msgs::JointState msg)
		{
			std_msgs::Int64 gui_msg;
			double load = std::abs(msg.load);

			if ((load >= MAX_LOAD) && (stop_movement == -1)) //Max load reached
			{
				//Check what command
				stop_movement = checkLoadCommand();

				//Wait because it could be spikes
				ros::Duration(0.5).sleep();

				if(load >= MAX_LOAD) //Check again after waiting to see if it is actually an issue and not just a millisecond spike
				{
					//Stop move group
					move_group.stop();
					ros::Duration(0.1).sleep();

					//Now move in opposite direction of the last recorded command until load is fixed
					double ret;
					moveit::planning_interface::MoveGroupInterface::Plan my_plan;
					moveit_msgs::RobotTrajectory trajectory;

					//Set correct target position based on which control signal is set
					correctLoadPosition();

					//Compute straight trajectory and set it in trajectory and set ret (i.e shouldnt be 0)
					computeStraightTrajectory(&ret, &trajectory);

					if (ret != 0) //Need to reposition arm to keep moving
					{
						//Set TimeStamps or pseudo velocity and acceleration
						rt_planner.setRobotTrajectoryMsg(*(move_group.getCurrentState()), trajectory);
						time_planner.computeTimeStamps(rt_planner, 0.5, 0.5); //Trajectory, velocity, acceleration
						rt_planner.getRobotTrajectoryMsg(trajectory);

						//Set plan and execute
						my_plan.trajectory_ = trajectory;
						move_group.execute(my_plan);
						ROS_ERROR("Fixed load");
					}

					//Vibrate
					gui_msg.data = 2; //Vibrate setting
					xbox_vibrate.publish(gui_msg);
					ros::Duration(0.75).sleep();

					stop_movement = -1;
				}
				else
				{
					stop_movement = -1;
				}
			}
		}

		/*void checkJoint3Load(dynamixel_msgs::JointState msg)
		{
			std_msgs::Int64 gui_msg;
			double load = std::abs(msg.load);

			if (load >= MAX_LOAD) //Max load reached
			{
				//Stop move group
				move_group.stop();
				ros::Duration(0.1).sleep(); //Wait so move group can calculate its position quick enough

				//Check what command
				stop_movement = checkLoadCommand();

				ROS_ERROR("Joint3[%d]", stop_movement);

				//Vibrate
				gui_msg.data = 2; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.75).sleep();
			}

			if (load >= PLACE_LOAD) //Just vibrate for placement onto ground
			{
				//Vibrate
				gui_msg.data = 2; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.75).sleep();
			}

			if (load < PLACE_LOAD)
			{
				stop_movement = -1; //Free up movement now
			}
		}

		void checkJoint5Load(dynamixel_msgs::JointState msg)
		{
			std_msgs::Int64 gui_msg;
			double load = std::abs(msg.load);

			if (load >= MAX_LOAD) //Max load reached
			{
				//Stop move group
				move_group.stop();
				ros::Duration(0.1).sleep(); //Wait so move group can calculate its position quick enough

				//Check what command
				stop_movement = checkLoadCommand();

				ROS_ERROR("Joint5[%d]", stop_movement);

				//Vibrate
				gui_msg.data = 2; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.75).sleep();
			}

			if (load >= PLACE_LOAD) //Just vibrate for placement onto ground
			{
				//Vibrate
				gui_msg.data = 2; //Vibrate setting
				xbox_vibrate.publish(gui_msg);
				ros::Duration(0.75).sleep();
			}

			if (load < PLACE_LOAD)
			{
				stop_movement = -1; //Free up movement now
			}
		}*/
};


//---------------------------------------------------------------------------------------------------------------------------------------------
//Main Function
//---------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_controller");

	ArmController controller;

	controller.runROSLoop();

	return 0;
}
