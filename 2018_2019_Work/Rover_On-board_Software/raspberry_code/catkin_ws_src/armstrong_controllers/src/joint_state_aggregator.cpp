#include <vector>
#include <string>
#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

//using namespace std;

class JointStateAggregator
{

public:
    JointStateAggregator()
    {
        private_nh = ros::NodeHandle("~"); //Private parameter node handler
        private_nh.param<int>("rate", publish_rate, 50); //Declare node handler object
    }
    
    bool initialize()
    {
        XmlRpc::XmlRpcValue val; //Retrieving lists
        std::vector<std::string> static_joints;
        
        if (private_nh.getParam("static_joints", val))
        {
            if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("static_joints parameter is not a list");
                return false;
            }
            
            for (int i = 0; i < val.size(); ++i)
            {
                static_joints.push_back(static_cast<std::string>(val[i]));
            }
            
        }
        
        std::vector<std::string> controller_names;
        
        if (private_nh.getParam("controllers", val))
        {
            if (val.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("controllers parameter is not a list");
                return false;
            }
            
            for (int i = 0; i < val.size(); ++i)
            {
                controller_names.push_back(static_cast<std::string>(val[i]));
            }
        }
        
        int num_static = static_joints.size();
        int num_controllers = controller_names.size();
        int num_total = num_static + num_controllers;
        
        msg.name.resize(num_total);
        msg.position.resize(num_total);
        msg.velocity.resize(num_total);
        msg.effort.resize(num_total);
        
        for (int i = 0; i < num_static; ++i)
        {
            msg.name[i] = static_joints[i];
            msg.position[i] = 0.0;
            msg.velocity[i] = 0.0;
            msg.effort[i] = 0.0;
        }
        
        controller_state_subs.resize(num_controllers);
        
        for (int i = 0; i < num_controllers; ++i)
        {
            controller_state_subs[i] =
				nh.subscribe<dynamixel_msgs::JointState>(controller_names[i] +  "/state", 100, boost::bind(&JointStateAggregator::processControllerState, this, _1, i+num_static));
        }
        
        for (int i = 0; i < num_controllers; ++i)
        {
            ros::topic::waitForMessage<dynamixel_msgs::JointState>(controller_names[i] +  "/state");
        }
        
        joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100); 
        
        return true;
    }
    
    void processControllerState(const dynamixel_msgs::JointStateConstPtr& msg_temp, int i)
    {
        if (i == 6) //Gripper controller needs to be flipped
        {
            msg.name[i] = msg_temp->name;
            msg.position[i] = (-1) * msg_temp->current_pos;
            msg.velocity[i] = msg_temp->velocity;
            msg.effort[i] = msg_temp->load;
        }
        else
        {
            msg.name[i] = msg_temp->name;
            msg.position[i] = msg_temp->current_pos;
            msg.velocity[i] = msg_temp->velocity;
            msg.effort[i] = msg_temp->load;
        }
    }
    
    void start()
    {
        ros::Rate loop_rate(publish_rate);
        
        while (ros::ok())
        {
            msg.header.stamp = ros::Time::now();
            joint_states_pub.publish(msg);
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::V_Subscriber controller_state_subs;
    ros::Publisher joint_states_pub;
    
    int publish_rate;
    sensor_msgs::JointState msg;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joint_state_aggregator"); //Declare name of node
    JointStateAggregator jsa; //Make a aggregator object jsa
    
    if (!jsa.initialize()) //Run initialize function
    {
        ROS_ERROR("JointStateAggregator failed to initialize");
        return 1;
    }
    
    jsa.start(); //Now run start function and stay there
    
    return 0;
}
