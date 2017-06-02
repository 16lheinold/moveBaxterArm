#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "baxter_core_msgs/SolvePositionIK.h"  
#include "baxter_core_msgs/EndpointState.h"
#include "std_msgs/Header.h"
#include "moveBaxterArm/goTo.h"

geometry_msgs::Point initialPosition;
geometry_msgs::Point difference;
geometry_msgs::Quaternion initialQ;
bool posStarted = false;
bool goToStarted = false;

void positionCallBack(const baxter_core_msgs::EndpointState::ConstPtr &msg);
void goToCallback(const moveBaxterArm::goTo::ConstPtr &msg);

int move(baxter_core_msgs::JointCommand* message, ros::ServiceClient* client);

int main(int argc, char* argv[]){
	ros::init(argc, argv, "rightArmMover"); //Init
	ros::NodeHandle n("~"); //Nodehandle
	ros::Rate loop_rate(10); //loop rate

	ros::Publisher armPub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000); //publisher
	ros::Subscriber positionSub = n.subscribe("/robot/limb/right/endpoint_state", 1000, positionCallBack); //sub - position
	ros::Subscriber goToSub = n.subscribe("/goTo", 1000, goToCallback);
	ros::ServiceClient client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
	baxter_core_msgs::JointCommand _message;
	while(ros::ok()){
		if(posStarted && goToStarted){
			baxter_core_msgs::JointCommand message;
			move(&message, &client); 
			posStarted = false;
			goToStarted = false;
			_message = message;
		}
		armPub.publish(_message);
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

int move(baxter_core_msgs::JointCommand* message, ros::ServiceClient* client){
	ros::Rate loop_rate(10); //loop rate

	message->mode = message->POSITION_MODE;

	message->names.push_back("right_s0");
	message->names.push_back("right_s1");
	message->names.push_back("right_e0");
	message->names.push_back("right_e1");
	message->names.push_back("right_w0");
	message->names.push_back("right_w1");
	message->names.push_back("right_w2");

	baxter_core_msgs::SolvePositionIK srv;

	std_msgs::Header finalHeader;
	finalHeader.stamp = ros::Time::now();
	finalHeader.frame_id = "world";

	geometry_msgs::PoseStamped finalPoseStamped;
	geometry_msgs::Pose finalPose;
	geometry_msgs::Point finalPosition;
	geometry_msgs::Quaternion finalQ;

	finalQ = initialQ;
	
	finalPosition.z = initialPosition.z + difference.z;
	finalPosition.y = initialPosition.y + difference.y;
	finalPosition.x = initialPosition.x + difference.x;

	ROS_INFO("Final positions: %.2lf, %.2lf, %.2lf", finalPosition.x, finalPosition.y, finalPosition.z);
	
	finalPose.position = finalPosition;
	finalPose.orientation = finalQ;
	finalPoseStamped.header = finalHeader;
	finalPoseStamped.pose = finalPose;

	srv.request.pose_stamp.push_back(finalPoseStamped);

	if(ros::service::waitForService("/ExternalTools/right/PositionKinematicsNode/IKService", 5.0)){
		if((*client).call(srv)){
			//find response by rossrv show SolvePositionIK
			ROS_INFO("Names: ");
			if(!srv.response.isValid[0]){
				ROS_ERROR("Couldn't find valid solution :(");
				return 1;
			}
			for(int i = 0; i < 7; i++){
				ROS_INFO("%s, ", srv.response.joints.at(0).name[i].c_str());
			}
			ROS_INFO("\nPosition: ");
			for(int i = 0; i < 7; i++){
				ROS_INFO("%.3lf, ", srv.response.joints.at(0).position[i]);
			}
			for(int i = 0; i < 7; i++){
				message->command.push_back(srv.response.joints.at(0).position[i]);
			}
		} else {
			ROS_ERROR("Failed to call service :(");
			return 1;
		}
	} else {
		ROS_ERROR("Failed to wait for service");
		return 1;
	}
}

void positionCallBack(const baxter_core_msgs::EndpointState::ConstPtr &msg){
	posStarted = true;
	initialPosition.x = msg->pose.position.x;
	initialPosition.y = msg->pose.position.y;
	initialPosition.z = msg->pose.position.z;

	initialQ.x = msg->pose.orientation.x;
	initialQ.y = msg->pose.orientation.y;
	initialQ.z = msg->pose.orientation.z;
	initialQ.w = msg->pose.orientation.w;
}

void goToCallback(const moveBaxterArm::goTo::ConstPtr &msg){
	goToStarted = true;
	ROS_INFO("go to callback");
	difference.x = msg->x;
	difference.y = msg->y;
	difference.z = msg->z;
}