#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "geometry_msgs/Pose.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "ur_kinematics/ur_kin.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
int failedtf = 0;
sensor_msgs::JointState joint_states;
std_srvs::Trigger begin_comp;
int service_call_succeeded;
tf2_ros::Buffer tfBuffer;
osrf_gear::LogicalCameraImage first_image;
osrf_gear::GetMaterialLocations find_bins;
osrf_gear::Order first_order;
osrf_gear::Shipment first_shipment;
osrf_gear::Product first_product;
std::vector<osrf_gear::Model> first_model;
std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_camera_bin_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_agv_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_quality_vector;
std::vector<trajectory_msgs::JointTrajectoryPoint> points;

void chatterCallback(const osrf_gear::Order::ConstPtr& orders){
  order_vector.push_back(*orders);
}
void logicalCameraBinVectorCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int cam_num){
  logic_camera_bin_vector[cam_num] = *msg;
}
void logicalCameraAgvVectorCallback(const osrf_gear::LogicalCameraImage::ConstPtr& agv_msg, int cam_num){
  logic_agv_vector[cam_num] = *agv_msg;
}
void logicalCameraQualVectorCallback(const osrf_gear::LogicalCameraImage::ConstPtr& qual_msg, int cam_num){
  logic_quality_vector[cam_num] = *qual_msg;
}
void logicbin1CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& bin_msg){
  logicalCameraBinVectorCallback(bin_msg, 0);
}
void logicbin2CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& bin_msg){
  logicalCameraBinVectorCallback(bin_msg, 1);
}
void logicbin3CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& bin_msg){
  logicalCameraBinVectorCallback(bin_msg, 2);
}
void logicbin4CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& bin_msg){
  logicalCameraBinVectorCallback(bin_msg, 3);
}
void logicbin5CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& bin_msg){
  logicalCameraBinVectorCallback(bin_msg, 4);
}
void logicbin6CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& bin_msg){
  logicalCameraBinVectorCallback(bin_msg, 5);
}
void logicagv1CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& agv_msg){
  logicalCameraAgvVectorCallback(agv_msg, 0);
}
void logicagv2CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& agv_msg){
  logicalCameraAgvVectorCallback(agv_msg, 1);
}
void logicQuality1CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& qual_msg){
  logicalCameraQualVectorCallback(qual_msg, 0);
}
void logicQuality2CameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& qual_msg){
  logicalCameraQualVectorCallback(qual_msg, 1);
}
void jointCB(const sensor_msgs::JointState::ConstPtr& joint_msgs){
  joint_states = *joint_msgs;
}
int main(int argc, char **argv)
  {
  ros::init(argc, argv, "trigger_subscriber_node");
  
  ros::NodeHandle n;
  order_vector.clear();
  first_model.clear();
  logic_camera_bin_vector.clear();
  logic_camera_bin_vector.resize(6);
  logic_agv_vector.clear();
  logic_agv_vector.resize(2);
  logic_quality_vector.clear();
  logic_quality_vector.resize(2);
  points.clear();
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

  ros::Publisher arm_command_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
  
  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, chatterCallback);

  ros::Subscriber logical_camera_subscriber_agv1 = n.subscribe("/ariac/logical_camera_agv1", 10, logicagv1CameraCallback);
  ros::Subscriber logical_camera_subscriber_agv2 = n.subscribe("/ariac/logical_camera_agv2", 10, logicagv2CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin1 = n.subscribe("/ariac/logical_camera_bin1", 10, logicbin1CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin2 = n.subscribe("/ariac/logical_camera_bin2", 10, logicbin2CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin3 = n.subscribe("/ariac/logical_camera_bin3", 10, logicbin3CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin4 = n.subscribe("/ariac/logical_camera_bin4", 10, logicbin4CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin5 = n.subscribe("/ariac/logical_camera_bin5", 10, logicbin5CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin6 = n.subscribe("/ariac/logical_camera_bin6", 10, logicbin6CameraCallback);
  ros::Subscriber logical_camera_subscriber_quality_control_sensor1 = n.subscribe("/ariac/quality_control_sensor_1", 10, logicQuality1CameraCallback);
  ros::Subscriber logical_camera_subscriber_quality_control_sensor2 = n.subscribe("/ariac/quality_control_sensor_2", 10, logicQuality2CameraCallback);
  ros::Subscriber joint_states_h = n.subscribe("ariac/arm1/joint_states", 10, jointCB);

  while(ros::ok() && !tfBuffer.canTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0,0), ros::Duration(4.0))){
  failedtf++;
}
  ros::ServiceClient request_bin = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  service_call_succeeded = begin_client.call(begin_comp);
	if (service_call_succeeded == 0){
	  ROS_ERROR("Competition service call failed!");
          ros::shutdown();
        }
	else{
	  if(begin_comp.response.success){
          ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
          }
          else{
	  ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
            if(strcmp(begin_comp.response.message.c_str(), "cannot start if not in 'init' state")){
              ros::shutdown();
            }
          }
       }
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok()){
    if(order_vector.size() > 0){
      geometry_msgs::TransformStamped tfStamped;
      geometry_msgs::TransformStamped tfActuator_Stamped;
      geometry_msgs::TransformStamped tfWorld_Stamped;
      std::string bin_frame;
      first_order = order_vector.front();
      first_shipment = first_order.shipments.front();
      first_product = first_shipment.products.front();
      find_bins.request.material_type = first_product.type;
      service_call_succeeded = request_bin.call(find_bins);
      if(service_call_succeeded = 0){
        ROS_ERROR("product does not exist or no orders in play");
      }
      else{
        ROS_INFO("product type: %s is in  %s", first_product.type.c_str(), find_bins.response.storage_units.front().unit_id.c_str());
	if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin1")){
 	  first_image = logic_camera_bin_vector[0];
	  bin_frame = "logical_camera_bin1_frame";
          try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_bin1_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
        else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin2")){
	  first_image = logic_camera_bin_vector[1];
	  bin_frame = "logical_camera_bin2_frame";
	 
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_bin2_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
        else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin3")){
	  first_image = logic_camera_bin_vector[2];
	  bin_frame = "logical_camera_bin3_frame";
	  
          try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_bin3_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
        else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin4")){
	  first_image = logic_camera_bin_vector[3];
	  bin_frame = "logical_camera_bin4_frame";
	  
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_bin4_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin5")){
	  first_image = logic_camera_bin_vector[4];
	  bin_frame = "logical_camera_bin5_frame";
	  
          try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_bin5_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin6")){
	  first_image = logic_camera_bin_vector[5];
	  bin_frame = "logical_camera_bin6_frame";
	  
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_bin6_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "agv1")){
	  first_image = logic_agv_vector[0];
	  bin_frame = "logical_camera_agv1_frame";
	  
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_agv1_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "agv2")){
	  first_image = logic_agv_vector[1];
	  bin_frame = "logical_camera_agv2_frame";
	  
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_camera_agv2_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "quality_control_sensor_1")){
	  first_image = logic_agv_vector[0];
	  
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "quality_control_sensor_1_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "quality_control_sensor_2")){
	  first_image = logic_agv_vector[1];
	  
	  try{
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "quality_control_sensor_2_frame", ros::Time(0,0), ros::Duration(1.0));
	  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	  }
	  catch (tf2::TransformException &ex){
	    ROS_ERROR("%s", ex.what());
	  }
        }
        for(int i = 0; i < first_image.models.size(); i++){
	  if(!strcmp(first_image.models[i].type.c_str(), first_product.type.c_str())){
	    first_model.push_back(first_image.models[i]);
	    ROS_INFO_ONCE("The pose of the first product type %s in %s is:", first_image.models[i].type.c_str(), find_bins.response.storage_units.front().unit_id.c_str());
	    ROS_WARN_STREAM_ONCE(first_model.front().pose);
	  }
        }
      }
while(first_model.size() > 0){
      double T_pose[4][4];
      geometry_msgs::PoseStamped part_pose, image_pose, actuator_pose, waypoint, goal_pose;
      image_pose.pose = first_image.pose;
      try{
        tfWorld_Stamped = tfBuffer.lookupTransform("arm1_base_link", "world", ros::Time(0,0), ros::Duration(1.0));
	ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
      }
      catch (tf2::TransformException &ex){
	ROS_ERROR("%s", ex.what());
      }
      try{
	tfStamped = tfBuffer.lookupTransform("arm1_base_link", bin_frame, ros::Time(0,0), ros::Duration(1.0));
	ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
      }
      catch (tf2::TransformException &ex){
	ROS_ERROR("%s", ex.what());
      }
      tf2::doTransform(image_pose, actuator_pose, tfActuator_Stamped);
    part_pose.pose = first_model.back().pose;
      tf2::doTransform(part_pose, goal_pose, tfStamped);
      tf2::doTransform(image_pose, waypoint, tfWorld_Stamped);
      double T_way[4][4]= {{0.0, -1.0, 0.0, waypoint.pose.position.x + 0.3}, \
      {0.0, 0.0, 1.0, waypoint.pose.position.y - 0.2}, \
      {-1.0, 0.0, 0.0, waypoint.pose.position.z}, \
      {0.0, 0.0, 0.0, 1.0}};
      double T_des[4][4] = {{0.0, -1.0, 0.0, goal_pose.pose.position.x}, \
      {0.0, 0.0, 1.0, goal_pose.pose.position.y}, \
      {-1.0, 0.0, 0.0, goal_pose.pose.position.z + 0.2}, \
      {0.0, 0.0, 0.0, 1.0}};
      double q_pose[6], q_way[8][6], q_sols[8][6];
      int count = 0;
  //joint_states.position[1] is the linear_arm_actuator, not used in inverse calculation
      q_pose[0] = joint_states.position[2];
      q_pose[1] = joint_states.position[3];
      q_pose[2] = joint_states.position[0];
      q_pose[3] = joint_states.position[4];
      q_pose[4] = joint_states.position[5];
      q_pose[5] = joint_states.position[6];

      ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
      int num_way = ur_kinematics::inverse((double *)&T_way, (double *)&q_way, 0.0);

      int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_sols, 0.0);
      trajectory_msgs::JointTrajectory joint_trajectory;

      joint_trajectory.header.seq = count++;
      joint_trajectory.header.stamp = ros::Time::now();
      joint_trajectory.header.frame_id = "/base_link";

      joint_trajectory.joint_names.clear();
 joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
      joint_trajectory.joint_names.push_back("shoulder_pan_joint");
      joint_trajectory.joint_names.push_back("shoulder_lift_joint");
      joint_trajectory.joint_names.push_back("elbow_joint");
      joint_trajectory.joint_names.push_back("wrist_1_joint");
      joint_trajectory.joint_names.push_back("wrist_2_joint");
      joint_trajectory.joint_names.push_back("wrist_3_joint");

      joint_trajectory.points.resize(2);
      joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
      for(int indy = 0; indy < joint_trajectory.joint_names.size(); indy++){
        for(int indz = 0; indz < joint_states.name.size(); indz++) {
          if(joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
            joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
            break;
          }
        }
      }
      joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
      if(joint_states.position[1] > actuator_pose.pose.position.x + .1 || joint_states.position[1] < actuator_pose.pose.position.x - .1){
        joint_trajectory.points.resize(2);
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	joint_trajectory.points[1] = joint_trajectory.points[0];
	joint_trajectory.points[1].positions[0] = actuator_pose.pose.position.x;
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
        arm_command_pub.publish(joint_trajectory);
        ros::Duration(5.0).sleep();
	continue;
      } 
      if(points.size() > 0){
        joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
        joint_trajectory.points[1] = points[0];
        joint_trajectory.points[1].time_from_start = ros::Duration(2.0);
        /*joint_trajectory.points[2].positions.resize(joint_trajectory.joint_names.size());
        joint_trajectory.points[2] = points[0];
        joint_trajectory.points[2].time_from_start = ros::Duration(4.0);*/ 
        arm_command_pub.publish(joint_trajectory);
        ros::Duration(10.0).sleep();
        points.clear();
        first_model.pop_back();
        continue; 
      }
      points.push_back(joint_trajectory.points[0]);
     
	
  //will change to add limits to select the best solution
      int q_way_indx = 0;
      int q_sols_indx = 0;
      for(int i = 0; i < num_way; i++){
         if(q_way[i][0] > -1.57 && q_way[i][0] < 1.57){
	   if(q_way[i][1] > 0 && q_way[i][1] < 1.57){
              q_way_indx = i;
              break;
	   }
	 }
      }
      for(int i = 0; num_sols; i++){
         if(q_sols[i][0] > -1.57 && q_sols[i][0] < 1.57){
	   if(q_sols[i][1] > 0 && q_sols[i][1] < 1.57){
              q_sols_indx = i;
              break;
	   }
	 }
      }
      joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
      joint_trajectory.points[1].positions[0] = joint_states.position[1];
      for (int indy = 0; indy< 6; indy++){
        joint_trajectory.points[1].positions[indy + 1] = q_sols[q_way_indx][indy];
      }
      points.push_back(joint_trajectory.points[1]);
      joint_trajectory.points[1].time_from_start = ros::Duration(2.0);
      /*joint_trajectory.points[2].positions.resize(joint_trajectory.joint_names.size());
      joint_trajectory.points[2].positions[0] = joint_states.position[1];
      for (int indy = 0; indy< 6; indy++){
        joint_trajectory.points[2].positions[indy + 1] = q_sols[q_sols_indx][indy];
      }
      points.push_back(joint_trajectory.points[2]);
      joint_trajectory.points[2].time_from_start = ros::Duration(4.0);*/
      arm_command_pub.publish(joint_trajectory);
      ros::Duration(10.0).sleep();
     
    }
    }
    points.clear();
  }
  ros::waitForShutdown();
  return 0;
}
