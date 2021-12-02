#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "geometry_msgs/Pose.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "ur_kinematics/ur_kin.h"
#include "ros/ros.h"
#include "angles/angles.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
sensor_msgs::JointState joint_states;
std_srvs::Trigger begin_comp;
osrf_gear::VacuumGripperControl grip_control;
int service_call_succeeded, vac_grip_succeeded;
tf2_ros::Buffer tfBuffer;
osrf_gear::LogicalCameraImage first_image;
osrf_gear::GetMaterialLocations find_bins;
osrf_gear::Order current_order;
osrf_gear::Shipment current_shipment;
osrf_gear::Product current_product;
osrf_gear::VacuumGripperState vac_state;
std::vector<geometry_msgs::Pose> current_product_pose;
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
void vacStateCB(const osrf_gear::VacuumGripperState::ConstPtr& vac_msg){
  vac_state = *vac_msg;
}
int time_from_start (trajectory_msgs::JointTrajectory *trajectory){
  double max_position_dif = 0;
  int max_position_dif_num;
  float current_dif = 0;
  if(trajectory->points.size() < 1){
    return 0;
  }
  for(int j = 1; j < trajectory->points.size(); j++){
    double current_dif = 0;
    double max_position_dif = 0;
    for (int i = 0; i < trajectory->points[j].positions.size(); i++){
      current_dif = fabs(trajectory->points[j].positions[i] - trajectory->points[j-1].positions[i]);
      if(current_dif > max_position_dif){
        max_position_dif = double(current_dif);
      }
      //Normalizing values received from Professor Lee
      if (trajectory->joint_names[i] == "linear_arm_actuator_joint"){
        current_dif *= 1.7;
      }
      else{
        current_dif *= .6;
      }
      if(current_dif > max_position_dif){
        max_position_dif = double(current_dif);
      }
    }
    trajectory->points[j].time_from_start = trajectory->points[j-1].time_from_start + ros::Duration(max_position_dif) + ros::Duration(0.2);
  }
  return 0;
}
int set_empty(geometry_msgs::PoseStamped *set){
  set->pose.position.x = 0.0;
  set->pose.position.y = 0.0;
  set->pose.position.z = 0.0;
  set->pose.orientation.x = 0.0;
  set->pose.orientation.y = 0.0;
  set->pose.orientation.z = 0.0;
  set->pose.orientation.w = 0.0;
  return 0;
}
/*int select_sol(double *q_sols, int num_sols){
  int q_sols_indx = 0;
  for(int i = 0; i < num_sols; i++){
    ROS_INFO("response %d : %f", i, (*(q_sols + i) + 1));
    if(3*M_PI_2 < (*(q_sols + i)) && (*(q_sols + i)) < 2*M_PI){
      ROS_INFO("response %d : %f", i, (*(q_sols + i) + 4));
      if((*(q_sols + i) + 4) > 3* M_PI_2 - 0.01 && (*(q_sols + i) + 4) < 3 * M_PI_2 + 0.01){
	ROS_INFO("found");
        q_sols_indx = i;
        break;
      }   
    }
  }
  ROS_INFO("found %d", q_sols_indx);
  (*(q_sols + q_sols_indx + 1)) = angles::normalize_angle((*(q_sols + q_sols_indx + 1)));
  return q_sols_indx;
}*/
int engage_gripper(osrf_gear::VacuumGripperControl *grip_control, ros::ServiceClient *vacuum_client, int choice){
  bool holding = vac_state.attached;
  grip_control->request.enable = choice;
  service_call_succeeded = vacuum_client->call(*grip_control);
  if (service_call_succeeded == 0){
    ROS_ERROR("Gripper failed to engage!");
    ros::shutdown();
  }
  else{
    if(grip_control->response.success){
      ros::Time start_wait = ros::Time::now();
      //gross code, update later
      while(ros::Time::now().toSec() - start_wait.toSec() < 3){
	if(!(vac_state.attached == holding)){
	ROS_INFO("grip succeeded: %d %d", vac_state.attached, holding);
	return 1;
	}
      }
      ROS_INFO("grip failed");
      return 2;
    }
    else{
      return grip_control->response.success;
	ROS_INFO("gripper could not grip");
    }
  }
  return 0;
}
int main(int argc, char **argv)
  {
  ros::init(argc, argv, "trigger_subscriber_node");
  
  ros::NodeHandle n;
  order_vector.clear();
  current_product_pose.clear();
  logic_camera_bin_vector.clear();
  logic_camera_bin_vector.resize(6);
  logic_agv_vector.clear();
  logic_agv_vector.resize(2);
  logic_quality_vector.clear();
  logic_quality_vector.resize(2);
  points.clear();
  tf2_ros::TransformListener tfListener(tfBuffer);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  
  ros::ServiceClient vacuum_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
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

  ros::Subscriber vacuum_subscriber = n.subscribe("/ariac/arm1/gripper/state", 10, vacStateCB);

  while(ros::ok() && !tfBuffer.canTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0,0), ros::Duration(4.0))){
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
      std::string camera_bin_frame, bin_frame, tray_frame, agv_camera_frame;
      current_order = order_vector.back();
      geometry_msgs::PoseStamped last_desired;
      int agv_tray = 2;
      set_empty(&last_desired);
      ros::Duration(1.0).sleep();
      while(current_order.shipments.size() > 0){
        current_shipment = current_order.shipments.back();
        agv_camera_frame = "logical_camera_agv2_frame";
        tray_frame = "kit_tray_2";
        while(current_shipment.products.size() > 0){
          current_product = current_shipment.products.back();
          find_bins.request.material_type = current_product.type;
          service_call_succeeded = request_bin.call(find_bins);
          if(service_call_succeeded = 0){
            ROS_ERROR("product does not exist or no orders in play");
          }
          else{
            ROS_INFO("product type: %s is in  %s", current_product.type.c_str(), find_bins.response.storage_units.front().unit_id.c_str());
	    if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin1")){
 	      first_image = logic_camera_bin_vector[0];
	      camera_bin_frame = "logical_camera_bin1_frame";
	      bin_frame = "bin1_frame";
            }
            else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin2")){
	      first_image = logic_camera_bin_vector[1];
	      camera_bin_frame = "logical_camera_bin2_frame";
	      bin_frame = "bin2_frame";
            }
            else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin3")){
	      first_image = logic_camera_bin_vector[2];
	      camera_bin_frame = "logical_camera_bin3_frame";
	      bin_frame = "bin3_frame";
	    }  
            else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin4")){
	      first_image = logic_camera_bin_vector[3];
	      camera_bin_frame = "logical_camera_bin4_frame";
	      bin_frame = "bin4_frame";
            }
	    else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin5")){
	      first_image = logic_camera_bin_vector[4];
	      camera_bin_frame = "logical_camera_bin5_frame";
	      bin_frame = "bin5_frame";
            }
	    else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin6")){
	      first_image = logic_camera_bin_vector[5];
	      camera_bin_frame = "logical_camera_bin6_frame";
	      bin_frame = "bin6_frame";
            }
	    else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "agv1")){
	      first_image = logic_agv_vector[0];
	      camera_bin_frame = "logical_camera_agv1_frame";
	      bin_frame = "agv1_frame";
            }
	    else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "agv2")){
	      first_image = logic_agv_vector[1];
	      camera_bin_frame = "logical_camera_agv2_frame";
	      bin_frame = "agv2_frame";
            }
	    else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "quality_control_sensor_1")){
	      first_image = logic_agv_vector[0];
            }
	    else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "quality_control_sensor_2")){
	      first_image = logic_agv_vector[1];
	  
            }
            for(int i = 0; i < first_image.models.size(); i++){
	      if(!strcmp(first_image.models[i].type.c_str(), current_product.type.c_str())){
	        current_product_pose.push_back(first_image.models[i].pose);
	        ROS_INFO_ONCE("The pose of the first product type %s in %s is:", first_image.models[i].type.c_str(), find_bins.response.storage_units.front().unit_id.c_str());
	        ROS_WARN_STREAM_ONCE(current_product_pose.back());
		break;
              }
            }
          }
          int check_linear = 1;
          while(current_product_pose.size() > 0){ //only ever reaches one but allows for looping
	    ROS_INFO("here %d", current_product_pose.size());
            double T_pose[4][4];
	    int agv = 0;
            int num_way = 0;
            geometry_msgs::PoseStamped part_pose, bin_pose, actuator_pose, goal_pose;
	    ROS_INFO("%d , %d", vac_state.attached, points.size());
	    if(vac_state.attached == 1 && points.size() == 0){
                camera_bin_frame = tray_frame;
	        bin_frame = agv_camera_frame;
		part_pose.pose = current_product.pose;
	        agv = 1;
            }
            else{
              part_pose.pose = current_product_pose.back();
            }
            ROS_INFO("%d", agv);
	    tfActuator_Stamped = tfBuffer.lookupTransform("arm1_linear_arm_actuator", bin_frame, ros::Time(0,0), ros::Duration(1.0));
	      ROS_DEBUG("Transform to [%s] from [%s]", tfActuator_Stamped.header.frame_id.c_str(), tfActuator_Stamped.child_frame_id.c_str());
	    
	    tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_bin_frame, ros::Time(0,0), ros::Duration(1.0));
	      ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
            
      
            set_empty(&bin_pose);
            tf2::doTransform(bin_pose, actuator_pose, tfActuator_Stamped);
            tf2::doTransform(part_pose, goal_pose, tfStamped);
            double side = 0;
            if(part_pose.pose.position.y > bin_pose.pose.position.y){
              side = .2;
              if(agv == 1 && agv_tray == 2){
                side *= -1;
              }
            }
            else{
              side = -.2;
              if(agv == 1 && agv_tray == 2){
                side *= -1;
              }
            }
	    double T_way[4][4] = {{0.0, -1.0, 0.0, goal_pose.pose.position.x}, \
            {0.0, 0.0, 1.0, goal_pose.pose.position.y + side}, \
            {-1.0, 0.0, 0.0, goal_pose.pose.position.z + 0.5}, \
            {0.0, 0.0, 0.0, 1.0}};
            double T_agv1[4][4] = {{0.0, -1.0, 0.0, goal_pose.pose.position.x + side}, \
            {0.0, 0.0, 1.0, goal_pose.pose.position.y - 0.3}, \
            {-1.0, 0.0, 0.0, goal_pose.pose.position.z + 0.3}, \
            {0.0, 0.0, 0.0, 1.0}};
            double T_agv2[4][4] = {{0.0, -1.0, 0.0, goal_pose.pose.position.x + side}, \
            {0.0, 0.0, 1.0, goal_pose.pose.position.y + 0.3}, \
            {-1.0, 0.0, 0.0, goal_pose.pose.position.z + 0.3}, \
            {0.0, 0.0, 0.0, 1.0}};
            double T_des[4][4] = {{0.0, -1.0, 0.0, goal_pose.pose.position.x}, \
            {0.0, 0.0, 1.0, goal_pose.pose.position.y}, \
            {-1.0, 0.0, 0.0, goal_pose.pose.position.z + 0.019}, \
            {0.0, 0.0, 0.0, 1.0}};
            double q_pose[6], q_way[8][6], q_sols[8][6];
            int count = 0;
            int action_count = 0;
            if (agv == 1 && agv_tray == 1){
              num_way = ur_kinematics::inverse((double *)&T_agv1, (double *)&q_way, 0.0);
	    }
            else if (agv == 1 && agv_tray == 2){
              num_way = ur_kinematics::inverse((double *)&T_agv2, (double *)&q_way, 0.0);
	    }
	    else{
              num_way = ur_kinematics::inverse((double *)&T_way, (double *)&q_way, 0.0);
            }
            int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_sols, 0.0);
            trajectory_msgs::JointTrajectory joint_trajectory;
            control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
            
            joint_trajectory.header.seq = count++;
            joint_trajectory.header.stamp = ros::Time::now();
            joint_trajectory.header.frame_id = "/base_link";

            joint_trajectory_as.action_goal.header.seq = action_count++;
            joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
            joint_trajectory_as.action_goal.header.frame_id = "/base_link";
 
            joint_trajectory.joint_names.clear();
            joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
            joint_trajectory.joint_names.push_back("shoulder_pan_joint");
            joint_trajectory.joint_names.push_back("shoulder_lift_joint");
            joint_trajectory.joint_names.push_back("elbow_joint");
            joint_trajectory.joint_names.push_back("wrist_1_joint");
            joint_trajectory.joint_names.push_back("wrist_2_joint");
            joint_trajectory.joint_names.push_back("wrist_3_joint");
            joint_trajectory.points.resize(3);
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
            if(check_linear == 1){
              bool side;
              joint_trajectory.points.resize(2);
	      joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
     	      joint_trajectory.points[1] = joint_trajectory.points[0];
	      if(part_pose.pose.position.y > 0.0){
		ROS_INFO("%f", actuator_pose.pose.position.y);
                joint_trajectory.points[1].positions[0] = actuator_pose.pose.position.y + .5;
                side = 0;
              }
	      else{
                joint_trajectory.points[1].positions[0] = actuator_pose.pose.position.y - .5;
                side = 1;
	      }
	      if(side = 0 &&(actuator_pose.pose.position.y +.5) > 2 && agv == 0){
                 joint_trajectory.points[1].positions[0] = 2.56;
	      }
              else if ((actuator_pose.pose.position.y + .5) > 2 && agv == 1){
		joint_trajectory.points[1].positions[0] = 2.3;
              }
              if(side = 1 &&(actuator_pose.pose.position.y - .5) < -2 && agv == 0){
                ROS_INFO("%f", actuator_pose.pose.position.y);
                joint_trajectory.points[1].positions[0] = -2.5;
	      }
              else if ((actuator_pose.pose.position.y - .5) < -2 && agv == 1){
		joint_trajectory.points[1].positions[0] = -2.3;
              }
              
	      time_from_start(&joint_trajectory);
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
	      actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
	      ROS_INFO("action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
              check_linear = 0;
              ros::Duration(5.0).sleep();
	      continue;
            } 
            if(points.size() > 0){
	      points[0].positions[0] = points[2].positions[0];
              joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
              joint_trajectory.points[1] = points[1];
              joint_trajectory.points[2].positions.resize(joint_trajectory.joint_names.size());
              joint_trajectory.points[2] = points[0];
              time_from_start(&joint_trajectory);
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
	      actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
	      ROS_INFO("action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
	      ros::Duration(10.0).sleep();
	      if (vac_state.attached == 0){
                current_product_pose.pop_back();
                current_shipment.products.pop_back();
	      }
              check_linear = 1;
              points.clear();
              continue; 
            }
            points.push_back(joint_trajectory.points[0]);
     
	
      /*int q_way_indx = select_sol((double *)&q_way, num_way);*/
      /*int q_sols_indx = select_sol((double *)&q_sols, num_sols);*/
  //not using function because pointer has issues
            int q_way_indx = 0;
            int q_sols_indx = 0;
            for(int i = 0; i < num_way; i++){
              if(3*M_PI_2 < q_way[i][1] && q_way[i][1] < 2*M_PI){
                if(q_way[i][4] > M_PI_2 - 0.01 && q_way[i][4] < M_PI_2 + 0.01){
                  q_way_indx = i;
                break;
                }   
              }
            }
            q_way[q_way_indx][1] = angles::normalize_angle(q_way[q_way_indx][1]); 
            for(int i = 0; i < num_sols; i++){
              if(3*M_PI_2 < q_sols[i][1] && q_sols[i][1] < 2*M_PI){
                if(q_sols[i][4] > M_PI_2 - 0.01 && q_sols[i][4] < M_PI_2 + 0.01){
                  q_sols_indx = i;
                  break;
                }   
              }
            }
            q_sols[q_sols_indx][1] = angles::normalize_angle(q_sols[q_sols_indx][1]); 
            
            joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
            joint_trajectory.points[1].positions[0] = joint_states.position[1];
            for (int indy = 0; indy< 6; indy++){
              joint_trajectory.points[1].positions[indy + 1] = q_way[q_way_indx][indy];
            }
            points.push_back(joint_trajectory.points[1]);
            joint_trajectory.points[2].positions.resize(joint_trajectory.joint_names.size());
            joint_trajectory.points[2].positions[0] = joint_states.position[1];
            for (int indy = 0; indy< 6; indy++){
              joint_trajectory.points[2].positions[indy + 1] = q_sols[q_sols_indx][indy];
            }
            points.push_back(joint_trajectory.points[2]);
            time_from_start(&joint_trajectory);
            joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
            actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
   	    ROS_INFO("action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
	    ros::Duration(5.0).sleep();
	    if(vac_state.attached == 0){
	      bool holding = vac_state.attached;
              grip_control.request.enable = 1;
              service_call_succeeded = vacuum_client.call(grip_control);
              if (service_call_succeeded == 0){
                ROS_ERROR("Gripper failed to engage!");
                ros::shutdown();
              }
              else{
                if(grip_control.response.success){
                  ros::Time start_wait = ros::Time::now();
              //gross code, update later
                  while(ros::Time::now().toSec() - start_wait.toSec() < 5){
		    if(vac_state.attached == 1){
	              break;
                    }
                  }
                  if(vac_state.attached == 0){
                    grip_control.request.enable = 0;
                    service_call_succeeded = vacuum_client.call(grip_control);
                  }
                } 
                else{
	          ROS_INFO("gripper could not grip");
                }
              }
	    }
            else{
              grip_control.request.enable = 0;
              service_call_succeeded = vacuum_client.call(grip_control);
              if (service_call_succeeded == 0){
                ROS_ERROR("Gripper failed to engage!");
                ros::shutdown();
              }
              else{
                if(grip_control.response.success){
                  ros::Time start_wait = ros::Time::now();
              //gross code, update later
                  while(ros::Time::now().toSec() - start_wait.toSec() < 3){
		    if(vac_state.attached == 0){
	              break;
                    }
                  }  
                }
                else{
	          ROS_INFO("gripper could not grip");
                }
	      }
            }
          }
        }
        current_order.shipments.pop_back();
      }
      order_vector.pop_back();
    }
  }
      
    
  ros::waitForShutdown();
  return 0;
}
