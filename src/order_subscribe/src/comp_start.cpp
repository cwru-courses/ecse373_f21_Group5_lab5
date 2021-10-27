#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "geometry_msgs/Pose.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "ros/ros.h"
std_srvs::Trigger begin_comp;
int service_call_succeeded;
osrf_gear::LogicalCameraImage first_image;
osrf_gear::GetMaterialLocations find_bins;
osrf_gear::Order first_order;
osrf_gear::Shipment first_shipment;
osrf_gear::Product first_product;
std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_camera_bin_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_agv_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_quality_vector;

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
int main(int argc, char **argv)
  {
  ros::init(argc, argv, "trigger_subscriber_node");
  
  ros::NodeHandle n;
  
  order_vector.clear();
  logic_camera_bin_vector.clear();
  logic_camera_bin_vector.resize(6);
  logic_agv_vector.clear();
  logic_agv_vector.resize(2);
  logic_quality_vector.clear();
  logic_quality_vector.resize(2);

  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  
  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, chatterCallback);

  ros::Subscriber logical_camera_subscriber_agv1 = n.subscribe("/ariac/logical_camera_agv1", 1000, logicagv1CameraCallback);
  ros::Subscriber logical_camera_subscriber_agv2 = n.subscribe("/ariac/logical_camera_agv2", 1000, logicagv2CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin1 = n.subscribe("/ariac/logical_camera_bin1", 1000, logicbin1CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin2 = n.subscribe("/ariac/logical_camera_bin2", 1000, logicbin2CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin3 = n.subscribe("/ariac/logical_camera_bin3", 1000, logicbin3CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin4 = n.subscribe("/ariac/logical_camera_bin4", 1000, logicbin4CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin5 = n.subscribe("/ariac/logical_camera_bin5", 1000, logicbin5CameraCallback);
  ros::Subscriber logical_camera_subscriber_bin6 = n.subscribe("/ariac/logical_camera_bin6", 1000, logicbin6CameraCallback);
  ros::Subscriber logical_camera_subscriber_quality_control_sensor1 = n.subscribe("/ariac/quality_control_sensor_1", 1000, logicQuality1CameraCallback);
  ros::Subscriber logical_camera_subscriber_quality_control_sensor2 = n.subscribe("/ariac/quality_control_sensor_2", 1000, logicQuality2CameraCallback);
  
  ros::ServiceClient request_bin = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  service_call_succeeded = begin_client.call(begin_comp);
	if (service_call_succeeded == 0){
	  ROS_ERROR("Competition service call failed! Shit!");
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
        }
        else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin2")){
	  first_image = logic_camera_bin_vector[1];
        }
        else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin3")){
	  first_image = logic_camera_bin_vector[2];
        }
        else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin4")){
	  first_image = logic_camera_bin_vector[3];
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin5")){
	  first_image = logic_camera_bin_vector[4];
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "bin6")){
	  first_image = logic_camera_bin_vector[5];
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "agv1")){
	  first_image = logic_agv_vector[0];
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "agv2")){
	  first_image = logic_agv_vector[1];
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "quality_control_sensor_1")){
	  first_image = logic_agv_vector[0];
        }
	else if(!strcmp(find_bins.response.storage_units.front().unit_id.c_str(), "quality_control_sensor_2")){
	  first_image = logic_agv_vector[1];
        }
        for(int i = 0; first_image.models.size(); i++){
	  if(!strcmp(first_image.models[i].type.c_str(), first_product.type.c_str())){
	    ROS_INFO("The pose of the first product type %s in %s is:", first_image.models[i].type.c_str(), find_bins.response.storage_units.front().unit_id.c_str());
	    ROS_WARN_STREAM(first_image.models[i].pose);
	    break;
	  }
        }
      }
    }
    ros::Duration(2).sleep();
  }
  ros::waitForShutdown();
  return 0;
}
