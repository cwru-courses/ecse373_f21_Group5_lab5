#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "ros/ros.h"
std_srvs::Trigger begin_comp;
int service_call_succeeded;
osrf_gear::GetMaterialLocations find_bins;
osrf_gear::Order first_order;
osrf_gear::Shipment first_shipment;
osrf_gear::Product first_product;
std::vector<osrf_gear::Order> order_vector;
void chatterCallback(const osrf_gear::Order::ConstPtr& orders){
  order_vector.push_back(*orders);
}
int main(int argc, char **argv)
  {
  ros::init(argc, argv, "trigger_subscriber_node");
  
  ros::NodeHandle n;
  
  
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  order_vector.clear();
  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, chatterCallback);
  
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
        ROS_INFO("product type: %s is in first bin %s", first_product.type.c_str(), find_bins.response.storage_units.front().unit_id.c_str());
      }
    }
  }
  ros::waitForShutdown();
  return 0;
}
