# Order Subscriber + First Product Found

##ariac 2019
> The code and environments specified in this lab draw heavily from structures provided by the ariac 2019 competition.
Visit [Ariac 2019 Wiki](https://bitbucket.org/osrf/ariac/wiki/2019/documentation) to read these specs.
## launch files

### environment.launch
> The environment launch file chooses whether to load the sample environment or the simulation environment.
The arg `use_sample_environment` defaults to false. 
To run the sample environment, run the command<br>
`roslaunch order_subscriber environment.launch use_sample_environment:= true`.

### order_subscribe.launch
> This launch file runs the subscriber node that also starts the competition.

## subscriber node

### starting the competition
 > The competition starts using the Service std_srvs/Trigger.srv.
We first create a Trigger object, then call it using a ServiceClient.
`ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition")`
The topic /ariac/start_competition is used to see if starting is possible.
the integer, service_call_succeeded is used to check whether this call is successful.
When the trigger calls the ServiceClient `service_call_succeeded = begin_client.call(begin_comp)`<br>
if service_call_succeeded=0, the call itself failed.
If service_call_succeeded=1, the call succeeded.
However, within the Trigger is a boolean success.
If success is true, the trigger succeeded in starting the competition.
If success is false, the trigger failed. 
Trigger also contains a response string.
If success fails and the response string = *cannot start if not in 'init' state* the competition has already been started.
If service_call_succeeded=0 or the response message is not equal to the message above, we shut down ros as there is some error.
### reading the first order
> now that the competition has started, the first order is published.
We subscribe to the *ariac/order* topic to receive this message.
We may publish several orders at a time, therefore we need a vector<br>
to store the list of orders.
This vector must be cleared using `<vector_name>.clear()` in the main loop to initialize it.
Within the callback method, we add orders to the end of the vector using `<vector_name>.pushback(<order_msg_name>)` 

> The order vector can only be searched if an order exists. 
Otherwise, we will incur a segmentaion fault.
This is avoided by running `if(order_vector.size() > 0`

> The order message contains a vector of shipments, and each shipment contains a vector of products.
Once we have an order, we use `order_vector.front()` to grab the first element in the vector,<br>
which for now is the first order.
Following a similar logic, we grab the first shipment and the first product.

> The product will contain a string called type, which says what kind of product it is.
The Service GetMaterialLocations is run to find the locations where a product type is. 
It works similarly to Trigger, expect the topic we service is /ariac/material_locations.
Also, the service requires request info, the type of the product it is searching for. 
This can be accomplished with `<service_name>.request.material_type = <product_name>.type`

###Retrieving the pose of the first product
 > The locations of the products are found using the 10 logical cameras.
One for each of the six bins, two for the agvs, and two for quality.
We subscribe to all ten individually using the rostopics listed when running the simulation environment.
This is actually done using nested callbacks. 
For example, there is one callback for each bin. 
Each pushes the message it receives to a particular index of another vector that holds the messages of all bin logical cameras.
The first bin pushes to the zeroeth index.
`logicalCameraBinVectorCallback(bin_msg, 0)`
The outer vector receives it
`void logicalCameraBinVectorCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int cam_num){logic_camera_bin_vector[cam_num] = *msg;}`.
> In our code, once the GetMaterialsLocation service returns a location, we choose the index of the vector that matches that location.
We have three vectors, one for bins, one for agvs, and one for quality.
In this simulation, it retruns bin4.
We then search the logical camera image for bin4, in `logic_camera_bin_vector[3]`
>Each logicalCameraImage holds a vector of Models.
Each Model has a type, product types to be specific, and a pose.
We search the vector for the first model that has the same product type as the first product and return its pose.
We only do this once ever, using `ROS_WARN_STREAM_ONCE`







