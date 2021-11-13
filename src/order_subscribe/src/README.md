# Ariac overlook
> Updated with Lab 6 notes.
## ariac 2019
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

# Lab 5
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

### Retrieving the pose of the first product
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

# Lab 6

## Transforms

### Transform Listener

> The transform frames that exist in this environment can be tracked and found using a transform listener and buffer.
The listener tracks all tf and tf_static frames and inputs them to the buffer.
We initialize both of these objects and the listener must be initilaized after the node is created for it to work properly.

> To create a transform, we use the lookupTransform method of the buffer. 
`tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_bin_frame, ros::Time(0,0), ros::Duration(1.0));`
The inputs listed are the source frame, the target frame, the time the lookup starts, and how long it will take.

### Transform Frames 
> To move the robot arm, we must first ensure all poses given the arm are in the proper frame.
There are two target frames: The base_link and the linear arm actuator.

> The source frames depend on the storage unit being searched.
In this code, that is decided by the first element in the GetMaterialsLocation response.
For each possibility we provide values for strings `camera_bin_frame` and `bin_frame`.
The camera frame provides the frame of the logical camera for that specific location.
It is used to transform the location of the parts in a bin so the arm understands.
The bin frame is the frame of the storage location itself, which can be used to show the linear actuator where the arm should<br>
be located along the y axis (railing).

> A final source frame is `world`, which is the frame of the logical cameras themselves. This is used to tell the arm where the camera is and how to avoid it when reaching for parts.

## Joint Trajectory

### T matrix

> Joint trajectories define how the robot moves.
A trajetory conists of seven positions per point. 
Each point is a location the arm moves towards a destination, making them waypoints.
Each point beyond the initial point can be found using T matrixes.

> These matricies are 4 by 4. The left 3 by 3 provides the orientation of the desired position. 
The rightmost column provides the position of the desired location.
This location is determined by the transformed pose using the earlier transforms.

### UR10 kinematics

> This is a UR10 robot, and there are specific methods provided to tell ros how to move this robot. 
We specifically use `inverse(ur_kinematics::inverse((double *)&T, (double *)&q);`.
This method uses a provided T-matrix and finds how the six joints of the UR10 robot arm can angle themselves to have the final joint <br>
the "end effector" reach the desired location. 
Moving around a -2pi to 2pi range, there can be up to 8 solutions.
A solution is a list of angles that correspond to each joint. 
These angles are fed into the joint trajectory to move the robot.

> A quick note is this versin of the envirnoment always attempts to use "positive solutions" first.
This means the robot won't bend backwards through an awkward motion to reach the goal by default.

### Action Server

> The action server publishes the joint trajectories and provides feedback on the success or failure of the movement. 
Note: our implementation works, but states the actions are Aborted.
An aborted action occurs when there are collisions and the desired trajectory cannot be completed. 
However, our code does succeed without collisions.
![Working_Snapshot](project_ws/src/order_subscribe/Working_Snapshot.png);
This leads us to believe there may be an issue in the Action Server object header implementation. to be fixed.

### Returning to intitial position/ time issues

> Joint trajectories work by incrementing through space and time.
As a result, this makes it easy to backtrack through waypoints.
By saving the generated points in a trajectory, a second trajectory can be made with those points in the reverse order to <br>
return to the initial position. 
Our code moves to a part and then returns to initial position immediantly. 
This emulates picking up the part and then returning to a neatral state to move the part elsewhere.

> An issue with trajectories as implemented here is that the initial point is at time 0.0.
The JointTrajectory topics cannot initialize fast enough to run at time 0.0.
Therefore, as warning show, the first point is always lost.
AKA, the arm does not move to this point.
However, this is where the robot originates, or says it originates, so it should already be there.
Thus, there is no issue.
  














