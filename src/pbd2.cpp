/*!
    \file pbd2.cpp
    \brief Software Arc. Assignment Baxter PbD
    \author Eloise, Zsolt, Natalia
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pitt_msgs/TrackedShapes.h" // for out message (an array of TrackedShape)
#include "pitt_msgs/TrackedShape.h" // for out message
#include "baxter_core_msgs/EndpointState.h"
#include "geometry_msgs/Pose.h"
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/Range.h>

/*! \def PITT_MEASURMENT_TO_RECEIVE
    \brief We receive PITT_MEASURMENT_TO_RECEIVE messages from PITT before unsubscribing to the topic
*/
#define PITT_MEASURMENT_TO_RECEIVE 2
/*! \def DISTANCE_MEASURMENT_MODE
    \brief Choose 1 to use PITT and 2 to use iR sensors
*/
#define DISTANCE_MEASURMENT_MODE 2
/*! \def THRESHOLD_OPEN_PITT
    \brief Threshold in meters, if a gripper-object distance goes under this
     threshold, the gripper will open (only if the gripper was previously closed
     and not holding an object). The distance is computed using PITT.
*/
#define THRESHOLD_OPEN_PITT 0.13
/*! \def THRESHOLD_CLOSE_PITT
    \brief Threshold in meters, if a gripper-object distance goes under this
     threshold, the gripper will close (only if the gripper was previously open
     ). The distance is computed using PITT.
*/
#define THRESHOLD_CLOSE_PITT 0.10
#define THRESHOLD_OPEN_iR 0.25 //Same but with the distance from the iR sensor
#define THRESHOLD_CLOSE_iR 0.20


/*! \var tracked_shapes
    \brief Global variable used to recover pitt msgs in the tracked_shape_callback
*/
pitt_msgs::TrackedShapes  tracked_shapes, empty;
/*! \var pose_left
    \brief Global variable used to recover left end effector pose in the  endpoint_state_left callback
*/
geometry_msgs::Pose pose_right, pose_left;
/*! \var pitt_measurement_done
    \brief pitt_measurement_done will be set to true when PITT_MEASURMENT_TO_RECEIVE have been received
*/
int number_of_detected_objects=0;
int pitt_msgs_counter = 0;
bool pitt_measurement_done = false;
/*! \var left_range
    \brief Global variable used to recover left iRsensor range in the irSensor_left callback
*/
float left_range = 10000;
float right_range = 10000;
/*! \var left_hand_open
    \brief Global variable used to recover left gripper open or close state in the left_gripper_open callback
*/
bool left_hand_open = false;
/*! \var left_hand_gripping
    \brief Global variable used to recover if the left gripper holds someting in the left_gripper_open callback
*/
bool left_hand_gripping = false;
bool right_hand_open = false;
bool right_hand_gripping = false;
/*! \var sub_pitt
    \brief Global subscriber, will subscribe to /ransac_segmentation/trackedShapes and unsubscribe in the tracked_shape_callback
*/
ros::Subscriber sub_pitt;
ros::Publisher  pub_left_gripper;
ros::Publisher  pub_right_gripper;


/*! \fn float compute_distance_to_an_object(pitt_msgs::TrackedShape tracked_shape,geometry_msgs::Pose gripper_pose)
    \brief Compute the distance between the gripper and one object
    \param tracked_shape : an object coming from pitt
    \param gripper_pose : the position of a gripper
    \return distance
*/
float compute_distance_to_an_object(pitt_msgs::TrackedShape tracked_shape,geometry_msgs::Pose gripper_pose){
    float distance = sqrt( pow((float)(tracked_shape.z_pc_centroid-gripper_pose.position.z),2)
                    +pow((float)(tracked_shape.y_pc_centroid-gripper_pose.position.y),2)
                    +pow((float)(tracked_shape.x_pc_centroid-gripper_pose.position.x),2));
    return distance;
}

/*! \fn void compute_min_distance_to_objects(int* objects, float* distances, pitt_msgs::TrackedShapes tracked_shapes, geometry_msgs::Pose left_gripper_pose, geometry_msgs::Pose right_gripper_pose)
    \brief Find the closest object to the gripper and compute the gripper-nearest object distance
    \param objects : objects[0] store the index of the left gripper's nearest object. objects[1] for right gripper nearest object.
    \param distances : distances[0] store the distance of the left gripper with its nearest object. distances[1] for the right gripper
    \param tracked_shapes : recovered from pitt, contains all the detected objects
    \param left_gripper_pose
    \param right_gripper_pose
*/
void compute_min_distance_to_objects(int* objects, float* distances,
                                    pitt_msgs::TrackedShapes tracked_shapes,
                                    geometry_msgs::Pose left_gripper_pose,
                                    geometry_msgs::Pose right_gripper_pose){
    float left_distance = 100000;
    float right_distance = 100000;
    float distance = 0;

    for(int i= 0; i< number_of_detected_objects; i++){
        distance = compute_distance_to_an_object(tracked_shapes.tracked_shapes[i],left_gripper_pose);
        if(left_distance > distance ){
            left_distance = distance;
            distances[0]=distance;
            objects[0]=i;
        }

        distance = compute_distance_to_an_object(tracked_shapes.tracked_shapes[i],right_gripper_pose);
        if(right_distance > distance ){
            right_distance = distance;
            distances[1]=distance;
            objects[1]=i;
        }
    }
}

/*! \fn void vector_difference(float a[3], float b[3],float * difference)
    \brief Compute the difference of two vectors
    \param a[3] : first vector input
    \param b[3] : second vector input
    \param difference : the output difference
*/
void vector_difference(float a[3], float b[3],float * difference){
  difference[0] = a[0] -b[0];
  difference[1] = a[1] -b[1];
  difference[2] = a[2] -b[2];
}

/*! \fn float vector_square(float a[3])
    \brief Compute the square norm 2 of a vector
    \param a[3] : vector input
    \return square : the square norm 2
*/
float vector_square(float a[3]){
  float square = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
  return square;
}

/*! \fn void endpoint_state_left(const baxter_core_msgs::EndpointState::ConstPtr& msg)
    \brief Callback attached to the /robot/limb/left/endpoint_state topic, recover the left end effector pose.
    \param msg :  baxter_core_msgs::EndpointState containing the pose of the end effector
*/
void endpoint_state_left(const baxter_core_msgs::EndpointState::ConstPtr& msg)
{
    pose_left=msg->pose;
}

/*! \fn void endpoint_state_right(const baxter_core_msgs::EndpointState::ConstPtr& msg)
    \brief Callback attached to the /robot/limb/right/endpoint_state topic, recover the right end effector pose.
    \param msg : baxter_core_msgs::EndpointState containing the pose of the end effector
*/
void endpoint_state_right(const baxter_core_msgs::EndpointState::ConstPtr& msg)
{
    pose_right=msg->pose;
}

/*! \fn void irSensor_left(const sensor_msgs::Range::ConstPtr& msg)
    \brief Callback attached to the /robot/range/left_hand_range/state topic, recover the  ir sensor range measurment from the left end effector.
    \param msg :  sensor_msgs::Range::ConstPtr&  containing the ir sensor range measurment
*/
void irSensor_left(const sensor_msgs::Range::ConstPtr& msg)
{
    left_range= msg->range;
}

/*! \fn void irSensor_right(const sensor_msgs::Range::ConstPtr& msg)
    \brief Callback attached to the /robot/range/right_hand_range/state topic, recover the  ir sensor range measurment from the right end effector.
    \param msg :  sensor_msgs::Range::ConstPtr&  containing the ir sensor range measurment
*/
void irSensor_right(const sensor_msgs::Range::ConstPtr& msg)
{
    right_range= msg->range;
}

/*! \fn void tracked_shape_callback(const pitt_msgs::TrackedShapes::ConstPtr& msg)
    \brief Callback attached to the /ransac_segmentation/trackedShapes topic, recover shape and position of the objects detected by PITT.
    The associated subscriber will unsubscribe after PITT_MEASURMENT_TO_RECEIVE received measurment.
    \param msg :  pitt_msgs::TrackedShapes::ConstPtr&  recovered PITT message
*/
void tracked_shape_callback(const pitt_msgs::TrackedShapes::ConstPtr& msg)
{
    number_of_detected_objects = 0;
    number_of_detected_objects = msg->tracked_shapes.size();
    tracked_shapes = empty;
    tracked_shapes= *msg;
    pitt_msgs_counter ++;
    ROS_INFO("INSIDE THE LOOP\n");
    if(pitt_msgs_counter==PITT_MEASURMENT_TO_RECEIVE){
       pitt_measurement_done = true;
       ROS_INFO("SHUTDOWN THE LOOP\nn");
       sub_pitt.shutdown();
    }
}

/*! \fn void left_gripper_open(const baxter_core_msgs::EndEffectorState::ConstPtr& msg)
    \brief Callback attached to the /robot/end_effector/left_gripper/state topic,
     recover the open or closed state of the left gripper
    \param msg :  baxter_core_msgs::EndEffectorState::ConstPtr&  contains the position of the gripper (100 for opened and 0 for closed)
*/
void left_gripper_open(const baxter_core_msgs::EndEffectorState::ConstPtr& msg){
  ROS_INFO("Left gripper open/close position %f\n",msg->position);
  //ROS_INFO("FORCE %f\n",msg->force);
  if(msg->position>=90){
    left_hand_open = true;
  }
  else{
    left_hand_open = false;
  }
  if(!left_hand_open && msg->force >2){
    left_hand_gripping = true;
  }
  else{
    left_hand_gripping = false;
  }
}

/*! \fn void right_gripper_open(const baxter_core_msgs::EndEffectorState::ConstPtr& msg)
    \brief Callback attached to the /robot/end_effector/right_gripper/state topic,
     recover the open or closed state of the right gripper
    \param msg :  baxter_core_msgs::EndEffectorState::ConstPtr&  contains the position of the gripper (100 for opened and 0 for closed)
*/
void right_gripper_open(const baxter_core_msgs::EndEffectorState::ConstPtr& msg){
  //ROS_INFO("RIght gripper open/close position %f\n",msg->position);
  if(msg->position>=90){
    right_hand_open = true;
  }
  else{
    right_hand_open = false;
  }
  if(!right_hand_open && msg->force >2){
    right_hand_gripping = true;
  }
  else{
    right_hand_gripping = false;
  }
}

/*! \fn void open_left_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg)
    \brief Open left gripper when we push the circle button.
    \param msg baxter_core_msgs::DigitalIOState::ConstPtr& msg
*/
void open_left_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg){
	int state = msg->state;
	//ROS_INFO("Message state%i",state);
	if(msg->state == 1){
		baxter_core_msgs::EndEffectorCommand msg1;
		msg1.id = 65538;
		msg1.command = "release";
		msg1.args = "";
		msg1.sender = "pbd2";
		ROS_INFO("LEFT OPEN PUBLISHING");
		pub_left_gripper.publish(msg1);
	}
}

/*! \fn void close_left_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg)
    \brief Close left gripper when we push the circle button.
    \param msg baxter_core_msgs::DigitalIOState::ConstPtr& msg
*/
void close_left_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg){
	if(msg->state == 1){
		baxter_core_msgs::EndEffectorCommand msg1;
		msg1.id = 65538;
		msg1.command = "grip";
		msg1.args = "";
		msg1.sender = "pbd2";
		ROS_INFO("LEFT CLOSE PUBLISHING");
		pub_left_gripper.publish(msg1);
	}
}

/*! \fn void open_right_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg)
    \brief Open right gripper when we push the circle button.
    \param msg baxter_core_msgs::DigitalIOState::ConstPtr& msg
*/
void open_right_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg){
	int state = msg->state;
	//ROS_INFO("Message state%i",state);
	if(msg->state == 1){
		baxter_core_msgs::EndEffectorCommand msg1;
		msg1.id = 65538;
		msg1.command = "release";
		msg1.args = "";
		msg1.sender = "pbd2";
		ROS_INFO("RIGHT OPEN PUBLISHING");
		pub_right_gripper.publish(msg1);
	}
}

/*! \fn void close_right_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg)
    \brief Open right gripper when we push the circle button.
    \param msg baxter_core_msgs::DigitalIOState::ConstPtr& msg
*/
void close_right_call(const baxter_core_msgs::DigitalIOState::ConstPtr& msg){
	if(msg->state == 1){
		baxter_core_msgs::EndEffectorCommand msg1;
		msg1.id = 65538;
		msg1.command = "grip";
		msg1.args = "";
		msg1.sender = "pbd2";
		ROS_INFO("RIGHT CLOSE PUBLISHING");
		pub_right_gripper.publish(msg1);
	}
}

/*! \fn int main(int argc, char **argv)
    \brief main, publish to  robot/end_effector/left_gripper/command and
        robot/end_effector/right_gripper/command to open or close the grippers.
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pbd2");
    ros::NodeHandle n;
    sub_pitt = n.subscribe("/ransac_segmentation/trackedShapes", 1, tracked_shape_callback);
    ros::Subscriber sub_left_endpoint_state = n.subscribe("/robot/limb/left/endpoint_state", 1, endpoint_state_left);
    ros::Subscriber sub_right_endpoint_state = n.subscribe("/robot/limb/right/endpoint_state", 1, endpoint_state_right);
    ros::Subscriber sub_left_gripper_open = n.subscribe("/robot/end_effector/left_gripper/state", 1, left_gripper_open);
    ros::Subscriber sub_right_gripper_open = n.subscribe("/robot/end_effector/right_gripper/state", 1, right_gripper_open);
    ros::Subscriber sub_irSensor_left = n.subscribe("/robot/range/left_hand_range/state", 1, irSensor_left);
    ros::Subscriber sub_irSensor_right = n.subscribe("/robot/range/right_hand_range/state", 1, irSensor_right);
    pub_left_gripper = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",10);
    pub_right_gripper = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10);
    ros::Subscriber open_left = n.subscribe("/robot/digital_io/left_lower_button/state", 1, open_left_call);
    ros::Subscriber close_left = n.subscribe("/robot/digital_io/left_upper_button/state", 1, close_left_call);
    ros::Subscriber open_right = n.subscribe("/robot/digital_io/right_lower_button/state", 1, open_right_call);
    ros::Subscriber close_right = n.subscribe("/robot/digital_io/right_upper_button/state", 1, close_right_call);

    ros::Rate loop_rate(1);

    int* objects;
    float* distances;
    objects=(int*)malloc(sizeof(int)*2);
    distances=(float*)malloc(sizeof(float)*2);

    float left_distance = 10000;
    float right_distance = 10000;
    float threshold_open = 0;
    float threshold_close = 0;


    while(ros::ok()){

      compute_min_distance_to_objects(objects,distances,tracked_shapes,pose_left,pose_right);

      if(DISTANCE_MEASURMENT_MODE == 1){
          left_distance = distances[0];
          right_distance = distances[1];
          threshold_open = THRESHOLD_OPEN_PITT;
          threshold_close = THRESHOLD_CLOSE_PITT;
      }

      if(DISTANCE_MEASURMENT_MODE == 2){
          left_distance = left_range;
          right_distance = right_range;
          threshold_open = THRESHOLD_OPEN_iR;
          threshold_close = THRESHOLD_CLOSE_iR;
          pitt_measurement_done = true;
      }
      // ROS_INFO("LEFT ARM DISTANCE TO THE NEAREST OBJECT: [%f]   OBJECT NUMBER: [%d]\n",distances[0],objects[0]);
      //ROS_INFO("RIGHT ARM DISTANCE TO THE NEAREST OBJECT: [%f]   OBJECT NUMBER: [%d]\n",distances[1],objects[1]);

        ROS_INFO("Left distance %f\n",left_distance);

      if(left_distance < threshold_open && !left_hand_open && !left_hand_gripping && pitt_measurement_done){
        baxter_core_msgs::EndEffectorCommand msg1;
        msg1.id = 65538;
        msg1.command = "release";
        msg1.args = "";
        msg1.sender = "pbd2";
        msg1.sequence = 3;
        ROS_INFO("LEFT OPEN PUBLISHING");
        pub_left_gripper.publish(msg1);
      }
      if(right_distance < threshold_open && !right_hand_open && !right_hand_gripping && pitt_measurement_done){
        baxter_core_msgs::EndEffectorCommand msg1;
        msg1.id = 65538;
        msg1.command = "release";
        msg1.args = "";
        msg1.sender = "pbd2";
        msg1.sequence = 3;
        ROS_INFO("RIGHT OPEN PUBLISHING");
        pub_right_gripper.publish(msg1);
      }

      if(left_distance < threshold_close && left_hand_open && pitt_measurement_done){
        baxter_core_msgs::EndEffectorCommand msg1;
        msg1.id = 65538;
        msg1.command = "grip";
        msg1.args = "";
        msg1.sender = "pbd2";
        msg1.sequence = 3;
        ROS_INFO("LEFT CLOSE PUBLISHING");
        pub_left_gripper.publish(msg1);
        if(DISTANCE_MEASURMENT_MODE == 1){
            ros::Duration(4).sleep();
        }
      }

      if(right_distance < threshold_close  && right_hand_open && pitt_measurement_done){
        baxter_core_msgs::EndEffectorCommand msg1;
        msg1.id = 65538;
        msg1.command = "grip";
        msg1.args = "";
        msg1.sender = "pbd2";
        msg1.sequence = 3;
        ROS_INFO("RIGHT CLOSE PUBLISHING");
        pub_right_gripper.publish(msg1);
      }

      //Release the object at the same position in PITT mode
       if(DISTANCE_MEASURMENT_MODE == 1 && !left_hand_open && left_distance < threshold_close && pitt_measurement_done){
           ros::Duration(2).sleep();
           baxter_core_msgs::EndEffectorCommand msg1;
           msg1.id = 65538;
           msg1.command = "release";
           msg1.args = "";
           msg1.sender = "pbd2";
           msg1.sequence = 3;
           ROS_INFO("LEFT OPEN PUBLISHING");
           pub_left_gripper.publish(msg1);
           ros::Duration(4).sleep();
           msg1.id = 65538;
           msg1.command = "grip";
           msg1.args = "";
           msg1.sender = "pbd2";
           msg1.sequence = 3;
           pub_left_gripper.publish(msg1);
       }
      loop_rate.sleep();
      ros::spinOnce();


    }

    return 0;
}
