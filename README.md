# Pbd2 ros package

## What is pbd2 ?

Pbd2 is a ros package designed to be used with Baxter's programming by demonstration. In a pickup and release object task,
it allows Baxter to autonomously open and close its gripper.
This package has been tested with ros indigo and kinetic.

## How pbd2 works ?

pbd2 computes the distance between its grippers and the objects on the table. If the distance goes under a threshold and if the
gripper is closed and not holding an object, it will open. If the distance goes under another smaller threshold and if the gripper
is open, it will close.

The distance is computed either using PITT or using Baxter's infrared sensors, located on the grippers.
In PITT mode, the release will be autonomous as soon as the object will return at its initial position.

Documentation is available inside the src folder.

## How to use pbd2 ?

In pbd2.cpp change DISTANCE_MEASURMENT_MODE
* 1 for using pbd2 with PITT
* 2 for using pbd2 with iR sensors

### In pitt mode :

    *launch the kinect driver (see kinect_pitt_setup_guidelines.pdf)
    *launch pitt segmentation : roslaunch pitt_object_table_segmentation table_segmentation.launch
    *launch pbd2 node : rosrun pbd2 pbd2
    *wait until pitt measurments are done, the message SHUTDOWN LOOP will be printed on screen
    *launch the programation by demonstration : rosrun baxter_examples joint_recorder.py -f <example_file>
    *record and ctrl+c when over
    *launch the action server : rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
    *playback :  rosrun baxter_examples joint_trajectory_file_playback.py -f <example_file>

### In iR sensors mode :

    *launch pbd2 node : rosrun pbd2 pbd2
    *launch the programation by demonstration : rosrun baxter_examples joint_recorder.py -f <example_file>
    *record and ctrl+c when over
    *launch the action server : rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
    *playback :  rosrun baxter_examples joint_trajectory_file_playback.py -f <example_file>
