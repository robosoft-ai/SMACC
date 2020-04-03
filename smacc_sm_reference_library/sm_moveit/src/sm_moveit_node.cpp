// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <angles/angles.h>

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "l_gripper_finger_joint";
  posture.joint_names[1] = "r_gripper_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = -0.0;
  posture.points[0].positions[1] = -0.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "l_gripper_finger_joint";
  posture.joint_names[1] = "r_gripper_finger_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group, const geometry_msgs::Pose &object_pose)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  //grasps[0].grasp_pose.header.frame_id = "panda_link0";
  //tf2::Quaternion orientation;
  //orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  //grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  //grasps[0].grasp_pose.pose.position.x = 0.415;
  //grasps[0].grasp_pose.pose.position.y = 0;
  //grasps[0].grasp_pose.pose.position.z = 0.5;

  grasps[0].grasp_pose.header.frame_id = "/map";

  grasps[0].grasp_pose.pose = object_pose;
  //grasps[0].grasp_pose.pose.position.x -= 0.0;
  //grasps[0].grasp_pose.pose.position.z += 0.2;
  tf2::Quaternion orientation;
  //orientation.setRPY(0, M_PI / 2, 0);
  grasps[0].grasp_pose.pose.orientation.w = 1;
  //grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  // // Setting pre-grasp approach
  // // ++++++++++++++++++++++++++
  // /* Defined with respect to frame_id */
  // grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  // /* Direction is set as positive x axis */
  // grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  // grasps[0].pre_grasp_approach.min_distance = 0.095;
  // grasps[0].pre_grasp_approach.desired_distance = 0.115;

  //openGripper(grasps[0].pre_grasp_posture);

  grasps[0].pre_grasp_approach.direction.header.frame_id = "/map";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.0;
  grasps[0].pre_grasp_approach.desired_distance = 0.;

  //closedGripper(grasps[0].grasp_posture);

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "/map";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.0;
  grasps[0].post_grasp_retreat.desired_distance = 0.0;

  move_group.setSupportSurfaceName("table_0");
  // Call pick to pick up the object using the grasps given
  move_group.pick("cube_0", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface &arm)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  arm.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  arm.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sm_moveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  cl_move_base_z::Pose cube_pose("/cube_0", "/map");
  cube_pose.waitTransformUpdate();

  ros::WallDuration(6.0).sleep();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm_with_torso");
  group.setPlanningTime(1.0);

  ROS_INFO_STREAM("CubePose: " << cube_pose.get());

  ROS_INFO("------- TESTING PLAN --------");
  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id = "/map";
  target_pose1.header.stamp = ros::Time::now();
  target_pose1.pose= cube_pose.get();
  target_pose1.pose.position.x -= 0;
  //target_pose1.position.y = -target_pose1.position.y;
  target_pose1.pose.position.z += 0.3;

  auto cubeYawOnTable = tf::getYaw(target_pose1.pose.orientation);
  auto degrees90 = M_PI/2;

  while(cubeYawOnTable > degrees90)
  {
    cubeYawOnTable -= degrees90;
  }

  while(cubeYawOnTable < -degrees90)
  {
    cubeYawOnTable += degrees90;
  }

  ROS_INFO("picking yaw: %lf", cubeYawOnTable);
  auto quat = tf::createQuaternionFromRPY(0, M_PI / 2, cubeYawOnTable);
  tf::quaternionTFToMsg(quat, target_pose1.pose.orientation);

  group.setPoseTarget(target_pose1);
  group.setPoseReferenceFrame("/map");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success)
  {
    group.execute(my_plan);

    ros::WallDuration(15.0).sleep();
    ROS_INFO("------- CARTESIAN TEST --------");
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(target_pose1.pose); // up and out
    auto target_pose2 = target_pose1.pose;

    target_pose2.position.z -= 0.12;
    //target_pose2.position.x -= 0.15;
    waypoints.push_back(target_pose2); // left

    group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.01, // eef_step
                                                 0.0,  // jump_threshold
                                                 trajectory);

    auto grasp_pose_plan = my_plan;
    grasp_pose_plan.trajectory_ = trajectory;
    group.execute(grasp_pose_plan);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);

    ros::WallDuration(5.0).sleep();         
    ROS_INFO("------- RETREAT TEST --------");
    /*planning_scene_interface.removeCollisionObjects({"cube_0"});
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group_variable_values[group_variable_values.size() -1 ]=0;
    group_variable_values[group_variable_values.size() -2 ]=0;
  
    
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if(!success)
      exit(0);*/

    waypoints.clear();
    
    target_pose2 = target_pose1.pose;
    target_pose2.position.z -= 0.12;
    waypoints.push_back(target_pose2); // up and out
    waypoints.push_back(target_pose1.pose); // up and out
    
    group.setMaxVelocityScalingFactor(0.1);

    planning_scene_interface.removeCollisionObjects({"cube_0"});
    moveit_msgs::RobotTrajectory retreat_trajectory;
    fraction = group.computeCartesianPath(waypoints,
                                                 0.01, // eef_step
                                                 0.0,  // jump_threshold
                                                 retreat_trajectory);

    auto retreat_plan = my_plan;
    retreat_plan.trajectory_ = retreat_trajectory;
    group.execute(retreat_plan);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);

    ros::WallDuration(15.0).sleep();
    ROS_INFO("------- PICKING PIPELINE --------");
    group.setPlanningTime(4.0);
    pick(group, target_pose1.pose);
  }
  /*addCollisionObjects(planning_scene_interface);


  // Wait a bit for ROS things to initialize
  

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();*/
  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL
