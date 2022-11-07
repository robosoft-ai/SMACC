#!/usr/bin/python
import rospy
from gazebo_msgs.msg import LinkStates
import tf
from moveit_python import PlanningSceneInterface


class FakePerceptionNode:
    def __init__(self):
        self.planning_scene = PlanningSceneInterface("map")

        self.tf_broacaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.pub = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.simulated_link_state_callback, queue_size=1
        )

        self.update_planning_scene = True
        self.last_update = rospy.Time.now()
        self.update_period = rospy.Duration(10)
        self.table_collision = True
        self.cube_collision = False

    def update(self):
        pass

    def simulated_link_state_callback(self, linksmsg):
        """
        simulated link state callback.

        string[] name
        geometry_msgs/Pose[] pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        geometry_msgs/Twist[] twist
          geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
        """
        cube_transforms = self.propagate_link_states_to_tf(linksmsg, "cube", "cube_", "map")
        table_transforms = self.propagate_link_states_to_tf(linksmsg, "table", "table_", "map")

        elapsed = rospy.Time.now() - self.last_update

        if elapsed > self.update_period:
            # if self.update_planning_scene:
            rospy.logdebug("updating planning scene")

            attached_objects = self.planning_scene.getKnownAttachedObjects()
            rospy.loginfo(attached_objects)

            # self.update_planning_scene = False
            if self.table_collision and "cube_0" not in attached_objects:
                for i, table_transf in enumerate(table_transforms):
                    # self.planning_scene.removeCollisionObject("table_" + str(i))
                    thickness = 0.12
                    pos = table_transf[0]
                    self.planning_scene.addBox(
                        "table_" + str(i),
                        1.2,
                        1.3,
                        0.001 + thickness,
                        pos[0],
                        pos[1],
                        0.7 - thickness * 0.5,
                    )  # 0.68

            if self.cube_collision and "cube_0" not in attached_objects:
                for i, cube_transf in enumerate(cube_transforms):
                    # self.planning_scene.removeCollisionObject("cube_" + str(i))
                    pos = cube_transf[0]
                    self.planning_scene.addCube("cube_" + str(i), 0.06, pos[0], pos[1], pos[2])
                    # self.cube_collision = False

    def propagate_link_states_to_tf(self, linksmsg, link_name_filter, object_prefix, global_frame):
        filteredLinkPoses = [
            b for i, b in enumerate(linksmsg.pose) if link_name_filter in linksmsg.name[i]
        ]

        transforms = []
        for i, cubepose in enumerate(filteredLinkPoses):
            quat = [
                cubepose.orientation.x,
                cubepose.orientation.y,
                cubepose.orientation.z,
                cubepose.orientation.w,
            ]
            trans = [cubepose.position.x, cubepose.position.y, cubepose.position.z]
            self.tf_broacaster.sendTransform(
                trans, quat, rospy.Time.now(), object_prefix + str(i), global_frame
            )

            transforms.append([trans, quat])

        return transforms


if __name__ == "__main__":

    rospy.init_node("fake_perception_node")

    fpn = FakePerceptionNode()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        fpn.update()
        r.sleep()
