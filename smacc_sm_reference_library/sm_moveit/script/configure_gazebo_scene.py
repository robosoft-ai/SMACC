#!/usr/bin/python
import math
import random
import sys

import rospkg
import rospy
import tf
import xacro

from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel

CUBE_EDGE_LENGTH = 0.04

BLOCK_COLOR_MAPPINGS = [
    {"material": "Gazebo/Green"},
    {"material": "Gazebo/Red"},
    {"material": "Gazebo/Blue"},
    {"material": "Gazebo/Green"},
    {"material": "Gazebo/Red"},
    {"material": "Gazebo/Blue"},
    {"material": "Gazebo/Red"},
    {"material": "Gazebo/Blue"},
    {"material": "Gazebo/Green"}
]


def spawn_urdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def load_xacro_file(file_path, mappings):
    urdf_doc = xacro.process_file(file_path, mappings=mappings)
    urdf_xml = urdf_doc.toprettyxml(indent='  ', encoding='utf-8')
    urdf_xml = urdf_xml.replace('\n', '')
    return urdf_xml


def spawn_xacro_urdf_model(name, path, pose, reference_frame, mappings):
    description_xml = load_xacro_file(path, mappings)
    spawn_urdf(name, description_xml, pose, reference_frame)

def spawn_xacro_sdf_model(name, path, pose, reference_frame, mappings):
    description_xml = load_xacro_file(path, mappings)
    spawn_sdf(name, description_xml, pose, reference_frame)

def spawn_urdf_model(name, path, pose, reference_frame):
    description_xml = ''
    with open(path, "r") as model_file:
        description_xml = model_file.read().replace('\n', '')

    spawn_urdf(name, description_xml, pose, reference_frame)


def spawn_sdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def spawn_sdf_model(name, path, pose, reference_frame):
    # Load Model SDF
    description_xml = ''
    with open(path, "r") as model_file:
        description_xml = model_file.read().replace('\n', '')
        spawn_sdf(name, description_xml, pose,reference_frame)



def load_gazebo_models():
    model_list = []

    world_reference_frame = "map"

    # sorting_demo model path
    gazebo_demo_models_path = rospkg.RosPack().get_path('sm_moveit') + "/models/"

    # Spawn Blocks Table
    blocks_table_name = "blocks_table"
    blocks_table_path = gazebo_demo_models_path + "table/model.sdf"
    blocks_table_pose = Pose(position=Point(x=0.75, y=0.0, z=0.0))

    spawn_sdf_model(blocks_table_name, blocks_table_path, blocks_table_pose, world_reference_frame)
    model_list.append(blocks_table_name)

    # Spawn Trays Table
    trays_table_name = "trays_table"
    trays_table_path = gazebo_demo_models_path + "table/model.sdf"
    trays_table_pose = Pose(position=Point(x=0.0, y=0.95, z=0.0))

    spawn_sdf_model(trays_table_name, trays_table_path, trays_table_pose, world_reference_frame)
    model_list.append(trays_table_name)

    # Spawn trays
    tray_path = gazebo_demo_models_path + "tray/tray.urdf.xacro"

    tray_poses = [Pose(position=Point(x=-0.3, y=0.7, z=0.7828)),
                  Pose(position=Point(x=0.0, y=0.7, z=0.7828)),
                  Pose(position=Point(x=0.3, y=0.7, z=0.7828))]

    for (i, pose) in enumerate(tray_poses):
        name = "tray{}".format(i)
        spawn_xacro_urdf_model(name, tray_path, pose, world_reference_frame, {})
        model_list.append(name)

    # Spawn blocks
    block_path = gazebo_demo_models_path + "block/block.urdf.xacro"

    block_poses = []
    k = 3
    for i in xrange(k):
        for j in xrange(k):
            q = tf.transformations.quaternion_from_euler(random.uniform(0, 2 * math.pi), random.uniform(0, 2 * math.pi),
                                                         random.uniform(0, 2 * math.pi))


            block_poses.append(Pose(position=Point(x=0.45 + j * 0.15 + random.uniform(-1, 1) * 0.03,
                                                   y=-0.15 + i * 0.15 + random.uniform(-1, 1) * 0.03, z=0.7725),
                                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))

            #block_poses.append(Pose(position=Point(x= 0.45 + j*0.12 +random.uniform(-1, 1)*0.03 , y= -0.15 + i * 0.15 +random.uniform(-1, 1)*0.03, z=0.7725)))

    """
    Pose(position=Point(x=0.60, y=0.1265, z=0.7725)),
    Pose(position=Point(x=0.80, y=0.12, z=0.7725)),
    Pose(position=Point(x=0.60, y=-0.1, z=0.7725)),
    Pose(position=Point(x=0.80, y=-0.1, z=0.7725)),
    Pose(position=Point(x=0.4225, y=-0.1, z=0.7725)),
    Pose(position=Point(x=0.60, y=-0.35, z=0.7725)),
    Pose(position=Point(x=0.80, y=-0.35, z=0.7725)),
    Pose(position=Point(x=0.4225, y=-0.35, z=0.7725))
    """

    for (i, (pose, color_mappings)) in enumerate(zip(block_poses, BLOCK_COLOR_MAPPINGS)):
        name = "block{}".format(i)

        mappings = {"edge_length" : str(CUBE_EDGE_LENGTH)}
        mappings.update(color_mappings)

        spawn_xacro_urdf_model(name, block_path, pose, world_reference_frame, mappings)
        model_list.append(name)

    return model_list, block_poses


def delete_gazebo_models(model_list):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        for model in model_list:
            resp_delete = delete_model(model)
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))


if __name__=="__main__":
    load_gazebo_models()