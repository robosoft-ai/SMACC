import smacc_msgs
import smacc_msgs.msg
import rospy

rospy.init_node("fake_state_machine")

pub = rospy.Publisher("/server_name/smacc/container_status", smacc_msgs.msg.SmaccContainerStatus,queue_size=1)

pub2 = rospy.Publisher("/server_name/smacc/container_structure", smacc_msgs.msg.SmaccContainerStructure,queue_size=1)

while not rospy.is_shutdown():
    msg = smacc_msgs.msg.SmaccContainerStatus()
    msg.path="/SM_ROOT"
    msg.initial_states = ["FOO"]
    msg.active_states = ["None"]
    msg.local_data = ""
    msg.info = "HEARTBEAT"

    pub.publish(msg)

    msg = smacc_msgs.msg.SmaccContainerStructure()
    msg.path="/SM_ROOT"
    msg.children= ["FOO", "BAR"]
    msg.internal_outcomes= ["outcome1", "outcome2", "outcome2"]
    msg.outcomes_from = ["FOO", "FOO", "BAR"]
    msg.outcomes_to = ["BAR", "outcome4", "FOO"]
    msg.container_outcomes = ["outcome4", "outcome5"]

    pub2.publish(msg)

    rospy.loginfo("step")

    rospy.sleep(1)


