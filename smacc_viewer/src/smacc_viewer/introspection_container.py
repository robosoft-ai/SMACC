
import roslib;
import rospy
from std_msgs.msg import Header

import pickle
import threading

import rostopic

from smacc_msgs.msg import SmaccContainerStatus,SmaccContainerInitialStatusCmd,SmaccContainerStructure

__all__ = ['IntrospectionClient']

# Topic names
STATUS_TOPIC = '/smacc/status'
INIT_TOPIC = '/smacc/container_init'


from smacc_viewer.smacc_user_data import UserData

class IntrospectionClient():
    def get_servers(self):
        """Get the base names that are broadcasting smacc states."""

        # Get the currently broadcasted smacc introspection topics
        topics = rostopic.find_by_type('smacc_msgs/SmaccStatus')
        rootservernames= [t[:t.rfind(STATUS_TOPIC)] for t in topics]
        
        return rootservernames

        #return [t[:t.rfind(STATUS_TOPIC)] for t in topics]

    def set_initial_state(self,
            server,
            path,
            initial_states,
            initial_userdata = UserData(),
            timeout = None):
        """Set the initial state of a smacc server.
        
        @type server: string
        @param server: The name of the introspection server to which this client
        should connect.

        @type path: string
        @param path: The path to the target container in the state machine.

        @type initial_states: list of string
        @param inital_state: The state the target container should take when it
        starts. This is as list of at least one state label.

        @type initial_userdata: UserData
        @param initial_userdata: The userdata to inject into the target container.

        @type timeout: rospy.Duration
        @param timeout: Timeout for this call. If this is set to None, it will not
        block, and the initial state may not be set before the target state machine
        goes active.
        """

        # Construct initial state command
        initial_status_msg = SmaccContainerInitialStatusCmd(
                path = path,
                initial_states = initial_states,
                local_data = pickle.dumps(initial_userdata._data,2))

        # A status message to receive confirmation that the state was set properly
        msg_response = SmaccContainerStatus()

        # Define a local callback to just stuff a local message
        def local_cb(msg, msg_response):
            rospy.logdebug("Received status response: "+str(msg))
            msg_response.path = msg.path
            msg_response.initial_states = msg.initial_states
            msg_response.local_data = msg.local_data

        # Create a subscriber to verify the request went through
        state_sub = rospy.Subscriber(server+STATUS_TOPIC, SmaccContainerStatus,
                callback=local_cb, callback_args=msg_response)

        # Create a publisher to send the command
        rospy.logdebug("Sending initial state command: "+str(initial_status_msg.path)+" on topic '"+server+INIT_TOPIC+"'")
        init_pub = rospy.Publisher(server+INIT_TOPIC,
                SmaccContainerInitialStatusCmd, queue_size=1)
        init_pub.publish(initial_status_msg)

        start_time = rospy.Time.now()

        # Block until we get a new state back 
        if timeout is not None:
            while rospy.Time.now() - start_time < timeout:
                # Send the initial state command
                init_pub.publish(initial_status_msg)

                # Filter messages that are from other containers
                if msg_response.path == path:
                    # Check if the heartbeat came back to match
                    state_match = all([s in msg_response.initial_states for s in initial_states])
                    local_data = UserData()
                    local_data._data = pickle.loads(msg_response.local_data)
                    ud_match = all([\
                            (key in local_data and local_data._data[key] == initial_userdata._data[key])\
                            for key in initial_userdata._data])

                    rospy.logdebug("STATE MATCH: "+str(state_match)+", UD_MATCH: "+str(ud_match))

                    if state_match and ud_match:
                        return True
                rospy.sleep(0.3)
            return False

class ContainerProxy():
    """smacc Container Introspection proxy.

    This class is used as a container for introspection and debugging.
    """
    def __init__(self, server_name, container, path, update_rate=rospy.Duration(2.0)):
        """Constructor for tree-wide data structure.
        """
        self._path = path
        self._container = container
        self._update_rate = update_rate
        self._status_pub_lock = threading.Lock()

        # Advertise init service
        self._init_cmd = rospy.Subscriber(
                server_name + INIT_TOPIC,
                SmaccContainerInitialStatusCmd,
                self._init_cmd_cb)

        # Advertise structure publisher
        self._structure_pub = rospy.Publisher(
                name=server_name + STRUCTURE_TOPIC,
                data_class=SmaccContainerStructure,
                queue_size=1)

        # Advertise status publisher
        self._status_pub = rospy.Publisher(
                name=server_name + STATUS_TOPIC,
                data_class=SmaccContainerStatus,
                queue_size=1)

        # Set transition callback
        container.register_transition_cb(self._transition_cb)

        # Create thread to constantly publish
        self._status_pub_thread = threading.Thread(name=server_name+':status_publisher',target=self._status_pub_loop)

        self._structure_pub_thread = threading.Thread(name=server_name+':structure_publisher',target=self._structure_pub_loop)

        self._keep_running = False

    def start(self):
        self._keep_running = True
        self._status_pub_thread.start()
        self._structure_pub_thread.start()
    
    def stop(self):
        self._keep_running = False

    def _status_pub_loop(self):
        """Loop to publish the status and structure heartbeats."""
        while not rospy.is_shutdown() and self._keep_running:
            #TODO
            self._publish_status('HEARTBEAT')
            try:
                end_time = rospy.Time.now() + self._update_rate
                while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                    rospy.sleep(0.1)
            except:
                pass

    def _structure_pub_loop(self):
        """Loop to publish the status and structure heartbeats."""
        while not rospy.is_shutdown() and self._keep_running:
            self._publish_structure('HEARTBEAT')
            try:
                end_time = rospy.Time.now() + self._update_rate
                while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                    rospy.sleep(0.1)
            except:
                pass

    def _publish_structure(self, info_str=''):
        path = self._path
        children = list(self._container.get_children().keys())

        internal_outcomes = [] 
        outcomes_from = [] 
        outcomes_to = [] 
        for (outcome, from_label, to_label) in self._container.get_internal_edges():
            internal_outcomes.append(str(outcome))
            outcomes_from.append(str(from_label))
            outcomes_to.append(str(to_label))
        container_outcomes = self._container.get_registered_outcomes()

        # Construct structure message
        structure_msg = SmaccContainerStructure(
                Header(stamp = rospy.Time.now()),
                path,
                children,
                internal_outcomes,
                outcomes_from,
                outcomes_to,
                container_outcomes)
        try:
            self._structure_pub.publish(structure_msg)
        except:
            if not rospy.is_shutdown():
                rospy.logerr("Publishing smacc introspection structure message failed.")

    def _publish_status(self, info_str=''):
        """Publish current state of this container."""
        # Construct messages
        with self._status_pub_lock:
            path = self._path
            
            #print str(structure_msg)
            # Construct status message
            #print self._container.get_active_states()
            state_msg = SmaccContainerStatus(
                    Header(stamp = rospy.Time.now()),
                    path,
                    self._container.get_initial_states(),
                    self._container.get_active_states(),
                    pickle.dumps(self._container.userdata._data,2),
                    info_str)
            # Publish message
            self._status_pub.publish(state_msg)

    ### Transition reporting
    def _transition_cb(self, *args, **kwargs):
        """Transition callback, passed to all internal nodes in the tree.
        This callback locks an internal mutex, preventing any hooked transitions
        from occurring while we're walking the tree.
        """
        info_str = (str(args) + ', ' + str(kwargs))
        rospy.logdebug("Transitioning: "+info_str)
        self._publish_status(info_str)

    def _init_cmd_cb(self, msg):
        """Initialize a tree's state and userdata."""
        initial_states = msg.initial_states
        local_data = msg.local_data

        # Check if this init message is directed at this path
        rospy.logdebug('Received init message for path: '+msg.path+' to '+str(initial_states))
        if msg.path == self._path:
            if all(s in self._container.get_children() for s in initial_states):
                ud = UserData()
                ud._data = pickle.loads(msg.local_data)
                rospy.logdebug("Setting initial state in smacc path: '"+self._path+"' to '"+str(initial_states)+"' with userdata: "+str(ud._data))

                # Set the initial state
                self._container.set_initial_state(
                        initial_states,
                        ud)
                # Publish initial state
                self._publish_status("REMOTE_INIT")
            else:
                rospy.logerr("Attempting to set initial state in container '"+self._path+"' to '"+str(initial_states)+"', but this container only has states: "+str(self._container.get_children()))