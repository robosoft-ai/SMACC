#!/usr/bin/env python

# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# Copyright (c) 2013, Jonathan Bohren, The Johns Hopkins University
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#   * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#   * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren 

import rospy
import rospkg
import sys
sys.path.append('../devel/lib/python2.7/dist-packages/')

from smacc_msgs.msg import SmaccContainerStatus,SmaccContainerInitialStatusCmd,SmaccContainerStructure, SmaccStateMachine

import sys
import os
import threading
import pickle
import pprint
import copy
import StringIO
import colorsys
import time

import wxversion
if wxversion.checkInstalled("2.8"):
    wxversion.select("2.8")
else:
    print("wxversion 2.8 is not installed, installed versions are {}".format(wxversion.getInstalled()))
import wx
import wx.richtext

import textwrap

STATUS_TOPIC = '/smacc/container_status'
INIT_TOPIC = '/smacc/container_init'
STRUCTURE_TOPIC = '/smacc/container_structure'
STATE_MACHINE_DESC_TOPIC = '/smacc/state_machine_description'

## this import system (or ros-released) xdot
# import xdot
## need to import currnt package, but not to load this file
# http://stackoverflow.com/questions/6031584/importing-from-builtin-library-when-module-with-same-name-exists
def import_non_local(name, custom_name=None):
    import imp, sys

    custom_name = custom_name or name

    path = filter(lambda x: x != os.path.dirname(os.path.abspath(__file__)), sys.path)
    f, pathname, desc = imp.find_module(name, path)

    module = imp.load_module(custom_name, f, pathname, desc)
    if f:
        f.close()

    return module

smacc_viewer = import_non_local('smacc_viewer')
from smacc_viewer import xdot
##
from smacc_viewer.smacc_user_data import UserData
from smacc_viewer.introspection_container import IntrospectionClient

### Helper Functions
def graph_attr_string(attrs):
    """Generate an xdot graph attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ';\n'.join(attrs_strs)+';\n'

def attr_string(attrs):
    """Generate an xdot node attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ' ['+(', '.join(attrs_strs))+']'

def get_parent_path(path):
    """Get the parent path of an xdot node."""
    path_tokens = path.split('/')
    if len(path_tokens) > 2:
        parent_path = '/'.join(path_tokens[0:-1])
    else:
        parent_path = '/'.join(path_tokens[0:1])
    return parent_path

def get_label(path):
    """Get the label of an xdot node."""
    path_tokens = path.split('/')
    return path_tokens[-1]

def hex2t(color_str):
    """Convert a hexadecimal color strng into a color tuple."""
    color_tuple = [int(color_str[i:i+2],16)/255.0    for i in range(1,len(color_str),2)]
    return color_tuple

class ContainerNode():
    """
    This class represents a given container in a running smacc system. 

    Its primary use is to generate dotcode for a smacc container. It has
    methods for responding to structure and status messages from a smacc
    introspection server, as well as methods for updating the styles of a 
    graph once it's been drawn.
    """
    def __init__(self, server_name, msg):
        # Store path info
        self._server_name = server_name
        self._path = msg.path
        splitpath = msg.path.split('/')
        self._label = splitpath[-1]
        self._dir = '/'.join(splitpath[0:-1])

        self._children = msg.children
        self._internal_outcomes = msg.internal_outcomes
        self._outcomes_from = msg.outcomes_from
        self._outcomes_to = msg.outcomes_to

        self._container_outcomes = msg.container_outcomes

        # Status
        self._initial_states = []
        self._active_states = []
        self._last_active_states = []
        self._local_data = UserData()
        self._info = ''

    def update_structure(self, msg):
        """Update the structure of this container from a given message. Return True if anything changes."""
        needs_update = False

        if self._children != msg.children\
                or self._internal_outcomes != msg.internal_outcomes\
                or self._outcomes_from != msg.outcomes_from\
                or self._outcomes_to != msg.outcomes_to\
                or self._container_outcomes != msg.container_outcomes:
            needs_update = True

        if needs_update:
            self._children = msg.children
            self._internal_outcomes = msg.internal_outcomes
            self._outcomes_from = msg.outcomes_from
            self._outcomes_to = msg.outcomes_to

            self._container_outcomes = msg.container_outcomes

        return needs_update

    def update_status(self, msg):
        """Update the known userdata and active state set and return True if the graph needs to be redrawn."""

        # Initialize the return value
        needs_update = False

        # Check if the initial states or active states have changed
        if set(msg.initial_states) != set(self._initial_states):
            self._structure_changed = True
            needs_update = True
        if set(msg.active_states) != set(self._active_states):
            needs_update = True

        # Store the initial and active states
        self._initial_states = msg.initial_states
        self._last_active_states = self._active_states
        self._active_states = msg.active_states

        # Unpack the user data
        while not rospy.is_shutdown():
            try:
                #self._local_data._data = pickle.loads(msg.local_data)
                break
            except ImportError as ie:
                # This will only happen once for each package
                modulename = ie.args[0][16:]
                packagename = modulename[0:modulename.find('.')]
                roslib.load_manifest(packagename)
                #self._local_data._data = pickle.loads(msg.local_data)

        # Store the info string
        self._info = msg.info

        return needs_update


    def get_dotcode(self, selected_paths, closed_paths, depth, max_depth, containers, show_all, label_wrapper, attrs={}):
        """Generate the dotcode representing this container.
        
        @param selected_paths: The paths to nodes that are selected
        @closed paths: The paths that shouldn't be expanded
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to which we should traverse the tree
        @param containers: A dict of containers keyed by their paths
        @param show_all: True if implicit transitions should be shown
        @param label_wrapper: A text wrapper for wrapping element names
        @param attrs: A dict of dotcode attributes for this cluster
        """

        dotstr = 'subgraph "cluster_%s" {\n' % (self._path)
        if depth == 0:
            #attrs['style'] = 'filled,rounded'
            attrs['color'] = '#00000000'
            attrs['fillcolor'] = '#0000000F'
        #attrs['rank'] = 'max'

        #,'succeeded','aborted','preempted'attrs['label'] = self._label
        dotstr += graph_attr_string(attrs)

        # Add start/terimate target
        proxy_attrs = {
                'URL':self._path,
                'shape':'plaintext',
                'color':'gray',
                'fontsize':'18',
                'fontweight':'18',
                'rank':'min',
                'height':'0.01'}
        proxy_attrs['label'] = '\\n'.join(label_wrapper.wrap(self._label))
        dotstr += '"%s" %s;\n' % (
                '/'.join([self._path,'__proxy__']),
                attr_string(proxy_attrs))

        # Check if we should expand this container
        if max_depth == -1 or depth <= max_depth:
            # Add container outcomes
            dotstr += 'subgraph "cluster_%s" {\n' % '/'.join([self._path,'__outcomes__'])
            outcomes_attrs = {
                    'style':'rounded,filled',
                    'rank':'sink',
                    'color':'#FFFFFFFF',#'#871C34',
                    'fillcolor':'#FFFFFF00'#'#FE464f3F'#'#DB889A'
                    }
            dotstr += graph_attr_string(outcomes_attrs)

            for outcome_label in self._container_outcomes:
                outcome_path = ':'.join([self._path,outcome_label])
                outcome_attrs = {
                        'shape':'box',
                        'height':'0.3',
                        'style':'filled,rounded',
                        'fontsize':'12',
                        'fillcolor':'#FE464f',#'#EDC2CC',
                        'color':'#780006',#'#EBAEBB',
                        'fontcolor':'#780006',#'#EBAEBB',
                        'label':'\\n'.join(label_wrapper.wrap(outcome_label)),
                        'URL':':'.join([self._path,outcome_label])
                        }
                dotstr += '"%s" %s;\n' % (outcome_path,attr_string(outcome_attrs))
            dotstr += "}\n"

            # Iterate over children
            for child_label in self._children:
                child_attrs = {
                        'style':'filled,setlinewidth(2)',
                        'color':'#000000FF',
                        'fillcolor':'#FFFFFF00'
                        }

                child_path = '/'.join([self._path,child_label])
                # Generate dotcode for children
                if child_path in containers:
                    child_attrs['style'] += ',rounded'

                    dotstr += containers[child_path].get_dotcode(
                            selected_paths,
                            closed_paths,
                            depth+1, max_depth,
                            containers,
                            show_all,
                            label_wrapper,
                            child_attrs)
                else:
                    child_attrs['label'] = '\\n'.join(label_wrapper.wrap(child_label))
                    child_attrs['URL'] = child_path
                    dotstr += '"%s" %s;\n' % (child_path, attr_string(child_attrs))

            # Iterate over edges
            internal_edges = zip(
                    self._internal_outcomes,
                    self._outcomes_from,
                    self._outcomes_to)

            # Add edge from container label to initial state
            internal_edges += [('','__proxy__',initial_child) for initial_child in self._initial_states]

            has_explicit_transitions = []
            for (outcome_label,from_label,to_label) in internal_edges:
                if to_label != 'None' or outcome_label == to_label:
                    has_explicit_transitions.append(from_label)

            # Draw internal edges
            for (outcome_label,from_label,to_label) in internal_edges:

                from_path = '/'.join([self._path, from_label])

                if show_all \
                        or to_label != 'None'\
                        or from_label not in has_explicit_transitions \
                        or (outcome_label == from_label) \
                        or from_path in containers:
                    # Set the implicit target of this outcome
                    if to_label == 'None':
                        to_label = outcome_label

                    to_path = '/'.join([self._path, to_label])

                    edge_attrs = {
                            'URL':':'.join([from_path,outcome_label,to_path]),
                            'fontsize':'12',
                            'label':'\\n'.join(label_wrapper.wrap(outcome_label))}
                    edge_attrs['style'] = 'setlinewidth(2)'

                    # Hide implicit
                    #if not show_all and to_label == outcome_label:
                    #    edge_attrs['style'] += ',invis'

                    from_key = '"%s"' % from_path
                    if from_path in containers:
                        if max_depth == -1 or depth+1 <= max_depth:
                            from_key = '"%s:%s"' % ( from_path, outcome_label)
                        else:
                            edge_attrs['ltail'] = 'cluster_'+from_path
                            from_path = '/'.join([from_path,'__proxy__'])
                            from_key = '"%s"' % ( from_path )

                    to_key = ''
                    if to_label in self._container_outcomes:
                        to_key = '"%s:%s"' % (self._path,to_label)
                        edge_attrs['color'] = '#00000055'# '#780006'
                    else:
                        if to_path in containers:
                            edge_attrs['lhead'] = 'cluster_'+to_path
                            to_path = '/'.join([to_path,'__proxy__'])
                        to_key = '"%s"' % to_path

                    dotstr += '%s -> %s %s;\n' % (
                            from_key, to_key, attr_string(edge_attrs))

        dotstr += '}\n'
        return dotstr

    def set_styles(self, selected_paths, depth, max_depth, items, subgraph_shapes, containers):
        """Update the styles for a list of containers without regenerating the dotcode.

        This function is called recursively to update an entire tree.
        
        @param selected_paths: A list of paths to nodes that are currently selected.
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to traverse into the tree
        @param items: A dict of all the graph items, keyed by url
        @param subgraph_shapes: A dictionary of shapes from the rendering engine
        @param containers: A dict of all the containers
        """

        # Color root container
        """
        if depth == 0:
            container_shapes = subgraph_shapes['cluster_'+self._path]
            container_color = (0,0,0,0)
            container_fillcolor = (0,0,0,0)

            for shape in container_shapes:
                shape.pen.color = container_color
                shape.pen.fillcolor = container_fillcolor
                """

        # Color shapes for outcomes

        # Color children
        if max_depth == -1 or depth <= max_depth:
            # Iterate over children
            for child_label in self._children:
                child_path = '/'.join([self._path,child_label])

                child_color = [0.5,0.5,0.5,1]
                child_fillcolor = [1,1,1,1]
                child_linewidth = 2

                active_color = hex2t('#5C7600FF')
                active_fillcolor = hex2t('#C0F700FF')

                initial_color = hex2t('#000000FF')
                initial_fillcolor = hex2t('#FFFFFFFF')

                if child_label in self._active_states:
                    # Check if the child is active
                    child_color = active_color
                    child_fillcolor = active_fillcolor
                    child_linewidth = 5
                elif child_label in self._initial_states:
                    # Initial style
                    #child_fillcolor = initial_fillcolor
                    child_color = initial_color
                    child_linewidth = 2

                # Check if the child is selected
                if child_path in selected_paths:
                    child_color = hex2t('#FB000DFF')

                # Generate dotcode for child containers 
                if child_path in containers:
                    subgraph_id = 'cluster_'+child_path
                    if subgraph_id in subgraph_shapes:
                        if child_label in self._active_states:
                            child_fillcolor[3] = 0.25
                        elif 0 and child_label in self._initial_states:
                            child_fillcolor[3] = 0.25
                        else:
                            if max_depth > 0:
                                v = 1.0-0.25*((depth+1)/float(max_depth))
                            else:
                                v = 0.85
                            child_fillcolor = [v,v,v,1.0]

                        
                        for shape in subgraph_shapes['cluster_'+child_path]:
                            pen = shape.pen
                            if len(pen.color) > 3:
                                pen_color_opacity = pen.color[3]
                                if pen_color_opacity < 0.01:
                                    pen_color_opacity = 0
                            else:
                                pen_color_opacity = 0.5
                            shape.pen.color = child_color[0:3]+[pen_color_opacity]
                            shape.pen.fillcolor = [child_fillcolor[i] for i in range(min(3,len(pen.fillcolor)))]
                            shape.pen.linewidth = child_linewidth

                        # Recurse on this child
                        containers[child_path].set_styles(
                                selected_paths,
                                depth+1, max_depth,
                                items,
                                subgraph_shapes,
                                containers)
                else:
                    if child_path in items:
                        for shape in items[child_path].shapes:
                            if not isinstance(shape,xdot.xdot.TextShape):
                                shape.pen.color = child_color
                                shape.pen.fillcolor = child_fillcolor
                                shape.pen.linewidth = child_linewidth
                    else:
                        #print child_path+" NOT IN "+str(items.keys())
                        pass

class SmaccViewerFrame(wx.Frame):
    """
    This class provides a GUI application for viewing smacc plans.
    """
    def __init__(self):
        wx.Frame.__init__(self, None, -1, "smacc Viewer", size=(720,480))

        # Create graph
        self._containers = {}
        self._top_containers = {}
        self._update_cond = threading.Condition()
        self._needs_refresh = True
        self.dotstr = ''

        vbox = wx.BoxSizer(wx.VERTICAL)


        # Create Splitter
        self.content_splitter = wx.SplitterWindow(self, -1,style = wx.SP_LIVE_UPDATE)
        self.content_splitter.SetMinimumPaneSize(24)
        self.content_splitter.SetSashGravity(0.85)


        # Create viewer pane
        viewer = wx.Panel(self.content_splitter,-1)

        # Create smacc viewer 
        nb = wx.Notebook(viewer,-1,style=wx.NB_TOP | wx.WANTS_CHARS)
        viewer_box = wx.BoxSizer()
        viewer_box.Add(nb,1,wx.EXPAND | wx.ALL, 4)
        viewer.SetSizer(viewer_box)

        # Create graph view
        graph_view = wx.Panel(nb,-1)
        gv_vbox = wx.BoxSizer(wx.VERTICAL)
        graph_view.SetSizer(gv_vbox)

        # Construct toolbar
        toolbar = wx.ToolBar(graph_view, -1)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"Path: "))

        # Path list
        self.path_combo = wx.ComboBox(toolbar, -1, style=wx.CB_DROPDOWN)
        self.path_combo .Bind(wx.EVT_COMBOBOX, self.set_path)
        self.path_combo.Append('/')
        self.path_combo.SetValue('/')
        toolbar.AddControl(self.path_combo)

        # Depth spinner
        self.depth_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=-1,
                max=1337,
                initial=-1)
        self.depth_spinner.Bind(wx.EVT_SPINCTRL,self.set_depth)
        self._max_depth = -1
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Depth: "))
        toolbar.AddControl(self.depth_spinner)

        # Label width spinner
        self.width_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=1,
                max=1337,
                initial=40)
        self.width_spinner.Bind(wx.EVT_SPINCTRL,self.set_label_width)
        self._label_wrapper = textwrap.TextWrapper(40,break_long_words=True)
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Label Width: "))
        toolbar.AddControl(self.width_spinner)

        # Implicit transition display
        toggle_all = wx.ToggleButton(toolbar,-1,'Show Implicit')
        toggle_all.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_all_transitions)
        self._show_all_transitions = False

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddControl(toggle_all)

        toggle_auto_focus = wx.ToggleButton(toolbar, -1, 'Auto Focus')
        toggle_auto_focus.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_auto_focus)
        self._auto_focus = False

        toolbar.AddControl(wx.StaticText(toolbar, -1, "    "))
        toolbar.AddControl(toggle_auto_focus)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddLabelTool(wx.ID_HELP, 'Help',
                wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
        toolbar.AddLabelTool(wx.ID_SAVE, 'Save',
                wx.ArtProvider.GetBitmap(wx.ART_FILE_SAVE,wx.ART_OTHER,(16,16)) )
        toolbar.Realize()

        self.Bind(wx.EVT_TOOL, self.ShowControlsDialog, id=wx.ID_HELP)
        self.Bind(wx.EVT_TOOL, self.SaveDotGraph, id=wx.ID_SAVE)

        # Create dot graph widget
        self.widget = xdot.wxxdot.WxDotWindow(graph_view, -1)

        gv_vbox.Add(toolbar, 0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        # Create tree view widget
        self.tree = wx.TreeCtrl(nb,-1,style=wx.TR_HAS_BUTTONS)
        nb.AddPage(graph_view,"Graph View")
        nb.AddPage(self.tree,"Tree View")


        # Create userdata widget
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.ud_win = wx.ScrolledWindow(self.content_splitter, -1)
        self.ud_gs = wx.BoxSizer(wx.VERTICAL)

        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Path:"),0, borders, border)

        self.path_input = wx.ComboBox(self.ud_win,-1,style=wx.CB_DROPDOWN)
        self.path_input.Bind(wx.EVT_COMBOBOX,self.selection_changed)
        self.ud_gs.Add(self.path_input,0,wx.EXPAND | borders, border)


        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Userdata:"),0, borders, border)

        self.ud_txt = wx.TextCtrl(self.ud_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.ud_gs.Add(self.ud_txt,1,wx.EXPAND | borders, border)
        
        # Add initial state button
        self.is_button = wx.Button(self.ud_win,-1,"Set as Initial State")
        self.is_button.Bind(wx.EVT_BUTTON, self.on_set_initial_state)
        self.is_button.Disable()
        self.ud_gs.Add(self.is_button,0,wx.EXPAND | wx.BOTTOM | borders, border)

        self.ud_win.SetSizer(self.ud_gs)


        # Set content splitter
        self.content_splitter.SplitVertically(viewer, self.ud_win, 512)

        # Add statusbar
        self.statusbar = wx.StatusBar(self,-1)

        # Add elements to sizer
        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        vbox.Add(self.statusbar, 0, wx.EXPAND)

        self.SetSizer(vbox)
        self.Center()

        # smacc introspection client
        self._client = IntrospectionClient()
        self._containers= {}
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}
        self._state_machine_subs = {}

        self.Bind(wx.EVT_IDLE,self.OnIdle)
        self.Bind(wx.EVT_CLOSE,self.OnQuit)

        # Register mouse event callback
        self.widget.register_select_callback(self.select_cb)
        self._path = '/'
        self._needs_zoom = True
        self._structure_changed = True
        self._statemachine_changed = True
        self.state_machine_msg = None
        self.state_machine_msg_first = True

        # Start a thread in the background to update the server list
        self._keep_running = True
        
        rospy.loginfo("starting update server list")
        self._server_list_thread = threading.Thread(target=self._update_server_list)
        self._server_list_thread.start()

        self._update_graph_thread = threading.Thread(target=self._update_graph)
        self._update_graph_thread.start()
        self._update_tree_thread = threading.Thread(target=self._update_tree)
        self._update_tree_thread.start()

        #other fields and style
        self.transition_color_by_tag = {"SUCCESS": '#99FF99', 'ABORT': '#FF9999',  'PREEMPT':'#FFFF99', 'CONTINUELOOP': '#AAAAFF', 'ENDLOOP': '#99FF99'}
        self.edge_color_by_tag = {"SUCCESS": '#22AA22', 'ABORT': '#FF4444', 'PREEMPT':'#AAAA44', 'CONTINUELOOP': '#4444FF', 'ENDLOOP': '#22AA22'}
        self.no_constraint_edge_by_tag = {"SUCCESS": False, 'ABORT': True, 'PREEMPT': True, 'CONTINUELOOP': False, 'ENDLOOP': False}

        self.transition_label_font_size = str(14)
        self.transition_node_font_size = str(20)

        self.first_message_file_flag=False

    def OnQuit(self,event):
        """Quit Event: kill threads and wait for join."""
        with self._update_cond:
            self._keep_running = False
            self._update_cond.notify_all()

        self._server_list_thread.join()
        self._update_graph_thread.join()
        self._update_tree_thread.join()
        
        event.Skip()

    def update_graph(self):
        """Notify all that the graph needs to be updated."""
        with self._update_cond:
            self._update_cond.notify_all()

    def on_set_initial_state(self, event):
        """Event: Change the initial state of the server."""
        state_path = self._selected_paths[0]
        parent_path = get_parent_path(state_path)
        state = get_label(state_path)

        server_name = self._containers[parent_path]._server_name
        self._client.set_initial_state(server_name,parent_path,[state],timeout = rospy.Duration(60.0))

    def set_path(self, event):
        """Event: Change the viewable path and update the graph."""
        self._path = self.path_combo.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_path(self, path):
        self._path = path
        self._needs_zoom = True
        self.path_combo.SetValue(path)
        self.update_graph()

    def set_depth(self, event):
        """Event: Change the maximum depth and update the graph."""
        self._max_depth = self.depth_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_max_depth(self, max_depth):
        self._max_depth = max_depth
        self.depth_spinner.SetValue(max_depth)
        self._needs_zoom = True
        self.update_graph()

    def set_label_width(self, event):
        """Event: Change the label wrapper width and update the graph."""
        self._label_wrapper.width = self.width_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def toggle_all_transitions(self, event):
        """Event: Change whether automatic transitions are hidden and update the graph."""
        self._show_all_transitions = not self._show_all_transitions
        self._structure_changed = True
        self.update_graph()

    def toggle_auto_focus(self, event):
        """Event: Enable/Disable automatically focusing"""
        self._auto_focus = not self._auto_focus
        self._needs_zoom = self._auto_focus
        self._structure_changed = True
        if not self._auto_focus:
            self._set_path('/')
            self._max_depth(-1)
        self.update_graph()

    def select_cb(self, item, event):
        """Event: Click to select a graph node to display user data and update the graph."""

        # Only set string status
        if not type(item.url) is str:
            return

        self.statusbar.SetStatusText(item.url)
        # Left button-up
        if event.ButtonUp(wx.MOUSE_BTN_LEFT):
            # Store this item's url as the selected path
            self._selected_paths = [item.url]
            # Update the selection dropdown
            self.path_input.SetValue(item.url)
            wx.PostEvent(
                    self.path_input.GetEventHandler(),
                    wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))
            self.update_graph()

    def selection_changed(self, event):
        """Event: Selection dropdown changed."""
        path_input_str = self.path_input.GetValue()

        # Check the path is non-zero length
        if len(path_input_str) > 0:
            # Split the path (state:outcome), and get the state path
            path = path_input_str.split(':')[0]

            # Get the container corresponding to this path, since userdata is
            # stored in the containers
            if path not in self._containers:
                parent_path = get_parent_path(path)
            else:
                parent_path = path

            if parent_path in self._containers:
                # Enable the initial state button for the selection
                self.is_button.Enable()

                # Get the container
                container = self._containers[parent_path]

                # Store the scroll position and selection
                pos = self.ud_txt.HitTestPos(wx.Point(0,0))
                sel = self.ud_txt.GetSelection()

                # Generate the userdata string
                #ud_str = ''
                #for (k,v) in container._local_data._data.iteritems():
                #    ud_str += str(k)+": "
                #    vstr = str(v)
                #    # Add a line break if this is a multiline value
                #    if vstr.find('\n') != -1:
                #        ud_str += '\n'
                #    ud_str+=vstr+'\n\n'

                # Set the userdata string
                #self.ud_txt.SetValue(ud_str)

                # Restore the scroll position and selection
                self.ud_txt.ShowPosition(pos[1])
                if sel != (0,0):
                    self.ud_txt.SetSelection(sel[0],sel[1])
            else:
                # Disable the initial state button for this selection
                self.is_button.Disable()

    def _structure_msg_update(self, msg, server_name):
        """Update the structure of the smacc plan (re-generate the dotcode)."""

        # Just return if we're shutting down
        if not self._keep_running:
            return

        # Get the node path
        path = msg.path
        pathsplit = path.split('/')
        pathsplit = [p.split("::")[-1] for p in pathsplit]
        path = "/".join(pathsplit)
        parent_path = '/'.join(pathsplit[0:-1])

        rospy.logdebug("RECEIVED: "+path)
        rospy.logdebug("CONTAINERS: "+str(self._containers.keys()))

        # Initialize redraw flag
        needs_redraw = False

        if path in self._containers:
            rospy.logdebug("UPDATING: "+path)

            # Update the structure of this known container
            needs_redraw = self._containers[path].update_structure(msg)
        else: 
            rospy.logdebug("CONSTRUCTING: "+path)

            # Create a new container
            container = ContainerNode(server_name, msg)
            self._containers[path] = container

            # Store this as a top container if it has no parent
            if parent_path == '':
                self._top_containers[path] = container

            # Append paths to selector
            print(path)
            self.path_combo.Append(path)
            
            self.path_input.Append(path)

            # We need to redraw thhe graph if this container's parent is already known
            if parent_path in self._containers:
                needs_redraw = True

        # Update the graph if necessary
        if needs_redraw:
            with self._update_cond:
                #self._structure_changed = True
                self._statemachine_changed
                self._needs_zoom = True # TODO: Make it so you can disable this
                self._update_cond.notify_all()

    def _state_machine_update(self,msg):
        self.state_machine_msg = copy.deepcopy(msg)
        #self._structure_changed = True

    def _status_msg_update(self, msg):
        """Process status messages."""

        # Check if we're in the process of shutting down
        if not self._keep_running:
            return

        if self._auto_focus and len(msg.info) > 0:
            self._set_path(msg.info)
            self._set_max_depth(msg.info.count('/')-1)

        # Get the path to the updating conainer
        path = msg.path
        rospy.logdebug("STATUS MSG: "+path)

        # Check if this is a known container
        if path in self._containers:
            # Get the container and check if the status update requires regeneration
            container = self._containers[path]
            if container.update_status(msg):
                with self._update_cond:
                    self._update_cond.notify_all()

            # TODO: Is this necessary?
            path_input_str = self.path_input.GetValue()
            if path_input_str == path or get_parent_path(path_input_str) == path:
                wx.PostEvent(
                        self.path_input.GetEventHandler(),
                        wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))

    def _update_graph(self):
        """This thread continuously updates the graph when it changes.

        The graph gets updated in one of two ways:

          1: The structure of the smacc plans has changed, or the display
          settings have been changed. In this case, the dotcode needs to be
          regenerated. 

          2: The status of the smacc plans has changed. In this case, we only
          need to change the styles of the graph.
        """
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                # Wait for the update condition to be triggered
                self._update_cond.wait()

                # Get the containers to update
                containers_to_update = {}
                if self._path in self._containers:
                    # Some non-root path
                    containers_to_update = {self._path:self._containers[self._path]}
                elif self._path == '/':
                    # Root path
                    containers_to_update = self._top_containers

                # Check if we need to re-generate the dotcode (if the structure changed)
                # TODO: needs_zoom is a misnomer
                if (self.state_machine_msg_first and self.state_machine_msg) and (self._structure_changed or self._needs_zoom):
                    self.state_machine_msg_first = False
                    dotstr = "digraph {\n\t"
                    dotstr += ';'.join([
                        "compound=true",
                        "outputmode=nodesfirst",
                        "labeljust=l",
                        "nodesep=0.2",
                        "minlen=2",
                        "mclimit=5",
                        "clusterrank=local",
                        "ranksep=0.2",
                        # "remincross=true",
                        # "rank=sink",
                        "ordering=\"out\"",
                        "rankdir=\"LR\""

                        ])
                    dotstr += ";\n"

                    # Generate the rest of the graph
                    # TODO: Only re-generate dotcode for containers that have changed
                    """
                    for path,tc in containers_to_update.iteritems():
                        dotstr += get_dotcode(
                                self._selected_paths,[],
                                0,self._max_depth,
                                self._containers,
                                self._show_all_transitions,
                                self._label_wrapper)
                    else:
                        dotstr += '"__empty__" [label="Path not available.", shape="plaintext"]'
                    """

                    dotstr += self.smacc_get_dotcode(
                                None,
                                None,
                                None,
                                None,
                                None,
                                None,
                                None)

                    dotstr += '\n}\n'

                    rospy.loginfo_throttle(1, dotstr)
                    self.serialize_dotstr_file(dotstr)
                    
                    
                    self.dotstr = dotstr
                    # Set the dotcode to the new dotcode, reset the flags
                    self.set_dotcode(dotstr,zoom=False)
                    self._structure_changed = False

                # Update the styles for the graph if there are any updates
                """
                for path,tc in containers_to_update.iteritems():
                    tc.set_styles(
                            self._selected_paths,
                            0,self._max_depth,
                            self.widget.items_by_url,
                            self.widget.subgraph_shapes,
                            self._containers)
                """

                # Redraw
                self.widget.Refresh()

 
    def serialize_dotstr_file(self, dotstr):
        if not self.first_message_file_flag:
            outfile=open("state_machine.dot","w")
            outfile.write(dotstr)
            outfile.close()

    
    def get_state_id_from_state_name(self, state_name, allstates):
        return "cluster_" + allstates.index(state_name)


    def get_lu_id(self, current_state, logic_unit):
        labelstr = logic_unit.type_name.split("::")[-1] 
        idlu = labelstr+ "_" + str(current_state.index)+"_"+ str(logic_unit.index)
        return idlu
    
    def node_id_from_event_source(self, event, current_state, logic_units_full_names, allstates, issource = True):
        sourcelabelstr = event.event_source.split("::")[-1]
        lrheadtail = ""
        if event.event_source in logic_units_full_names: 
            #LOGIC UNIT SOURCE CASE
            luindex = logic_units_full_names.index(event.event_source)
            sourcestr = sourcelabelstr + "_" + str(current_state.index)+"_"+ str(luindex)
        else:
            candidatestates = [st for st in allstates if st.name == event.event_source ]
            if len(candidatestates) >0: 
                # STATE SOURCE CASE
                event_source_state = candidatestates[0]

                if current_state != event_source_state:
                    # IN THIS CASE THE SOURCE OF THE EVENT IS ANOTHER STATE
                    #sourcestr = "cluster_"+str(candidatestates[0].index)
                    #sourcestr = candidatestates[0].name
                    if issource:
                        sourcestr = "internally_generated_"+str(event_source_state.index)
                    else:
                        sourcestr = "entry_"+str(event_source_state.index)

                    #lrheadtail = "cluster_"+str(candidatestates[0].index)
                else:
                    sourcestr = "internally_generated_"+str(event_source_state.index)

            elif len(candidatestates)> 1:
                rospy.logwarn("possible incorrect node identification on drawing: "+ str(event.event_source))
                sourcestr = sourcelabelstr+ "_" + str(current_state.index)
            else:
                sourcestr = sourcelabelstr+ "_" + str(current_state.index)
        return sourcestr, lrheadtail

    def sequentially_orthogonal_step_nodes(self, nodeid, previousNode, localtransitions):
        if previousNode is not None:
            localtransition= "\t\t\t" + previousNode + " -> " + nodeid +"[tailport=e headport=w style=invis]\n"
            if localtransition not in localtransitions:
                localtransitions.append(localtransition)

        return nodeid
    
    def get_transition_style_by_tag(self, transition_node_tag):
        if transition_node_tag in self.transition_color_by_tag:
            return {
                        "node_color": self.transition_color_by_tag[transition_node_tag],
                        "edge_color":  self.edge_color_by_tag[transition_node_tag]
                    }
        else:
            return {
                "node_color":"#DDDDDD",
                "edge_color":"#222222"
            }

    def state_to_dotlanguage(self, dotstr, cluster_count, st, transitionlist, allstates):
        rospy.loginfo("[SmaccViewer] plotting state: "+ str(st.name))
        logic_units_full_names = list()
        #childrenstate = [stx for stx in allstates if stx.name in st.children_states]
        childrenstate = [[stx for stx in allstates if stx.name == childname][0] for childname in st.children_states]
                
        stateid = "cluster_%d"%(st.index)
        dotstr+= "\nsubgraph "+stateid+"\n"
        dotstr+= "{\n"

        dotstr+="\tlabel = \"%s\";\n"%(st.name)
        
        #dotstr+="\tlabel = \"%s\";\n"%(stateid)

        dotstr+= "\tstyle=rounded;\n"
        dotstr+="\tcolor=black;\n"
        dotstr+= "\tfontsize = 35;\n"
        #dotstr+= "\tItem_%d;\n"%st.index

        draw_helpers = True
        
        contains_internal_events = [t for t in st.transitions if t.event.event_source==st.name]
        if len(contains_internal_events)> 0:
            dotstr+="\t\tinternally_generated_%d[style=\"filled,rounded\" shape=box color=\"#88aaff\" label=\"Internal\nevents\"];\n"%st.index
        
        entrynodeid = "entry_"+str(st.index)
        dotstr+="\t"+entrynodeid+"[label=\"\" color=black shape=circle bgcolor=black style=filled width=0.05]"

        dotstr+="\n"
        orthogonal_count = 0

        orthogonalid = "cluster_orthogonals_block_"+str(st.index);
        dotstr+= "\tsubgraph "+orthogonalid+"\n"
        dotstr+="\t{\n"      
        dotstr+= "\t\tlabel = \"\";\n"
        dotstr+= "\t\tcolor = \"#bbbbbb\";\n"
        
        exit_orthogonal_node = "exit_orthogonal_node"+str(st.index)
        
        # hide orthogonals on superstates
        #hidden_orthogonal = False
        hidden_orthogonal = any(childrenstate)
        if hidden_orthogonal:
            dotstr+= "\tstyle = \"invis\";\n"
        
        orthcount = len(st.orthogonals)

        localorthogonals_to_endblock=list()
        previousNode = None


        if not hidden_orthogonal:
            maxorthogonalitems =0 
            for j, orthogonal in enumerate(st.orthogonals):
                clientcount= len(orthogonal.client_names)
                substatebehaviorscount= len(orthogonal.substate_behavior_names)
                totalitems = clientcount + substatebehaviorscount
                if totalitems > maxorthogonalitems:
                    maxorthogonalitems = totalitems

            for j, orthogonal in enumerate(st.orthogonals):
                orthogonalidstr = "cluster_Orthogonal_%d_%d"%(st.index, orthogonal_count)
                dotstr+= "\t\tsubgraph " + orthogonalidstr + "\n"
                dotstr+="\t\t{\n"
                dotstr+= "\t\t\tfontsize=18;\n"
                #"node", "clust" , "graph" , "array(_flags)?(%d)?"
                #dotstr+= "\t\t\tpackmode=\"node\";\n"  
                dotstr+= "\t\t\trank=\"same\";\n"  


                orthogonal_count+=1

                orthogonal_label = (orthogonal.name.split("::")[-1])
                dotstr+= "\t\t\tlabel = \""+ orthogonal_label+"\";\n"
                dotstr+= "\t\t\tcolor = \"black\";\n"
                previousNode = None
                localtransitions = list()
                

                for i, client in enumerate(orthogonal.client_names):
                    labelstr = client.split("::")[-1]
                    idclientstr = labelstr+ "_" + str(st.index)

                    if hidden_orthogonal:
                        clientstyle = "style=invis"
                    else:
                        clientstyle = "style=filled"
                    
                    hidden_orthogonal
                    dotstr+="\t\t\t\"%s\"[%s shape=box label=%s];\n"%(idclientstr,clientstyle, labelstr)
                    previousNode = self.sequentially_orthogonal_step_nodes(idclientstr, previousNode, localtransitions)

                    #if j == 0 and i==0:
                    if i==0:
                        transitionlist.append("\""+entrynodeid +"\" -> \"" + idclientstr +"\"[color=\"#cccccc\"]")
                    
                for i, sb in enumerate(orthogonal.substate_behavior_names):
                    labelstr = sb.split("::")[-1]
                    idsbtr = labelstr+ "_" + str(st.index)
                    dotstr+="\t\t\t\"%s\"[style=filled shape=ellipse label=%s];\n"%(idsbtr,labelstr)
                    previousNode= self.sequentially_orthogonal_step_nodes(idsbtr, previousNode, localtransitions)

                # INVISIBLE ITEMS
                clientcount= len(orthogonal.client_names)
                substatebehaviorscount= len(orthogonal.substate_behavior_names)
                totalitems = clientcount + substatebehaviorscount

                for k in xrange(maxorthogonalitems - totalitems ):
                    idsbtr = "helpernode_" + str(st.index) +"_"+ str(j) + "_" + str(k)
                    stylestr="style=invis"
                    if draw_helpers:
                        stylestr=""

                    dotstr+="\t\t\t\"%s\"[style=filled shape=ellipse label=SbNodes1 %s];\n"%(idsbtr,stylestr)
                    previousNode= self.sequentially_orthogonal_step_nodes(idsbtr, previousNode, localtransitions)

                if previousNode is not None:
                    localorthogonals_to_endblock.append(previousNode+"->\""+exit_orthogonal_node +"\"")

                for lt in localtransitions:
                    dotstr+=lt
                                    
                dotstr+="\t\t}\n"

        # EXIT NODE OF ORTHOGONAL BLOCK
        dotstr+= "\t\t"+exit_orthogonal_node + "[label=\"\" color=\"#dddddd\" shape=circle bgcolor=\"#dddddd\" style=filled width=0.05];\n"

        # ONE ARROW FROM ORTHOGONAL CONTENTS TO NEXT BLOCK
        #if any(localorthogonals_to_endblock):
        #    dotstr+="\t\t"+ localorthogonals_to_endblock[-1]+"[color=\"#cccccc\"]\n"

        # MANY ARROWS FROM ORTHOGONAL CONTENTS TO NEXT BLOCK
        for t in localorthogonals_to_endblock:
            dotstr+="\t\t"+t+"[color=\"#cccccc\"]\n"

        dotstr+="\t}\n"


        logicunit_node_id = "logicunit_block_id"+str(st.index)
        logicunits_block_id = "cluster_"+ logicunit_node_id

        dotstr+= "\n\tsubgraph "+logicunits_block_id+"\n"
        dotstr+="\t{\n"      
        dotstr+= "\t\tlabel = \"\";\n"
        dotstr+= "\t\tcolor = \"#bbbbbb\";\n"
        
        dotstr += "\t\t"+logicunit_node_id+"[label=\"\" width=0.1 color=\"#dddddd\" shape=circle style=fill]\n"

        # LOGIC UNITS
        for i, lu in enumerate(st.logic_units): 
            logic_units_full_names.append(lu.type_name)
            labelstr = lu.type_name.split("::")[-1] 
            idlu = self.get_lu_id(st, lu) 
            dotstr+="\t\t\t\"%s\"[style=filled shape=box label=%s ];\n"%(idlu,labelstr)    

            for i,evsource in enumerate(lu.event_sources):
                sourcestr, statelrheadtail = self.node_id_from_event_source(evsource, st, logic_units_full_names, allstates )

                transitionstr="\t\t\""+sourcestr + "\"-> \"" + idlu + "\"[tailport=e headport=w xlabel=\"%s\" fontsize=\"%s\" penwidth=2]\n"%(evsource.event_type.split("::")[-1]+"\n"+ evsource.label, self.transition_label_font_size)
                transitionlist.append(transitionstr)
        dotstr+="\t}\n"

        # TRANSITIONS
        transition_block_id = "cluster_transition_block_"+str(st.index);
        dotstr+= "\tsubgraph "+transition_block_id+"\n"
        dotstr+="\t{\n"      
        dotstr+= "\tlabel = \"\";\n"
        dotstr+= "\tcolor = \"#bbbbbb\";\n"
        lasttransitionid = None
        for i, transition in enumerate(st.transitions):

            # TRANSITION NODE
            labelstr = transition.event.event_type.split("::")[-1] + "_" + transition.destiny_state_name
            
            idev = labelstr+ "_" + str(st.index)
            transition_node_tag = transition.transition_tag.split("::")[-1]
            transition_style = self.get_transition_style_by_tag(transition_node_tag)
            
            noconstraintattribute = "constraint=false"
            if transition_node_tag in self.no_constraint_edge_by_tag.keys():
                if self.no_constraint_edge_by_tag[transition_node_tag]:
                    noconstraintattribute = "constraint=false"
                else:
                    noconstraintattribute = ""
            
                

            dotstr+="\t\t\"%s\"[style=\"filled,rounded\" shape=box label=%s color=black fillcolor=\"%s\"];\n"%(idev, transition_node_tag, transition_style["node_color"])
            
            lasttransitionid = idev
            #ordering helper - edge
            if i == 0:
                transitionlist.append("\""+logicunit_node_id+ "\" -> \""+ idev +"\"[color=\"#dddddd\" penwidth=2]\n")
            
            # TRANSITION OUTPUT ARROW
            stdst = [stx for stx in self.state_machine_msg.states if stx.name== transition.destiny_state_name][0]
            target_stateid = "cluster_%d"%(stdst.index)
            source_stateid = "cluster_%d"%(st.index)
            transitionstr="\t\t\""+idev + "\"-> entry_"+str(stdst.index)+"[tailport=e headport=w color=\"%s\" penwidth=2 %s];\n"%(transition_style["edge_color"],noconstraintattribute)
            #transitionstr="\t\t\""+idev + "\"-> "+source_stateid+"[tailport=e headport=w color=blue];\n"
            transitionlist.append(transitionstr)
            #dotstr+=transitionstr

            # TRANSITION INPUT ARROW
            sourcestr, statelrheadtail = self.node_id_from_event_source(transition.event, st, logic_units_full_names, allstates )
            if statelrheadtail!="":
                statelrheadtail = "ltail=" + statelrheadtail
                #statelrheadtail=""
            else:
                statelrheadtail=""

            transitionstr="\t\t\""+sourcestr + "\"-> \"" + idev + "\"[tailport=e headport=w "+statelrheadtail+" xlabel=\"%s\" fontsize=\"%s\" color=\"%s\" penwidth=2]\n"%(transition.event.event_type.split("::")[-1]+"\n"+ transition.event.label, self.transition_label_font_size, transition_style["edge_color"])
            transitionlist.append(transitionstr)   
        dotstr+="\t}\n"

        #transitionlist.append("\t" + orthogonalid +"->"+ logicunits_block_id +"\n")
        #transitionlist.append("\t" + logicunits_block_id +"->"+ transition_block_id +"\n")
        #trans="\t" + logicunits_block_id.replace("cluster_","") +"->"+ transition_block_id.replace("cluster_","") +"\n"
        #transitionlist.append(trans)

        

        children_block_id = "children_block_id"+str(st.index)
        children_block_cluster_id = "cluster_"+ children_block_id
        dotstr+= "\tsubgraph "+children_block_cluster_id+"\n"
        dotstr+="{\n"    
        dotstr+= "\tlabel = \"\";\n"
        dotstr+= "\tcolor = \"#bbbbbb\";\n"

        if draw_helpers:
            submachine_helper_nodes_stylestr = "style=fill"
        else:
            submachine_helper_nodes_stylestr = "style=invis"

        #entry block of the substate machine
        dotstr += children_block_id+"[label=\"ENTRY\" width=0.1 color=\"#dddddd\" shape=circle %s]\n"%submachine_helper_nodes_stylestr

        
        children_block_end_id = "children_end_block_id"+str(st.index)
        childlasttransitionid = None
        initialsubchildentrynodeid =None
        childrentransitions = list()
        for sti in childrenstate:
            substatestr, cluster_count,subchildentrynodeid,childlasttransitionid = self.state_to_dotlanguage("", cluster_count, sti, transitionlist, allstates)
            
            if initialsubchildentrynodeid is None:
                initialsubchildentrynodeid = subchildentrynodeid
            #print("--------------------------------------")
            
            #transitionstr="\""+children_block_id + "\"->\"" + subchildentrynodeid +"\"[color=\"#dddddd\"]\n"
            #childrentransitions.append(transitionstr)

            # substate routine exit to substatemachine exit
            transitionstr="\""+childlasttransitionid + "\" -> \"" + children_block_end_id +"\"[color=\"#dddddd\"]"
            childrentransitions.append(transitionstr)
            dotstr+=substatestr

            lastsubchildentrynodeid,lastchildlasttransitionid = subchildentrynodeid,childlasttransitionid

            #transitionstr="\""+lastchildlasttransitionid + "\" -> \"" + subchildentrynodeid +"\"[color=\"#dddddd\"]"
            #childrentransitions.append(transitionstr)



        
        #SETTING END BLOCK OF THE SUBSTATE MACHINE
        dotstr += children_block_end_id+"[label=\"EXIT\" width=0.1 color=\"#dddddd\" shape=circle %s]\n"%submachine_helper_nodes_stylestr

        # ORTHOGONALS -> SUBSTATEMACHINE
        transitionstr="\""+exit_orthogonal_node + "\" -> \"" + children_block_id +"\"[color=\"#dddddd\"]"
        dotstr += transitionstr+"\n"            

        # SUBSTATEMACHINE -> LOGIC UNITS
        transitionstr="\""+children_block_end_id + "\" -> \"" + logicunit_node_id +"\"[color=\"#dddddd\"]"
        dotstr += transitionstr+"\n"            

        #transitionstr="\""+children_block_end_id + "\" -> \"" + logicunit_node_id +"\"[color=\"#dddddd\"]"
        #dotstr += transitionstr+"\n"            

        
        if initialsubchildentrynodeid is not None:
            transitionstr="\""+children_block_id + "\" -> \"" + initialsubchildentrynodeid +"\"[color=\"#dddddd\"]"
            dotstr += transitionstr+"\n"

        """
            transitionstr="\""+exit_orthogonal_node + "\" -> \"" + initialsubchildentrynodeid +"\"[color=\"#dddddd\"]"
            dotstr += transitionstr+"\n"

            transitionstr="\""+exit_orthogonal_node + "\" -> \"" + children_block_id +"\"[color=\"#dddddd\"]"
            dotstr += transitionstr+"\n"
        """
    
        #for chtrans in childrentransitions:
        #    dotstr += chtrans+"\n"

        if len(childrentransitions):
            dotstr += childrentransitions[-1]

        # SUBSTATEMACHINE ENTRY -> EXIT
        #transitionstr="\""+children_block_id + "\" -> \"" + children_block_end_id +"\"[color=\"#dddddd\"]"
        #dotstr += transitionstr+"\n"        

        cluster_count+=1
        dotstr+="}\n"    
            
        dotstr+="}\n"    
        #print("--------------------------------------")
        #print(dotstr)      
        return dotstr, cluster_count, entrynodeid,lasttransitionid

    
    def smacc_get_dotcode(self, selected_paths, closed_paths, depth, max_depth, containers, show_all, label_wrapper, attrs={}):
        """Generate the dotcode representing this container.
        
        @param selected_paths: The paths to nodes that are selected
        @closed paths: The paths that shouldn't be expanded
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to which we should traverse the tree
        @param containers: A dict of containers keyed by their paths
        @param show_all: True if implicit transitions should be shown
        @param label_wrapper: A text wrapper for wrapping element names
        @param attrs: A dict of dotcode attributes for this cluster
        """

        """
        digraph G {
            graph [fontsize=10 fontname="Verdana"];
            node [shape=record fontsize=10 fontname="Verdana"];

            subgraph cluster_0 {
                node [style=filled];
                "Item 1" "Item 2";
                label = "Container A";
                color=blue;
            }

            subgraph cluster_1 {
                node [style=filled];
                "Item 3" "Item 4";
                label = "Container B";
                color=blue;
            }

            subgraph cluster_2 {
                node [style=filled];
                "Item 5" "Item 6";
                label = "Container C";
                color=blue;
            }

            // Renders fine
            "Item 1" -> "Item 2";
            "Item 2" -> "Item 3";

            // Both of these create new nodes
            cluster_1 -> cluster_2;
            "Container A" -> "Container C";
        }
        """
        
        #if False: #self.state_machine_msg!=None:
        

        if self.state_machine_msg!=None:
            transitionlist = list()

            parent_mapping = {}
            for st in self.state_machine_msg.states:
                for child in st.children_states:
                    parent_mapping[child] = st

            root_states = [st for st in self.state_machine_msg.states if st.name not in parent_mapping.keys()]

            dotstr=""
            
            countsclusters=0
            for st in root_states:
                substatestr, countsclusters,entrynodeid, lastnodeid = self.state_to_dotlanguage("", countsclusters, st, transitionlist, self.state_machine_msg.states)
                dotstr += substatestr
               
            rospy.loginfo("STATE STR PLOTTING COMPLETE. Now plotting transitions.")
            for transitionstr in transitionlist:
                dotstr+=transitionstr + "\n"        

            rospy.loginfo("[STATE STR PLOTTING COMPLETE]")

        else:
            #dotstr= """
            #subgraph cluster_0 {
            #    node [style=filled];
            #    "Item 1" "Item 2";
            #    label = "Container A";
            #    color=blue;
            #}
            #"""

            dotstr= """
  subgraph cluster_p {
    label = "Parent";

    subgraph cluster_c1 {
      label = "Child one";
      a;

      subgraph cluster_gc_1 {
        label = "Grand-Child one";
         b;
      }
      subgraph cluster_gc_2 {
        label = "Grand-Child two";
          c;
          d;
      }
    }

    subgraph cluster_c2 {
      label = "Child two";
      e;
    }
    }

            """

        return dotstr

        dotstr = 'subgraph "cluster_%s" {\n' % (self._path)
        if depth == 0:
            #attrs['style'] = 'filled,rounded'
            attrs['color'] = '#00000000'
            attrs['fillcolor'] = '#0000000F'
        #attrs['rank'] = 'max'

        #,'succeeded','aborted','preempted'attrs['label'] = self._label
        dotstr += graph_attr_string(attrs)

        # Add start/terimate target
        proxy_attrs = {
                'URL':self._path,
                'shape':'plaintext',
                'color':'gray',
                'fontsize':'18',
                'fontweight':'18',
                'rank':'min',
                'height':'0.01'}
        proxy_attrs['label'] = '\\n'.join(label_wrapper.wrap(self._label))
        dotstr += '"%s" %s;\n' % (
                '/'.join([self._path,'__proxy__']),
                attr_string(proxy_attrs))

        # Check if we should expand this container
        if max_depth == -1 or depth <= max_depth:
            # Add container outcomes
            dotstr += 'subgraph "cluster_%s" {\n' % '/'.join([self._path,'__outcomes__'])
            outcomes_attrs = {
                    'style':'rounded,filled',
                    'rank':'sink',
                    'color':'#FFFFFFFF',#'#871C34',
                    'fillcolor':'#FFFFFF00'#'#FE464f3F'#'#DB889A'
                    }
            dotstr += graph_attr_string(outcomes_attrs)

            for outcome_label in self._container_outcomes:
                outcome_path = ':'.join([self._path,outcome_label])
                outcome_attrs = {
                        'shape':'box',
                        'height':'0.3',
                        'style':'filled,rounded',
                        'fontsize':'12',
                        'fillcolor':'#FE464f',#'#EDC2CC',
                        'color':'#780006',#'#EBAEBB',
                        'fontcolor':'#780006',#'#EBAEBB',
                        'label':'\\n'.join(label_wrapper.wrap(outcome_label)),
                        'URL':':'.join([self._path,outcome_label])
                        }
                dotstr += '"%s" %s;\n' % (outcome_path,attr_string(outcome_attrs))
            dotstr += "}\n"

            # Iterate over children
            for child_label in self._children:
                child_attrs = {
                        'style':'filled,setlinewidth(2)',
                        'color':'#000000FF',
                        'fillcolor':'#FFFFFF00'
                        }

                child_path = '/'.join([self._path,child_label])
                # Generate dotcode for children
                if child_path in containers:
                    child_attrs['style'] += ',rounded'

                    dotstr += containers[child_path].get_dotcode(
                            selected_paths,
                            closed_paths,
                            depth+1, max_depth,
                            containers,
                            show_all,
                            label_wrapper,
                            child_attrs)
                else:
                    child_attrs['label'] = '\\n'.join(label_wrapper.wrap(child_label))
                    child_attrs['URL'] = child_path
                    dotstr += '"%s" %s;\n' % (child_path, attr_string(child_attrs))

            # Iterate over edges
            internal_edges = zip(
                    self._internal_outcomes,
                    self._outcomes_from,
                    self._outcomes_to)

            # Add edge from container label to initial state
            internal_edges += [('','__proxy__',initial_child) for initial_child in self._initial_states]

            has_explicit_transitions = []
            for (outcome_label,from_label,to_label) in internal_edges:
                if to_label != 'None' or outcome_label == to_label:
                    has_explicit_transitions.append(from_label)

            # Draw internal edges
            for (outcome_label,from_label,to_label) in internal_edges:

                from_path = '/'.join([self._path, from_label])

                if show_all \
                        or to_label != 'None'\
                        or from_label not in has_explicit_transitions \
                        or (outcome_label == from_label) \
                        or from_path in containers:
                    # Set the implicit target of this outcome
                    if to_label == 'None':
                        to_label = outcome_label

                    to_path = '/'.join([self._path, to_label])

                    edge_attrs = {
                            'URL':':'.join([from_path,outcome_label,to_path]),
                            'fontsize':'12',
                            'label':'\\n'.join(label_wrapper.wrap(outcome_label))}
                    edge_attrs['style'] = 'setlinewidth(2)'

                    # Hide implicit
                    #if not show_all and to_label == outcome_label:
                    #    edge_attrs['style'] += ',invis'

                    from_key = '"%s"' % from_path
                    if from_path in containers:
                        if max_depth == -1 or depth+1 <= max_depth:
                            from_key = '"%s:%s"' % ( from_path, outcome_label)
                        else:
                            edge_attrs['ltail'] = 'cluster_'+from_path
                            from_path = '/'.join([from_path,'__proxy__'])
                            from_key = '"%s"' % ( from_path )

                    to_key = ''
                    if to_label in self._container_outcomes:
                        to_key = '"%s:%s"' % (self._path,to_label)
                        edge_attrs['color'] = '#00000055'# '#780006'
                    else:
                        if to_path in containers:
                            edge_attrs['lhead'] = 'cluster_'+to_path
                            to_path = '/'.join([to_path,'__proxy__'])
                        to_key = '"%s"' % to_path

                    dotstr += '%s -> %s %s;\n' % (
                            from_key, to_key, attr_string(edge_attrs))

        dotstr += '}\n'
        return dotstr

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self.widget.set_dotcode(dotcode, None):
            self.SetTitle('smacc Viewer')
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self.widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            self._needs_refresh = True
            wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())

    def _update_tree(self):
        """Update the tree view."""
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_cond.wait()
                self.tree.DeleteAllItems()
                self._tree_nodes = {}
                for path,tc in self._top_containers.iteritems():
                    pathfixed= path
                    self.add_to_tree(pathfixed, None)

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        if parent is None:
            container = self.tree.AddRoot(get_label(path))
        else:
            container = self.tree.AppendItem(parent,get_label(path))

        # Add children to tree
        for label in self._containers[path]._children:
            child_path = '/'.join([path,label])
            if child_path in self._containers.keys():
                self.add_to_tree(child_path, container)
            else:
                self.tree.AppendItem(container,label)

    def append_tree(self, container, parent = None):
        """Append an item to the tree view."""
        if not parent:
            node = self.tree.AddRoot(container._label)
            for child_label in container._children:
                self.tree.AppendItem(node,child_label)

    def OnIdle(self, event):
        """Event: On Idle, refresh the display if necessary, then un-set the flag."""
        if self._needs_refresh:
            self.Refresh()
            # Re-populate path combo
            self._needs_refresh = False

    def _update_server_list(self):
        """Update the list of known smacc introspection servers."""
        while self._keep_running:
            # Update the server list
            server_names = self._client.get_servers()
            rospy.loginfo(server_names)
            new_server_names = [sn for sn in server_names if sn not in self._status_subs]

            # Create subscribers for new servers
            for server_name in new_server_names:
                self._structure_subs[server_name] = rospy.Subscriber(
                        server_name+ STRUCTURE_TOPIC,
                        SmaccContainerStructure,
                        callback = self._structure_msg_update,
                        callback_args = server_name,
                        queue_size=50)

                self._status_subs[server_name] = rospy.Subscriber(
                        server_name+ STATUS_TOPIC,
                        SmaccContainerStatus,
                        callback = self._status_msg_update,
                        queue_size=50)

                # "/"+ stateMachineName + "/smacc/state_machine_description"
                self._state_machine_subs[server_name] = rospy.Subscriber(
                        server_name+ STATE_MACHINE_DESC_TOPIC,
                        SmaccStateMachine,
                        callback = self._state_machine_update,
                        queue_size=50)

            # This doesn't need to happen very often
            rospy.sleep(1.0)
            
            
            #self.server_combo.AppendItems([s for s in self._servers if s not in current_servers])

            # Grab the first server
            #current_value = self.server_combo.GetValue()
            #if current_value == '' and len(self._servers) > 0:
            #    self.server_combo.SetStringSelection(self._servers[0])
            #    self.set_server(self._servers[0])

    def ShowControlsDialog(self,event):
        dial = wx.MessageDialog(None,
                "Pan: Arrow Keys\nZoom: PageUp / PageDown\nZoom To Fit: F\nRefresh: R",
                'Keyboard Controls', wx.OK)
        dial.ShowModal()

    def SaveDotGraph(self,event):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        directory = rospkg.get_ros_home()+'/dotfiles/'
        if not os.path.exists(directory):
                os.makedirs(directory)
        filename = directory+timestr+'.dot'
        print('Writing to file: %s' % filename)
        with open(filename, 'w') as f:
            f.write(self.dotstr)

    def OnExit(self, event):
        pass

    def set_filter(self, filter):
        self.widget.set_filter(filter)

def main():
    from argparse import ArgumentParser
    p = ArgumentParser()
    p.add_argument('-f', '--auto-focus',
                 action='store_true',
                 help="Enable 'AutoFocus to subgraph' as default",
                 dest='enable_auto_focus')
    args = p.parse_args()
    app = wx.App()

    frame = SmaccViewerFrame()
    frame.set_filter('dot')

    frame.Show()

    if args.enable_auto_focus:
        frame.toggle_auto_focus(None)

    app.MainLoop()

if __name__ == '__main__':
    rospy.init_node('smacc_viewer',anonymous=False, disable_signals=True,log_level=rospy.INFO)
    sys.argv = rospy.myargv()
    main()
