"""
Copyright (c) 2017, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Please contact the author(s) of this library if you have any questions.
Authors: Vicenc Rubies Royo     ( vrubies@eecs.berkeley.edu )
         David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
         Sylvia Herbert         ( sylvia.herbert@eecs.berkeley.edu )
         Somil Bansal           ( somil@eecs.berkeley.edu )
         Jaime Fisac            ( jfisac@eecs.berkeley.edu )
"""

################################################################################
#
# Class to interface with ROS and apply the NeuralPolicy.
#
################################################################################

from neural_policy import NeuralPolicy

from crazyflie_msgs.msg import PositionStateStamped
from crazyflie_msgs.msg import NoYawControlStamped

import rospy
from std_msgs.msg import Empty

class NeuralTracker(object):
    def __init__(self):
        self._intialized = False
        self._in_flight = False

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/neural_tracker"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Create the NeuralPolicy.
        # TODO! Finish populating parameters.
        policy_params = {}
        self._policy = NeuralPolicy(self._network_file, policy_params)

        self._initialized = True
        return True

    def LoadParameters(self):
        # Get the network filename.
        if not rospy.has_param("network_file"):
            return False
        self._network_file = rospy.get_param("network_file")

        # Get the timer interval.
        if not rospy.has_param("time_step"):
            return False
        self._time_step = rospy.get_param("time_step")

        # Topics.
        if not rospy.has_param("state_topic"):
            return False
        self._state_topic = rospy.get_param("state_topic")

        if not rospy.has_param("ref_topic"):
            return False
        self._ref_topic = rospy.get_param("ref_topic")

        if not rospy.has_param("control_topic"):
            return False
        self._control_topic = rospy.get_param("control_topic")

        if not rospy.has_param("in_flight_topic"):
            return False
        self._in_flight_topic = rospy.get_param("in_flight_topic")

        return True

    def RegisterCallbacks(self):
        # Publishers.
        self._control_pub = rospy.Publisher(self._control_topic,
                                            NoYawControlStamped,
                                            queue_size=1)

        # Subscribers.
        self._state_sub = rospy.Subscriber(self._state_topic,
                                           PositionStateStamped,
                                           self.StateCallback)

        self._ref_sub = rospy.Subscriber(self._ref_topic,
                                         NoYawControlStamped,
                                         self.ReferenceCallback)

        self._in_flight_sub = rospy.Subscriber(self._in_flight_topic,
                                               Empty,
                                               self.InFlightCallback)

        # Timer.
        self._timer = rospy.Timer(rospy.Duration(self._time_step),
                                  self.TimerCallback)

        return True

    # Callback to process the takeoff signal.
    def InFlightCallback(self, msg):
        self._in_flight = True

    # Callback to process state.
    # HACK! Assuming state layout.
    def StateCallback(self, msg):
        self._state = np.array([msg.state.x, msg.state.y, msg.state.z,
                                msg.state.x_dot, msg.state.y_dot, msg.state.z_dot])

    # Callback to process references.
    # TODO! Right now this will not directly interface with the MetaPlanner
    # since that class publishes Trajectories. A simple fix will be to provide
    # a C++ node that holds Trajectories and publishes planner state on a timer.
    def ReferenceCallback(self, msg):
        self._ref = np.array([msg.state.x, msg.state.y, msg.state.z,
                              msg.state.x_dot, msg.state.y_dot, msg.state.z_dot])

    # Timer callback. Every time this fires, look at the relative state and
    # apply the optimal control.
    # NOTE! This exists only to enforce regularity. It would also work to apply
    # this function after every state or reference update.
    def TimerCallback(self, event):
        if not self._in_flight:
            return

        # Assuming we are in flight, then compute relative state and apply
        # optimal control.
        relative_state = self._state - self._ref
        optimal_control = self._policy.OptimalControl(relative_state)

        # Package into a message and publish.
        # HACK! Assuming control layout.
        msg = NoYawControlStamped()
        msg.header.stamp = rospy.Time.now()
        msg.control.roll = optimal_control[0]
        msg.control.pitch = optimal_control[1]
        msg.control.thrust = optimal_control[2]
        msg.control.priority = 1.0

        self._control_pub.publish(msg)
