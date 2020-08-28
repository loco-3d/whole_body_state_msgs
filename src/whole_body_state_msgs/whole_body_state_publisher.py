from __future__ import print_function, absolute_import

import rospy
from whole_body_state_msgs.msg import WholeBodyState
from .whole_body_interface import WholeBodyStateInterface

__all__ = ['WholeBodyStatePublisher']


class WholeBodyStatePublisher():
    def __init__(self, topic, model, frame_id="world", queue_size=10):
        # Initializing the publisher
        self._pub = rospy.Publisher(topic, WholeBodyState, queue_size=queue_size)
        self._wb_iface = WholeBodyStateInterface(model, frame_id)

    def publish(self, t, q, v, tau, p=dict(), pd=dict(), f=dict(), s=dict()):
        msg = self._wb_iface.writeToMessage(t, q, v, tau, p, pd, f, s)
        self._pub.publish(msg)
