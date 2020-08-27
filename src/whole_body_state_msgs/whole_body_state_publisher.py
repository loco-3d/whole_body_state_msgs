from __future__ import print_function, absolute_import

import rospy
from whole_body_state_msgs.msg import WholeBodyState
from .whole_body_interface import WholeBodyStateInterface

__all__ = ['WholeBodyStatePublisher']


class WholeBodyStatePublisher():
    def __init__(self, topic, model):
        # Initializing the publisher
        self.pub = rospy.Publisher(topic, WholeBodyState, queue_size=1)
        self.wb_iface = WholeBodyStateInterface(model)

    def publish(self, t, q, v, tau, p=dict(), pd=dict(), f=dict(), s=dict()):
        msg = self.wb_iface.writeToMessage(t, q, v, tau, p, pd, f, s)
        self.pub.publish(msg)
