from __future__ import print_function, absolute_import

import rospy
from whole_body_state_msgs.msg import WholeBodyController
from .whole_body_interface import WholeBodyStateInterface
import copy

__all__ = ['WholeBodyControllerPublisher']


class WholeBodyControllerPublisher():
    def __init__(self, topic, model):
        # Initializing the publisher
        self.pub = rospy.Publisher(topic, WholeBodyController, queue_size=1)
        self.wb_iface = WholeBodyStateInterface(model)

    def publish(self,
                t,
                q,
                q_des,
                v,
                v_des,
                tau,
                tau_des,
                p=dict(),
                p_des=dict(),
                pd=dict(),
                pd_des=dict(),
                f=dict(),
                f_des=dict(),
                s=dict(),
                s_des=dict()):
        msg = WholeBodyController()
        msg.header.stamp = rospy.Time(t)
        msg.header.frame_id = "world"
        msg.actual = copy.deepcopy(self.wb_iface.writeToMessage(t, q, v, tau, p, pd, f, s))
        msg.desired = copy.deepcopy(self.wb_iface.writeToMessage(t, q_des, v_des, tau_des, p_des, pd_des, f_des,
                                                                 s_des))
        self.pub.publish(msg)
