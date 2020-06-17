import rospy
import roslib
roslib.load_manifest('state_msgs')
from state_msgs.msg import WholeBodyController
from state_msgs import whole_body_interface as wb_iface
import copy


class WholeBodyControllerPublisher():
    def __init__(self, topic, model):
        # Initializing the publisher
        self.pub = rospy.Publisher(topic, WholeBodyController, queue_size=1)
        self.wb_iface = wb_iface.WholeBodyStateInterface(model)

    def publish(self, t, q, q_des, v, v_des, tau, tau_des, f=None, f_des=None, s=None, s_des=None):
        msg = WholeBodyController()
        msg.header.stamp = rospy.Time(t)
        msg.actual = copy.deepcopy(self.wb_iface.writeToMessage(t, q, v, tau, f, s))
        msg.desired = copy.deepcopy(self.wb_iface.writeToMessage(t, q_des, v_des, tau_des, f_des, s))
        self.pub.publish(msg)