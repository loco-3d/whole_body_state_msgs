import rospy
from state_msgs.msg import WholeBodyState
from state_msgs import whole_body_interface as wb_iface


class WholeBodyStatePublisher():
    def __init__(self, topic, model):
        # Initializing the publisher
        self.pub = rospy.Publisher(topic, WholeBodyState, queue_size=1)
        self.wb_iface = wb_iface.WholeBodyStateInterface(model)

    def publish(self, t, q, v, tau, p=dict(), pd=dict(), f=dict(), s=dict()):
        msg = self.wb_iface.writeToMessage(t, q, v, tau, p, pd, f, s)
        self.pub.publish(msg)