from __future__ import print_function, absolute_import

import rospy
from whole_body_state_msgs.msg import WholeBodyTrajectory
from .whole_body_interface import WholeBodyStateInterface
import copy

__all__ = ['WholeBodyTrajectoryPublisher']


class WholeBodyTrajectoryPublisher():
    def __init__(self, topic, model):
        # Defining the subscriber
        self.pub = rospy.Publisher(topic, WholeBodyTrajectory, queue_size=1)
        self.wb_iface = WholeBodyStateInterface(model)

    def publish(self, ts, qs, vs=None, us=None, ps=None, pds=None, fs=None, ss=None):
        msg = WholeBodyTrajectory()
        # Check that the length of the lists are consistent
        if len(ts) is not len(qs):
            print("Couldn't publish the message since the length of the qs list is not consistent")
            return
        if vs is not None:
            if len(ts) is not len(vs):
                print("Couldn't publish the message since the length of the vs list is not consistent")
                return
        if us is not None:
            if len(ts) is not len(us):
                print("Couldn't publish the message since the length of the us list is not consistent")
                return
        if ps is not None:
            if len(ts) is not len(ps):
                print("Couldn't publish the message since the length of the ps list is not consistent")
                return
        if pds is not None:
            if len(ts) is not len(pds):
                print("Couldn't publish the message since the length of the pds list is not consistent")
                return
        if fs is not None:
            if len(ts) is not len(fs):
                print("Couldn't publish the message since the length of the fs list is not consistent")
                return
        if ss is not None:
            if len(ts) is not len(ss):
                print("Couldn't publish the message since the length of the ss list is not consistent")
                return

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        for i in range(len(ts)):
            vi = None
            if vs is not None:
                vi = vs[i]
            ui = None
            if us is not None:
                vi = us[i]
            pi = dict()
            if ps is not None:
                pi = ps[i]
            pdi = dict()
            if pds is not None:
                pdi = pds[i]
            fi = dict()
            if fs is not None:
                fi = fs[i]
            wb_msg = copy.deepcopy(self.wb_iface.writeToMessage(ts[i], qs[i], vi, ui, pi, pdi, fi))
            msg.trajectory.append(wb_msg)
        self.pub.publish(msg)
