from __future__ import print_function, absolute_import

import rospy
from whole_body_state_msgs.msg import WholeBodyState, ContactState, JointState
import pinocchio
import numpy as np
import copy

__all__ = ['WholeBodyStateInterface']


class WholeBodyStateInterface():
    def __init__(self, model, frame_id="world"):
        self._model = model
        self._data = model.createData()
        self.frame_id = frame_id
        self._msg = WholeBodyState()
        self._msg.header.frame_id = frame_id
        njoints = self._model.njoints - 2
        for j in range(njoints):
            name = self._model.names[j + 2]
            joint_msg = JointState()
            joint_msg.name = name
            self._msg.joints.append(joint_msg)

    def writeToMessage(self, t, q, v=None, tau=None, p=dict(), pd=dict(), f=dict(), s=dict()):
        # Filling the time information
        self._msg.header.stamp = rospy.Time(t)
        self._msg.time = t

        if v is None:
            v = np.zeros(self._model.nv)
        if tau is None:
            tau = np.zeros(self._model.njoints - 2)

        # Filling the centroidal state
        pinocchio.centerOfMass(self._model, self._data, q, v)
        c = self._data.com[0]
        cd = self._data.vcom[0]
        # Center of mass
        self._msg.centroidal.com_position.x = c[0]
        self._msg.centroidal.com_position.y = c[1]
        self._msg.centroidal.com_position.z = c[2]
        self._msg.centroidal.com_velocity.x = cd[0]
        self._msg.centroidal.com_velocity.y = cd[1]
        self._msg.centroidal.com_velocity.z = cd[2]
        # Base
        self._msg.centroidal.base_orientation.x = q[3]
        self._msg.centroidal.base_orientation.y = q[4]
        self._msg.centroidal.base_orientation.z = q[5]
        self._msg.centroidal.base_orientation.w = q[6]
        self._msg.centroidal.base_angular_velocity.x = v[3]
        self._msg.centroidal.base_angular_velocity.y = v[4]
        self._msg.centroidal.base_angular_velocity.z = v[5]
        # Momenta
        momenta = pinocchio.computeCentroidalMomentum(self._model, self._data)
        momenta_rate = pinocchio.computeCentroidalMomentumTimeVariation(self._model, self._data)
        self._msg.centroidal.momenta.linear.x = momenta.linear[0]
        self._msg.centroidal.momenta.linear.y = momenta.linear[1]
        self._msg.centroidal.momenta.linear.z = momenta.linear[2]
        self._msg.centroidal.momenta.angular.x = momenta.angular[0]
        self._msg.centroidal.momenta.angular.y = momenta.angular[1]
        self._msg.centroidal.momenta.angular.z = momenta.angular[2]
        self._msg.centroidal.momenta_rate.linear.x = momenta_rate.linear[0]
        self._msg.centroidal.momenta_rate.linear.y = momenta_rate.linear[1]
        self._msg.centroidal.momenta_rate.linear.z = momenta_rate.linear[2]
        self._msg.centroidal.momenta_rate.angular.x = momenta_rate.angular[0]
        self._msg.centroidal.momenta_rate.angular.y = momenta_rate.angular[1]
        self._msg.centroidal.momenta_rate.angular.z = momenta_rate.angular[2]

        # Filling the joint state
        njoints = self._model.njoints - 2
        for j in range(njoints):
            self._msg.joints[j].position = q[7 + j]
            self._msg.joints[j].velocity = v[6 + j]
            self._msg.joints[j].effort = tau[j]

        # Filling the contact state
        names = p.keys() + pd.keys() + f.keys() + s.keys()
        names = list(dict.fromkeys(names))
        self._msg.contacts = [None] * len(names)
        if len(names) != 0 and (len(p.keys()) == 0 or len(pd.keys()) == 0):
            pinocchio.forwardKinematics(self._model, self._data, q, v)
        for i, name in enumerate(names):
            contact_msg = ContactState()
            contact_msg.name = name
            frame_id = self._model.getFrameId(name)
            # Retrive the contact position
            if p.has_key(name):
                pose = pinocchio.SE3ToXYZQUAT(p[name])
            else:
                oMf = pinocchio.updateFramePlacement(self._model, self._data, frame_id)
                pose = pinocchio.SE3ToXYZQUAT(oMf)
            # Retrieve the contact velocity
            if pd.has_key(name):
                ovf = pd[name]
            else:
                ovf = pinocchio.getFrameVelocity(self._model, self._data, frame_id,
                                                 pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            # Storing the contact position and velocity inside the message
            contact_msg.pose.position.x = pose[0]
            contact_msg.pose.position.y = pose[1]
            contact_msg.pose.position.z = pose[2]
            contact_msg.pose.orientation.x = pose[3]
            contact_msg.pose.orientation.y = pose[4]
            contact_msg.pose.orientation.z = pose[5]
            contact_msg.pose.orientation.w = pose[6]
            contact_msg.velocity.linear.x = ovf.linear[0]
            contact_msg.velocity.linear.y = ovf.linear[1]
            contact_msg.velocity.linear.z = ovf.linear[2]
            contact_msg.velocity.angular.x = ovf.angular[0]
            contact_msg.velocity.angular.y = ovf.angular[1]
            contact_msg.velocity.angular.z = ovf.angular[2]
            # Retrieving and storing force data
            if f.has_key(name):
                contact_info = f[name]
                ctype = contact_info[0]
                force = contact_info[1]
                contact_msg.type = ctype
                contact_msg.wrench.force.x = force.linear[0]
                contact_msg.wrench.force.y = force.linear[1]
                contact_msg.wrench.force.z = force.linear[2]
                contact_msg.wrench.torque.x = force.angular[0]
                contact_msg.wrench.torque.y = force.angular[1]
                contact_msg.wrench.torque.z = force.angular[2]
            if s.has_key(name):
                terrain_info = s[name]
                norm = terrain_info[0]
                friction = terrain_info[1]
                contact_msg.surface_normal.x = norm[0]
                contact_msg.surface_normal.y = norm[1]
                contact_msg.surface_normal.z = norm[2]
                contact_msg.friction_coefficient = friction
            self._msg.contacts[i] = contact_msg
        return copy.deepcopy(self._msg)

    def writeFromMessage(self, msg):
        t = msg.time
        q = np.zeros(self._model.nq)
        v = np.zeros(self._model.nv)
        tau = np.zeros(self._model.njoints - 2)
        p = dict()
        pd = dict()
        f = dict()
        s = dict()
        # Retrieve the generalized position and velocity, and joint torques
        q[3] = msg.centroidal.base_orientation.x
        q[4] = msg.centroidal.base_orientation.y
        q[5] = msg.centroidal.base_orientation.z
        q[6] = msg.centroidal.base_orientation.w
        v[3] = msg.centroidal.base_angular_velocity.x
        v[4] = msg.centroidal.base_angular_velocity.y
        v[5] = msg.centroidal.base_angular_velocity.z
        for j in range(len(msg.joints)):
            jointId = self._model.getJointId(msg.joints[j].name) - 2
            q[jointId + 7] = msg.joints[j].position
            v[jointId + 6] = msg.joints[j].velocity
            tau[jointId] = msg.joints[j].effort
        pinocchio.centerOfMass(self._model, self._data, q, v)
        q[0] = msg.centroidal.com_position.x - self._data.com[0][0]
        q[1] = msg.centroidal.com_position.y - self._data.com[0][1]
        q[2] = msg.centroidal.com_position.z - self._data.com[0][2]
        v[0] = msg.centroidal.com_velocity.x - self._data.vcom[0][0]
        v[1] = msg.centroidal.com_velocity.y - self._data.vcom[0][1]
        v[2] = msg.centroidal.com_velocity.z - self._data.vcom[0][2]
        # Retrive the contact information
        for contact in msg.contacts:
            name = contact.name
            # Contact pose
            pose = contact.pose
            position = np.array([pose.position.x, pose.position.y, pose.position.z])
            quaternion = pinocchio.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z)
            p[name] = pinocchio.SE3(quaternion, position)
            # Contact velocity
            velocity = contact.velocity
            lin_vel = np.array([velocity.linear.x, velocity.linear.y, velocity.linear.z])
            ang_vel = np.array([velocity.angular.x, velocity.angular.y, velocity.angular.z])
            pd[name] = pinocchio.Motion(lin_vel, ang_vel)
            # Contact wrench
            wrench = contact.wrench
            force = np.array([wrench.force.x, wrench.force.y, wrench.force.z])
            torque = np.array([wrench.torque.x, wrench.torque.y, wrench.torque.z])
            f[name] = [contact.type, pinocchio.Force(force, torque)]
            # Surface normal and friction coefficient
            normal = contact.surface_normal
            nsurf = np.array([normal.x, normal.y, normal.z])
            s[name] = [nsurf, contact.friction_coefficient]
        return t, q, v, tau, p, pd, f, s
