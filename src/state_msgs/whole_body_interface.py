import rospy
import roslib
roslib.load_manifest('state_msgs')
from state_msgs.msg import WholeBodyState, ContactState, JointState
import pinocchio


class WholeBodyStateInterface():
    def __init__(self, model):
        self.model = model
        self.data = model.createData()
        self.msg = WholeBodyState()
        self.msg.header.frame_id = "world"
        njoints = self.model.njoints - 2
        for j in range(njoints):
            name = self.model.names[j + 2]
            joint_msg = JointState()
            joint_msg.name = name
            self.msg.joints.append(joint_msg)

    def writeToMessage(self, t, q, v, tau, f=None):
        # Filling the time information
        self.msg.header.stamp = rospy.Time(t)
        self.msg.time = t

        # Filling the centroidal state
        pinocchio.centerOfMass(self.model, self.data, q, v)
        c = self.data.com[0]
        cd = self.data.vcom[0]
        # Center of mass
        self.msg.centroidal.com_position.x = c[0]
        self.msg.centroidal.com_position.y = c[1]
        self.msg.centroidal.com_position.z = c[2]
        self.msg.centroidal.com_velocity.x = cd[0]
        self.msg.centroidal.com_velocity.y = cd[1]
        self.msg.centroidal.com_velocity.z = cd[2]
        # Base
        self.msg.centroidal.base_orientation.x = q[3]
        self.msg.centroidal.base_orientation.y = q[4]
        self.msg.centroidal.base_orientation.z = q[5]
        self.msg.centroidal.base_orientation.w = q[6]
        self.msg.centroidal.base_angular_velocity.x = v[3]
        self.msg.centroidal.base_angular_velocity.y = v[4]
        self.msg.centroidal.base_angular_velocity.z = v[5]
        # Momenta
        momenta = pinocchio.computeCentroidalMomentum(self.model, self.data)
        momenta_rate = pinocchio.computeCentroidalMomentumTimeVariation(self.model, self.data)
        self.msg.centroidal.momenta.linear.x = momenta.linear[0]
        self.msg.centroidal.momenta.linear.y = momenta.linear[1]
        self.msg.centroidal.momenta.linear.z = momenta.linear[2]
        self.msg.centroidal.momenta.angular.x = momenta.angular[0]
        self.msg.centroidal.momenta.angular.y = momenta.angular[1]
        self.msg.centroidal.momenta.angular.z = momenta.angular[2]
        self.msg.centroidal.momenta_rate.linear.x = momenta_rate.linear[0]
        self.msg.centroidal.momenta_rate.linear.y = momenta_rate.linear[1]
        self.msg.centroidal.momenta_rate.linear.z = momenta_rate.linear[2]
        self.msg.centroidal.momenta_rate.angular.x = momenta_rate.angular[0]
        self.msg.centroidal.momenta_rate.angular.y = momenta_rate.angular[1]
        self.msg.centroidal.momenta_rate.angular.z = momenta_rate.angular[2]

        # Filling the joint state
        njoints = self.model.njoints - 2
        for j in range(njoints):
            self.msg.joints[j].position = q[7 + j]
            self.msg.joints[j].velocity = v[6 + j]
            self.msg.joints[j].effort = tau[j]

        # Filling the contact state
        pinocchio.forwardKinematics(self.model, self.data, q, v)
        if f is not None:
            self.msg.contacts = [None] * len(f.keys())
            for i, name in enumerate(f):
                contact_msg = ContactState()
                contact_msg.name = name

                frame_id = self.model.getFrameId(name)
                joint_id = self.model.frames[frame_id].parent
                oMf = pinocchio.updateFramePlacement(self.model, self.data, frame_id)
                pose = pinocchio.SE3ToXYZQUAT(oMf)
                ovf = pinocchio.getFrameVelocity(self.model, self.data, frame_id,
                                                 pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)

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

                contact_info = f[name]
                if contact_info[0] is not None:
                    contact_msg.type = contact_info[0]
                if contact_info[1] is not None:
                    contact_msg.wrench.force.x = contact_info[1].linear[0]
                    contact_msg.wrench.force.y = contact_info[1].linear[1]
                    contact_msg.wrench.force.z = contact_info[1].linear[2]
                    contact_msg.wrench.torque.x = contact_info[1].angular[0]
                    contact_msg.wrench.torque.y = contact_info[1].angular[1]
                    contact_msg.wrench.torque.z = contact_info[1].angular[2]

                if contact_info[2] is not None:
                    contact_msg.surface_normal.x = contact_info[2][0]
                    contact_msg.surface_normal.y = contact_info[2][1]
                    contact_msg.surface_normal.z = contact_info[2][2]
                if contact_info[3] is not None:
                    contact_msg.friction_coefficient = contact_info[3]

                self.msg.contacts[i] = contact_msg
        return self.msg
