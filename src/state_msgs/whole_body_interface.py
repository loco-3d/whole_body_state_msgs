import rospy
import roslib; roslib.load_manifest('state_msgs')
from state_msgs.msg import WholeBodyState, ContactState, JointState
import pinocchio


class WholeBodyStateInterface():
    def __init__(self, model):
        self.model = model
        self.data = model.createData()

    def writeToMessage(self, t, q, v, tau, f=None):
        msg = WholeBodyState()

        # Filling the time information
        msg.header.stamp = rospy.Time(t)
        msg.header.frame_id = self.model.frames[2].name
        msg.time = t

        # Filling the centroidal state
        pinocchio.centerOfMass(self.model, self.data, q, v)
        c = self.data.com[0]
        cd = self.data.vcom[0]
        # Center of mass
        msg.centroidal.com_position.x = c[0]
        msg.centroidal.com_position.y = c[1]
        msg.centroidal.com_position.z = c[2]
        msg.centroidal.com_velocity.x = cd[0]
        msg.centroidal.com_velocity.y = cd[1]
        msg.centroidal.com_velocity.z = cd[2]
        # Base
        msg.centroidal.base_orientation.x = q[3]
        msg.centroidal.base_orientation.y = q[4]
        msg.centroidal.base_orientation.z = q[5]
        msg.centroidal.base_orientation.w = q[6]
        msg.centroidal.base_angular_velocity.x = v[3]
        msg.centroidal.base_angular_velocity.y = v[4]
        msg.centroidal.base_angular_velocity.z = v[5]
        # Momenta
        momenta = pinocchio.computeCentroidalMomentum(self.model, self.data)
        momenta_rate = pinocchio.computeCentroidalMomentumTimeVariation(self.model, self.data)
        msg.centroidal.momenta.linear.x = momenta.linear[0]
        msg.centroidal.momenta.linear.y = momenta.linear[1]
        msg.centroidal.momenta.linear.z = momenta.linear[2]
        msg.centroidal.momenta.angular.x = momenta.angular[0]
        msg.centroidal.momenta.angular.y = momenta.angular[1]
        msg.centroidal.momenta.angular.z = momenta.angular[2]
        msg.centroidal.momenta_rate.linear.x = momenta_rate.linear[0]
        msg.centroidal.momenta_rate.linear.y = momenta_rate.linear[1]
        msg.centroidal.momenta_rate.linear.z = momenta_rate.linear[2]
        msg.centroidal.momenta_rate.angular.x = momenta_rate.angular[0]
        msg.centroidal.momenta_rate.angular.y = momenta_rate.angular[1]
        msg.centroidal.momenta_rate.angular.z = momenta_rate.angular[2]

        # Filling the joint state
        njoints = self.model.njoints - 2
        for j in range(njoints):
            name = self.model.names[j + 2]
            joint_msg = JointState()
            joint_msg.name = name
            joint_msg.position = q[7 + j]
            joint_msg.velocity = v[6 + j]
            joint_msg.effort = tau[j]
            msg.joints.append(joint_msg)

        # Filling the contact state
        pinocchio.forwardKinematics(self.model, self.data, q, v)
        if f is not None:
            for name in f:
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

                contact_msg.wrench.force.x = f[name].linear[0]
                contact_msg.wrench.force.y = f[name].linear[1]
                contact_msg.wrench.force.z = f[name].linear[2]
                contact_msg.wrench.torque.x = f[name].angular[0]
                contact_msg.wrench.torque.y = f[name].angular[1]
                contact_msg.wrench.torque.z = f[name].angular[2]

                msg.contacts.append(contact_msg)
        return msg