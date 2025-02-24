#!/usr/bin/env python

import rospy
from mav_msgs.msg import RollPitchYawrateThrust
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped, Vector3Stamped, QuaternionStamped, PoseStamped
from geometry_msgs.msg import Pose
import tf
import numpy as np

class ConverterMAVtoDJI(object):  
    def __init__(self, upub, opub, ppub):
        self.upub = upub
        self.o_pub = opub
        self.ppub = ppub
        self.odometry = Odometry()

    def vel_callback(self,msg):
        self.odometry.twist.twist.linear = msg.vector # TODO: Verifiy matching coordinate frame
        #w_b_orientation = tf.fromTranslationRotation(Pose(), self.odometry.pose.pose.orientation)
        #w_position = np.ones((4,1))
        #w_position[0] = msg.vector.x
        #w_position[1] = msg.vector.y
        #w_position[2] = msg.vector.z
        #b_position = w_b_orientation.inverse() * w_position;
        #self.odometry.twist.twist.linear.x = b_position[0]
        #self.odometry.twist.twist.linear.y = b_position[1]
        #self.odometry.twist.twist.linear.z = b_position[2]
    
    def pos_callback(self,msg):
        self.odometry.pose.pose.position = msg.point
    
    def attitude_callback(self,msg):
        self.odometry.pose.pose.orientation = msg.quaternion
    
    def publish_odometry(self):
        self.o_pub.publish(self.odometry)
        pose = PoseStamped()
        pose.pose = self.odometry.pose.pose
        pose.header = self.odometry.header
        self.ppub.publish(pose)
        #print(self.odometry.pose.position)

    def u_callback(self, msg):

        # get roll pitch thrust and yaw_rate from mav_nonlinear_mpc
        
        # roll = msg.roll
        # pitch = msg.pitch
        # yaw_rate = msg.yaw_rate
        # thrust_x = msg.thrust.x
        # thrust_y = msg.thrust.y
        # thrust_z = msg.thrust.z

        # rospy.loginfo('roll: %f, pitch: %f yaw_rate: %f',roll,pitch,yaw_rate)
        # rospy.loginfo('thrust x: %f, thrust y: %f thrust z: %f', thrust_x, thrust_y, thrust_z)
        u = Joy()
        g = 9.82
        u.axes = (msg.roll, msg.pitch, msg.thrust.z, msg.yaw_rate, 0x28)
        self.upub.publish(u)


def main():
    rospy.init_node('MAV_ctrl_monitor') #, anonymous=True)

    upub = rospy.Publisher('/dji_sdk/flight_control_setpoint_generic', Joy, queue_size=1)
    opub = rospy.Publisher('/DjiMatrice/ground_truth/odometry', Odometry, queue_size=1)
    ppub = rospy.Publisher('/DjiMatrice/ground_truth/pose', PoseStamped, queue_size=1)

    Converter = ConverterMAVtoDJI(upub, opub, ppub)

    rospy.Subscriber('/DjiMatrice/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, Converter.u_callback)
    rospy.Subscriber('/dji_sdk/velocity', Vector3Stamped, Converter.vel_callback)
    rospy.Subscriber('/dji_sdk/local_position', PointStamped, Converter.pos_callback)
    rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, Converter.attitude_callback)     

    r = rospy.Rate(200)
    while not rospy.is_shutdown():      
        Converter.publish_odometry()
        r.sleep()
    
    #rospy.spin()


if __name__ == '__main__':
    main()



