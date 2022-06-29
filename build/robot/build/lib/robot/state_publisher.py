from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_msgs.msg import Float64MultiArray

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.static_broadcaster = StaticTransformBroadcaster(self, qos=qos_profile)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'angle_publisher',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))


        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        self.theta_T = 0.0
        self.theta_R1 = 0.0
        self.theta_R2 = 0.0
        inc_T = 0.05
        inc_R1 = 0.03
        inc_R2 = 0.02

        # message declarations
        World_trans = TransformStamped()  #Torsional Transform
        World_trans.header.frame_id = 'world'
        World_trans.child_frame_id = 'base_link'

        T_trans = TransformStamped()  #Torsional Transform
        T_trans.header.frame_id = 'base_link'
        T_trans.child_frame_id = 'Link_1'

        T_pas_trans = TransformStamped()  #Torsional Transform
        T_pas_trans.header.frame_id = 'Link_1'
        T_pas_trans.child_frame_id = 'Link_1_pas'

        R1_trans = TransformStamped()  #Rotational 1 Transform
        R1_trans.header.frame_id = 'Link_1_pas'
        R1_trans.child_frame_id = 'Link_2'

        R1_pas_trans = TransformStamped()  #Rotational 1 pasador Transform
        R1_pas_trans.header.frame_id = 'Link_2'
        R1_pas_trans.child_frame_id = 'Link_2_pas'

        R2_trans = TransformStamped()  #Rotational 2 Transform
        R2_trans.header.frame_id = 'Link_2_pas'
        R2_trans.child_frame_id = 'Link_3'

        Lidar_trans = TransformStamped()  #Rotational 2 Transform
        Lidar_trans.header.frame_id = 'Link_1'
        Lidar_trans.child_frame_id = 'Lidar_link'



        joint_state = JointState()


        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['world_joint', 'T_joint', 'T_joint_pas', 'R1_joint', 'R1_joint_pas', 'R2_joint', 'Lidar_joint']
                joint_state.position = [0.0, self.theta_T, 0.0, self.theta_R1, 0.0, self.theta_R2, self.theta_R2]

                if ((self.theta_R1 > 3.54159) or (self.theta_R1 < -0.4)):
                    inc_R1 *= -1
                if ((self.theta_R2 > 2.22) or (self.theta_R2 < -2.22)):
                    inc_R2 *= -1


                # update transform
                World_trans.header.stamp = now.to_msg() #Generate Torsional Transform Message
                World_trans.transform.translation.x = 0.0
                World_trans.transform.translation.y = 0.0
                World_trans.transform.translation.z = 0.0
                #theta_T += inc_T
                World_trans.transform.rotation = \
                    euler_to_quaternion(0.0, 0.0, 0.0) # roll,pitch,yaw

                T_trans.header.stamp = now.to_msg() #Generate Torsional Transform Message
                T_trans.transform.translation.x = 0.0
                T_trans.transform.translation.y = 0.0
                T_trans.transform.translation.z = 0.19042
                #theta_T += inc_T
                T_trans.transform.rotation = \
                    euler_to_quaternion(1.574, -1.5708, self.theta_T) # roll,pitch,yaw

                T_pas_trans.header.stamp = now.to_msg() #Generate Torsional Transform Message
                T_pas_trans.transform.translation.x = 0.242
                T_pas_trans.transform.translation.y = 0.0
                T_pas_trans.transform.translation.z = 0.0
                #theta_T += inc_T
                T_pas_trans.transform.rotation = \
                    euler_to_quaternion(-1.5708, 0.0, -1.5686) # roll,pitch,yaw

                R1_trans.header.stamp = now.to_msg() #Generate Rotational 1 Transform Message
                R1_trans.transform.translation.x = 0.0
                R1_trans.transform.translation.y = 0.0
                R1_trans.transform.translation.z = 0.0
                #theta_R1 += inc_R1
                R1_trans.transform.rotation = \
                    euler_to_quaternion(self.theta_R1, 0, 0) # roll,pitch,yaw

                R1_pas_trans.header.stamp = now.to_msg() #Generate Torsional Transform Message
                R1_pas_trans.transform.translation.x = 0.0
                R1_pas_trans.transform.translation.y = 0.0
                R1_pas_trans.transform.translation.z = 0.25
                #theta_T += inc_T
                R1_pas_trans.transform.rotation = \
                    euler_to_quaternion(0.0, -0.015943, 0.0) # roll,pitch,yaw

                R2_trans.header.stamp = now.to_msg() #Generate Rotational 2 Transform Message
                R2_trans.transform.translation.x = 0.0
                R2_trans.transform.translation.y = 0.0
                R2_trans.transform.translation.z = 0.0
                #theta_R2 += inc_R2
                R2_trans.transform.rotation = \
                    euler_to_quaternion(3.1234+self.theta_R2, 0, -1.5708) # roll,pitch,yaw

                Lidar_trans.header.stamp = now.to_msg() #Generate Rotational 2 Transform Message
                Lidar_trans.transform.translation.x = 0.0
                Lidar_trans.transform.translation.y = -0.4
                Lidar_trans.transform.translation.z = 0.0
                #theta_R2 += inc_R2
                Lidar_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, 0) # roll,pitch,yaw



                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.static_broadcaster.sendTransform(World_trans)
                self.broadcaster.sendTransform(T_trans)
                self.static_broadcaster.sendTransform(T_pas_trans)
                self.broadcaster.sendTransform(R1_trans)
                self.static_broadcaster.sendTransform(R1_pas_trans)
                self.broadcaster.sendTransform(R2_trans)
                self.broadcaster.sendTransform(Lidar_trans)

                # Create new robot state
                '''tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree/4'''

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def listener_callback(self, msg):
        self.theta_T = msg.data[0]
        self.theta_R1 = msg.data[1]
        self.theta_R2 = msg.data[2]
        #self.get_logger().info('I heard: "%s"' % msg.data)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
