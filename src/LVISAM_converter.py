#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import Imu
import tf

def modify_orientation_rpy(roll, pitch, yaw):
    """
    Modifica roll, pitch e yaw:
    - Scambia roll e pitch.
    - Cambia il segno della yaw.
    """
    new_roll = -pitch
    new_pitch = roll
    new_yaw = -yaw
    return new_roll, new_pitch, new_yaw

def process_bag(input_bag, output_bag, imu_topic):
    """
    Elabora il bag file per modificare i dati di orientamento (roll, pitch, yaw).
    """
    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                # Se il messaggio e' sul topic IMU, modificalo
                if topic == imu_topic: 
                    # and isinstance(msg, Imu):
                    
                    # Estrai il quaternione
                    quaternion = [
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w
                    ]

                    # Converte il quaternione in angoli di Eulero (roll, pitch, yaw)
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
                    
                    # Modifica roll, pitch e yaw
                    new_roll, new_pitch, new_yaw = modify_orientation_rpy(roll, pitch, yaw)

                    # Converte i nuovi angoli di Eulero in quaternione
                    new_quaternion = tf.transformations.quaternion_from_euler(new_roll, new_pitch, new_yaw)
                    
                    # Aggiorna il messaggio IMU con i nuovi valori
                    msg.orientation.x = new_quaternion[0]
                    msg.orientation.y = new_quaternion[1]
                    msg.orientation.z = new_quaternion[2]
                    msg.orientation.w = new_quaternion[3]
                    # msg.angular_velocity.x = angular_x
                    # msg.angular_velocity.y = angular_y
                    # msg.angular_velocity.z = angular_z
                    # msg.linear_acceleration.x = acceleration_x
                    # msg.linear_acceleration.y = acceleration_y
                    # msg.linear_acceleration.z = acceleration_z
                
                # Scrivi il messaggio (modificato o no) nel nuovo bag
                outbag.write(topic, msg, t)

if __name__ == "__main__":
    # Parametri configurabili
    input_bag_file = "../bag/handheld.bag"       # Percorso al bag file di input
    output_bag_file = "modified_handheld.bag"     # Percorso al bag file di output
    imu_topic = "/imu_raw"             # Topic IMU da modificare
    
    print("Inizio elaborazione del bag file...")
    process_bag(input_bag_file, output_bag_file, imu_topic)
    print("Fine")
