#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import requests
def get_force_signal():
    url = "http://ec2-35-94-176-147.us-west-2.compute.amazonaws.com:5050/get-data"
    response = requests.get(url)
    response.raise_for_status()  # Raise an exception for HTTP errors
    data = response.json()
    print(data)
    return data.get("params_dict", {}).get("force_signal", None)
def stop_robot():
    rospy.init_node('stop_robot', anonymous=True)
    # subscribe to /scan
    # publish to /cmd_vel
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    stop_twist = Twist()

    # Set all velocities to 0
    stop_twist.linear.x = 0.0
    stop_twist.linear.y = 0.0
    stop_twist.linear.z = 0.0
    stop_twist.angular.x = 0.0
    stop_twist.angular.y = 0.0
    stop_twist.angular.z = 0.0

    while True:
        force_signal = get_force_signal()
        if force_signal != "LONG_PRESS":
            break
        twist_pub.publish(stop_twist)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        stop_robot()
    except rospy.ROSInterruptException:
        pass

