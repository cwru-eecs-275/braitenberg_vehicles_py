import rospy
from cmd_vel_eq import cmd_vel_eq

from stdr_msgs.msg import CO2SensorMeasurementMsg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


global g_co2_sensor_0_ppm, g_co2_sensor_1_ppm
g_co2_sensor_0_ppm = 0.0
g_co2_sensor_1_ppm = 0.0

def updateCO2Sensor0(msg):
    global g_co2_sensor_0_ppm
    g_co2_sensor_0_ppm = msg.co2_ppm

def updateCO2Sensor1(msg):
    global g_co2_sensor_1_ppm
    g_co2_sensor_1_ppm = msg.co2_ppm

def updateLocalOdometryInformation(odom_info):
    pass

def main():
    global g_co2_sensor_0_ppm, g_co2_sensor_1_ppm

    base_cmd = Twist()

    # Name this node
    nh = rospy.init_node('robot_controller_py')

    # Set a rate for the loop
    naptime = rospy.Rate(10) # 10hz

    co2_sensor_0 = rospy.Subscriber("co2_sensor_0", CO2SensorMeasurementMsg, updateCO2Sensor0)
    co2_sensor_1 = rospy.Subscriber("co2_sensor_1", CO2SensorMeasurementMsg, updateCO2Sensor1)
    robot_geometry = rospy.Subscriber("odom", Odometry, updateLocalOdometryInformation)

    # Publish to the velocity topic
    cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    while not rospy.is_shutdown():
        vel, ang_vel = cmd_vel_eq(g_co2_sensor_0_ppm, g_co2_sensor_1_ppm)

        base_cmd.linear.x = vel
        base_cmd.angular.z = ang_vel

        cmd_vel_pub_.publish(base_cmd)
        naptime.sleep()
