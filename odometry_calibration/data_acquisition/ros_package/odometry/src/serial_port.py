#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from datetime import datetime



def serial_connection():
    #init node
    rospy.init_node('read_odometry', anonymous = True)
    raw_pub = rospy.Publisher('encoder_raw_data', String, queue_size=10)

    #open serial port
    ser = serial.Serial('/dev/mySTM32', 115200)
    serial_output = ser.readline()

    #clear messages buffer
    while (ser.inWaiting()):
        serial_output = ser.readline()

    while ser.isOpen () and not rospy.is_shutdown():

        serial_output = ser.readline()
        serial_output = serial_output.decode("utf-8")
        #print(serial_output)

        # Get time
        timeStamp = "%.4f" % time.time()

        # join data
        dataOut = (str(timeStamp) + " " + serial_output + "\n")
        print(dataOut)

        # Write to txt file
        file.write(dataOut)

        file.flush()

        #Publish
        raw_pub.publish(dataOut)

             
    ser.close()
    file.close()
        
        

 



if __name__ == '__main__':
    try:
        path = '/home/lsa/LOGs/'
        date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        extension = '.txt'
        name = path + date + extension
        file = open( name, 'w')
        serial_connection()
    except rospy.ROSInterruptException:
        pass
