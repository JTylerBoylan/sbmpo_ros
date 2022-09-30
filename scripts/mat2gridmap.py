import rospy
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from  scipy.io import loadmat
import pandas as pd
import numpy as np
import rospkg

def mat2gridmap():

    # Initialize ROS node
    rospy.init_node('mat2gridmap', anonymous=True)

    # Get parameters
    pub_topic = rospy.get_param("~publish_topic")
    data_file = rospy.get_param("~data_file")
    frame_id = rospy.get_param("~frame_id")
    resolution = rospy.get_param("~resolution")
    length_x = rospy.get_param("~length_x")
    length_y = rospy.get_param("~length_y")
    position_x = rospy.get_param("~position_x")
    position_y = rospy.get_param("~position_y")

    # Get data from .mat file
    rospack = rospkg.RosPack()
    path = rospack.get_path("hybrid_astar")
    data = loadmat(path + "/" + data_file)['dataforlocalplanner']
    xdata = data[0][0]['X']
    ydata = data[0][0]['Y']
    zdata = data[0][0]['Z']
    tdata = data[0][0]['temperarure_soil']

    # Get map size
    xlen = len(xdata)
    ylen = len(ydata)

    # Format data
    zdataList = np.array(zdata).flatten('F')
    tdataList = np.array(tdata).flatten('F')

    # Create rostopic variables
    pub = rospy.Publisher(pub_topic, GridMap, queue_size=10)
    msg = GridMap()

    # Loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Message Header
        msg.info.header.stamp = rospy.Time.now()
        msg.info.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.length_x = length_x
        msg.info.length_y = length_y
        msg.info.pose.position.x = position_x
        msg.info.pose.position.y = position_y
        msg.info.pose.position.z = 0.0
        msg.info.pose.orientation.x = 0.0
        msg.info.pose.orientation.y = 0.0
        msg.info.pose.orientation.z = 0.0
        msg.info.pose.orientation.w = 1.0

        # Layers
        msg.layers = ['elevation', 'temperature']
        msg.basic_layers = ['elevation']
        msg.data.clear()

        dataArrayZ = Float32MultiArray()
        dataArrayT = Float32MultiArray()

        dim0Z = MultiArrayDimension()
        dim0Z.label = "column_index"
        dim0Z.stride = xlen*ylen
        dim0Z.size = xlen

        dim1Z = MultiArrayDimension()
        dim1Z.label = "row_index"
        dim1Z.stride = ylen
        dim1Z.size = ylen

        dataArrayZ.layout.dim = [dim0Z, dim1Z]
        dataArrayZ.data = zdataList

        dim0T = MultiArrayDimension()
        dim0T.label = "column_index"
        dim0T.stride = xlen*xlen
        dim0T.size = xlen

        dim1T = MultiArrayDimension()
        dim1T.label = "row_index"
        dim1T.stride = ylen
        dim1T.size = ylen

        dataArrayT.layout.dim = [dim0T, dim1T]
        dataArrayT.data = tdataList

        msg.data = [dataArrayZ, dataArrayT]

        msg.outer_start_index = 0
        msg.inner_start_index = 0

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        mat2gridmap()
    except rospy.ROSInterruptException:
        pass