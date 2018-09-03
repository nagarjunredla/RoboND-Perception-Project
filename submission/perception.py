#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

class dropbox_data(object):
    def __init__(self, position, arm):
        self.pos = position
        self.arm = arm

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Define functions as required
def voxel_grid(pcl_in, leaf_size = 0.005):
    vox = pcl_in.make_voxel_grid_filter()

    # Set Voxel size
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)

    # Call the filter function to obtained downsampled Point Cloud
    pcl_out = vox.filter()

    return pcl_out

def statistical_outlier_filter(pcl_in, neighbor_pts, scale_factor):
    # Create filter object
    outlier_filter = pcl_in.make_statistical_outlier_filter()
    # Set neighboring points
    outlier_filter.set_mean_k(neighbor_pts)
    # Set threshold
    outlier_filter.set_std_dev_mul_thresh(scale_factor)
    # Return data
    return outlier_filter.filter()

def passthrough_filter(pcl_in, axis = 'x', axis_min = 0.1, axis_max = 0.9):
    # Create a PassThrough filter object
    passthrough = pcl_in.make_passthrough_filter()

    # Assign axis and range
    passthrough.set_filter_field_name(axis)
    passthrough.set_filter_limits(axis_min,axis_max)

    # Using the filter function to obtain point cloud
    pcl_out = passthrough.filter()
    
    return pcl_out

def ransac_filter(pcl_in, max_distance=0.01):
    # Create segmentation object
    seg = pcl_in.make_segmenter()

    # Select the model to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for point to be considered fitting the model
    seg.set_distance_threshold(max_distance)

    # Call the segment function
    return seg.segment()

def euclidean_clustering(white_cloud, cluster_tolerance = 0.02, min_cluster_size = 50, max_cluster_size = 25000):
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    ec.set_SearchMethod(tree)
    return ec.Extract()

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # Statistical Outlier Filtering
    cloud_filtered = statistical_outlier_filter(cloud, 5, 0.05)
    # Voxel Grid Downsampling
    cloud_filtered = voxel_grid(cloud_filtered, 0.005)

    # PassThrough Filter
    cloud_filtered = passthrough_filter(cloud_filtered, axis = 'x', axis_min = 0.1, axis_max = 0.9)
    cloud_filtered = passthrough_filter(cloud_filtered, axis = 'y', axis_min = -0.47, axis_max = 0.47)
    cloud_filtered = passthrough_filter(cloud_filtered, axis = 'z', axis_min = 0.6, axis_max = 0.9)

    # RANSAC Plane Segmentation
    inliers, coefficients = ransac_filter(cloud_filtered, max_distance = 0.01)

    # Extract inliers and outliers
    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    cluster_indices = euclidean_clustering(white_cloud, cluster_tolerance = 0.02, min_cluster_size = 50, max_cluster_size = 25000)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, idx in enumerate(indices):
            color_cluster_point_list.append([white_cloud[idx][0],
                                             white_cloud[idx][1],
                                             white_cloud[idx][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # Convert the cluster from PCL to ROS
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        color_hist = compute_color_histograms(ros_cluster, using_hsv = True)
        # Compute normals and histogram
        normals = get_normals(ros_cluster)
        normal_hist = compute_normal_histograms(normals)
        feature_vec = np.concatenate((color_hist, normal_hist))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature_vec.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.2
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    label_list = []
    test_scene_num = Int32()
    test_scene_num.data = 3
    object_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    arm_name = String()
    dict_list = []
    dropbox_dict = {}

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    
    # Parse parameters into individual variables

    # Rotate PR2 in place to capture side tables for the collision map
    
    #pr2_base_mover_pub.publish(-1.5)
    #rospy.sleep(15.0)

    #pr2_base_mover_pub.publish(1.5)
    #rospy.sleep(30.0)

    #pr2_base_mover_pub.publish(0)
    #rospy.sleep(15.0)
    
    # Loop through the pick list
    for i in range(len(object_list_param)):

        # Get the PointCloud for a given object and obtain it's centroid
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        # Find Detected objects
        object_labels = []
        object_centroids = []
        for object in object_list:
            if object.label == object_name.data:
                object_labels.append(object.label)
                points_arr = ros_to_pcl(object.cloud).to_array()
                centroid = np.mean(points_arr,axis=0)[:3]
                object_centroids.append(centroid)
		# Pick Pose
	        pick_pose.position.x = np.asscalar(centroid[0])
		pick_pose.position.y = np.asscalar(centroid[1])
		pick_pose.position.z = np.asscalar(centroid[2])
                # Place Pose
                if object_group == dropbox_param[0]['group']:
                    place_pose.position.x = dropbox_param[0]['position'][0]
                    place_pose.position.y = dropbox_param[0]['position'][1]
                    place_pose.position.z = dropbox_param[0]['position'][2]

                    arm_name.data = dropbox_param[0]['name']
                    print("param0 " + object_group)

                elif object_group == dropbox_param[1]['group']:
                    place_pose.position.x = dropbox_param[1]['position'][0]
                    place_pose.position.y = dropbox_param[1]['position'][1]
                    place_pose.position.z = dropbox_param[1]['position'][2]

                    arm_name.data = dropbox_param[1]['name']
                    print("param1 " + object_group)

                else:
                    print('dropbox not recognized')
                 
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
        print("wahoo")


    # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
    yaml_list = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
    dict_list.append(yaml_list)
    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

            #try:
            #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # Insert your message variables to be sent as a service request
            #    resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            #    print ("Response: ",resp.success)

            #except rospy.ServiceException, e:
            #    print "Service call failed: %s"%e

    print("we here")

    # Output your request parameters into output yaml file
    filename = 'output' + str(test_scene_num.data) + '.yaml'
    send_to_yaml(filename, dict_list)

    return


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", PointCloud2, pcl_callback, queue_size = 1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pr2_base_mover_pub = rospy.Publisher("pr2/world_joint_controller/command", Float64, queue_size=10)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
