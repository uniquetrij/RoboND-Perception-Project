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

###########################################################
from pcl_helper import ros_to_pcl, pcl_to_ros, get_color_list, rgb_to_float, XYZRGB_to_XYZ

###########################################################


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

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    data =  ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier = data.make_statistical_outlier_filter()
    outlier.set_mean_k(10)
    x = 0.5
    outlier.set_std_dev_mul_thresh(x)
    out_filtered = outlier.filter()

    # TODO: Voxel Grid Downsampling
    voxel = out_filtered.make_voxel_grid_filter()
    LEAF_SIZE =.005
    voxel.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud = voxel.filter()

    # TODO: PassThrough Filter
    pass_z = cloud.make_passthrough_filter()
    filter_axis = 'z'
    pass_z.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    pass_z.set_filter_limits(axis_min, axis_max)
    cloud = pass_z.filter()

    pass_y = cloud.make_passthrough_filter()
    filter_axis = 'y'
    pass_y.set_filter_field_name(filter_axis)
    axis_min = -0.4
    axis_max = 0.4
    pass_y.set_filter_limits(axis_min, axis_max)
    cloud = pass_y.filter()

    # TODO: RANSAC Plane Segmentation
    segmenter = cloud.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.03
    segmenter.set_distance_threshold(max_distance)
    inliers, coefficients = segmenter.segment()

    # TODO: Extract inliers and outliers
    tabletop = cloud.extract(inliers, negative=False)
    items = cloud.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(items)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)# TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects_list = []
    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = items.extract(pts_list)

        # convert pcl to ros
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    TEST_SCENE_NUM = Int32()
    OBJECT_NAME = String()
    WHICH_ARM = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()

    object_name = []
    object_group = []
    labels = []
    centroids = [] # to be list of tuples (x, y, z)

    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    left_position = None
    right_position = None
    dict_list = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    drop_box_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    for i in range(len(drop_box_param)):
        if(drop_box_param[i]['name'] == 'left'):
            left_position = drop_box_param[i]['position']
        else:
            right_position = drop_box_param[i]['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in range(len(object_list_param)):

        # Get object name from pick list
        OBJECT_NAME.data = object_list_param[i]['name']

        # Specify the test scene number
        TEST_SCENE_NUM.data = 2

        # Get index of object from stored list
        obj_idx = labels.index(object_list_param[i]['name'])

        # Stop if object was not detected in the scene
        if (obj_idx == -1):
            rospy.loginfo('Object not detected')
            return

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        centroid = centroids[obj_idx]
        PICK_POSE.position.x = np.asscalar(centroid[0])
        PICK_POSE.position.y = np.asscalar(centroid[1])
        PICK_POSE.position.z = np.asscalar(centroid[2])

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place
        if (object_list_param[i]['group'] == 'red'):
            WHICH_ARM.data = 'left'
            PLACE_POSE.position.x = left_position[0]
            PLACE_POSE.position.y = left_position[1]
            PLACE_POSE.position.z = left_position[2]

        else:
            WHICH_ARM.data = 'right'
            PLACE_POSE.position.x = right_position[0]
            PLACE_POSE.position.y = right_position[1]
            PLACE_POSE.position.z = right_position[2]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('pick_and_place_main')

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
