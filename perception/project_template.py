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
    print(dict_list)	
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
    	yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    PointXYZRGB = ros_to_pcl(pcl_msg)
    cloud = PointXYZRGB

    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min=0.605
    axis_max=1.1
    passthrough.set_filter_limits(axis_min,axis_max)
    cloud_filtered = passthrough.filter()
    
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min=0.35
    axis_max=1.1
    passthrough.set_filter_limits(axis_min,axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance=0.01
    seg.set_distance_threshold(max_distance)
    inliers,coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    cloud_table   = cloud_filtered.extract(inliers, negative=False)

    # TODO: Euclidean Clustering
   
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(75)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    print(len(cluster_indices))

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
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    #ros_cloud_filtered   = pcl_to_ros(cloud_filtered)
    #ros_cloud_objects = pcl_to_ros(cloud_objects)
    #ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    #filtered_scene_pub.publish(ros_cloud_filtered)
    #pcl_objects_pub.publish(ros_cloud_objects)
    #pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects=[]

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        #labeled_feature = [feature, model_name]

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

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
def pr2_mover(found_object_list):
    
    #the pick list
    pick_list = rospy.get_param('/object_list')
    pick_dictionary = {}
    for object in pick_list:
        pick_dictionary[object['name']] = object['group']

    #the place list
    place_list = rospy.get_param('/dropbox')
    
    #take the place list and make a dictionary so we can just pup in the color and get the coordinates
    place_dictionary = {}
    for pose in place_list:
	place_dictionary[pose['group']] = pose['position']

    #same so we can pop in the color and get the arm    
    arm_dictionary = {}
    for pose in place_list:
        arm_dictionary[pose['group']] = pose['name']

    #print(arm_dictionary)     

    labels=[]
    centroids=[]

    test_scene_num = Int32()
    test_scene_num.data = 2
   
    #get the predicted labels and centroids from the classification algorithm
    for object in found_object_list:
        
        labels.append(object.label)               
    	points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    
    #make those centroids standard python
    for i in range(len(centroids)):
	for j in range(3):
		centroids[i][j] = np.asscalar(centroids[i][j])
   
    dict_list = []

    #start by choosing a label from the classified objects
    for i in range(len(labels)):
        #check it against all of the objects in the pick list
	for j in range(len(pick_list)):
		#once we match the object with one in the pick list we can create a .yaml file
		if labels[i] == pick_list[j]['name']:
			
             		print(labels[i]) 
			test_scene_num = Int32()
    			test_scene_num.data = 1
	                 
                        #the name can come from label or object list		
			object_name = String()
			object_name.data =str(labels[i])
			

			arm_name = String()
			arm_name.data = arm_dictionary[pick_dictionary[labels[i]]]				

			pick_pose = Pose()
			x = Float64()
			y = Float64()
			z = Float64()
			x = float(centroids[i][0])
			y = float(centroids[i][1])
                        z = float(centroids[i][2])
                        pick_pose.position.x = x
			pick_pose.position.y = y 
			pick_pose.position.z = z
			
                        
			place_pose = Pose()
                        x2 = Float64()
                        y2 = Float64()
                        z2 = Float64()
                        x2 = float(place_dictionary[pick_dictionary[labels[i]]][0])
                        y2 = float(place_dictionary[pick_dictionary[labels[i]]][1])
                        z2 = float(place_dictionary[pick_dictionary[labels[i]]][2])
                        place_pose.position.x = x2
                        place_pose.position.y = y2
                        place_pose.position.z = z2
                        
			print(test_scene_num, arm_name, object_name, pick_pose, place_pose)
			yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                        dict_list.append(yaml_dict)
			print("We have a match!") 
			
    send_to_yaml('output_3.yaml', dict_list) 
        # TODO: Create 'place_pose' for the object
        
        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # TODO: Output your request parameters into output yaml file


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous=True)

    # TO:DO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    #filtered_scene_pub = rospy.Publisher("/filtered_scene", PointCloud2, queue_size=1)
    #pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    #pcl_table_pub   = rospy.Publisher("/pcl_table"  , PointCloud2, queue_size=1)
    pcl_cluster_pub   = rospy.Publisher("/pcl_cluster"  , PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects"  , DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers"  , Marker, queue_size=1)

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

