## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

[//]: # (Image References)

[env]: ./misc/env.png
[cloud]: ./misc/cloud.png
[stat]: ./misc/stat.png
[vox]: ./misc/vox.png
[pass_z]: ./misc/pass_z.png
[pass_yz]: ./misc/pass_yz.png
[pass_xyz]: ./misc/pass_xyz.png
[cluster]: ./misc/cluster.png
[table]: ./misc/table.png
[objects]: ./misc/objects.png
[svm]: ./misc/svm.png
[recog_1]: ./misc/recog_1.png
[recog_2]: ./misc/recog_2.png
[recog_3]: ./misc/recog_3.png

[robot-joint-space-2]: ./misc_images/robot-joint-space-2.png
[inverse-calculation]: ./misc_images/inverse-calculation.png
[screenshot]: ./misc_images/screenshot.png

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! 

### Exercise 1, 2 and 3 pipeline implemented

The PR2 robot is fitted with an RGB-D camera that provides depth-per-pixel information in addition to an RGB image. 
This camera continuously captures the point cloud from the robot's environment and writes to the ros topic `/pr2/world/points`.
This serves as our input to the perception pipeline. Here is an example image from __Test World 1__ depicting how the original 
environment with objects lying on the table looks like from the robot's perspective.

![alt text][env]

#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The input received from the camera is a point cloud data in the form of _ROS PointCloud2_ message object. 
We need to convert it to _PCL PointXYZRGB_ to be able to use it in our pipeline and do further processing. 
The following code does the job for us.

```python
    # TODO: Convert ROS msg to PCL data
    cloud_filtered =  ros_to_pcl(pcl_msg)
```

Initially the input cloud may be noisy as the following image.

![alt text][cloud]

This data needs to be cleaned and filtered to obtain point cloud corresponding to the objects of interest only.
To do this we first apply __PCL’s Statistical Outlier Filter__ to get rid of the noise in the point cloud data. 
Assuming a Gaussian distribution, we filter out the noise as follows:

```python
    # TODO: Statistical Outlier Filtering
    # Much like the previous filters, we start by creating a filter object:
    stat = point_cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    stat.set_mean_k(8)

    # Set threshold scale factor
    x = 0.3

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    stat.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    stat_filtered = stat.filter()
```

The filtered data looks much cleaner like the following image as compared to the image in the figure above:

![alt text][stat]

The data volume of the cloud points is huge and would require enormous computation power. 
The cloud points may be downsampled to reduce the computation to a certain extent without impacting the detection result 
using __Voxel Grid Downsampling__ as follows.

```python
    # TODO: Voxel Grid Downsampling

    # Create a VoxelGrid filter object for our input point cloud
    vox = stat_filtered.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.0075

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    vox_filtered = vox.filter()
    # filename = 'voxel_downsampled.pcd'
    # pcl.save(cloud_filtered, filename)
```

The result is as follows where the point cloud is less dense but the individual envirenment elements are still recognizable:

![alt text][vox]

It may be noted here that the region denoted by the point cloud is much bigger than the region of interest on the table 
where the objects are placed. The reducing the computation further more, we can crop the regions external to the region 
of interest. This is accomplished the following code using __Pass Through Filtering__, first anong `z-axis` to get rid 
of the height (thickness) of the table:

```python
    # TODO: PassThrough Filter

    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough = vox_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    pass_filtered_z = passthrough.filter()
```

![alt text][pass_z]

followed by along `y-axis`, to crop off the left and right edges of the table :

```python
    passthrough = pass_filtered_z.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = 0.45
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    pass_filtered_yz = passthrough.filter()
    # filename = 'pass_through_filtered.pcd'
    # pcl.save(cloud_filtered, filename)
```
![alt text][pass_yz]

and finally along `x-axis` to achieve a rectangular region of interest.
```python
    passthrough = pass_filtered_yz.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.35
    axis_max = 1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    pass_filtered_xyz = passthrough.filter()
    # filename = 'pass_through_filtered.pcd'
    # pcl.save(cloud_filtered, filename)
```

![alt text][pass_xyz]

Next we remove the table such that only the objects of interest remain. In that way it will become easier to identify the objects individually.
Since we already have the table surface plane model with us, we can use __Random Sample Consensus__ or RANSAC plane segmentation algorithm to achieve this.

```python
    # TODO: RANSAC Plane Segmentation

    # Create the segmentation object
    seg = pass_filtered_xyz.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = 0.02
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    # Extract inliers
    extracted_inliers = pass_filtered_xyz.extract(inliers, negative=False)
    # filename = 'extracted_inliers.pcd'
    # pcl.save(extracted_inliers, filename)

    extracted_outliers = pass_filtered_xyz.extract(inliers, negative=True)
    # filename = 'extracted_outliers.pcd'
    # pcl.save(extracted_outliers, filename)
```

The separated out objects and the table top is shown int he following figure:

![alt text][table]

![alt text][objects]

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented. 

Now since we have only the cloud points corresponding to individual objects, we can segment the cloud points and cluster 
them such that the objects can be recognized individually. We use __PCL's Euclidean Clustering__ algorithm for this purpose as follows:

```python
    # TODO: Euclidean Clustering
    # Euclidean Clustering
    white_cloud =  XYZRGB_to_XYZ(extracted_outliers)# Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.04)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
```
For the purpose of visualization, we color the clusters uniquely and the visualization result follows. 

```python
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
```

![alt text][cluster]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Now it is time to detect the objects from the individual clusters. Since each object has a unique shape and size, we will use
a simple machine learning algorithm known as __Support Vector Machine__ or SVM to predict the object classes in this case.

We'll use the provided scripts [capture_features.py](./sensor_stick/scripts/capture_features.py) to first capture the 
features of the objects followed by [train_svm.py](./sensor_stick/scripts/train_svm.py) to train the svm model.

We first capture 200 example images of each object (total 8 objects ) which results a total of 1600 features across 8 classes.
The svm reached an accuracy of 94% as can be seen in the figure below.

![alt text][svm]
 
The trained model can be found here [model.sav](./model.sav). We load this model in the script 
[project_template.py](./pr2_robot/scripts/project_template.py) as follows:

```python
    # TODO: Load Model From disk
    model = pickle.load(open('src/RoboND-Perception-Project/model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
```

Now we can use the model to predict the objects from the clusters and publish the result which will be used by the 
PR2 robot to actuate the movements.

```python
# Classify the clusters!
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = extracted_outliers.extract(pts_list)

        # TODO: convert the cluster from pcl to ROS using helper function
        sample_cloud = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .3
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = sample_cloud
        detected_objects.append(do)
```


### Pick and Place Setup

---

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.


The following are the detection results for the 3 worlds and their corresponding ``output yaml``:

**Test 1 World [output_1.yaml](./output_1.yaml)**. All objects were detected correctly.
![alt text][recog_1]
---

**Test 2 World [output_2.yaml](./output_2.yaml)**. Again, all objects were detected correctly.
![alt text][recog_2]
---

**Test 3 World[output_3.yaml](./output_3.yaml)**. 7 out of 8 objects were detected correctly. The `glue` was incorrectly 
detected as sticky_notes.

![alt text][recog_3]
---

### Comments and Discussion

The incorrect detection is mostly because the glue is the test 3 world is partially occluded by the book in front of it. 
Training the svm with a larger dataset should fix the problem which I could not perform due to system constraints and time. 

Also I faced a strange issue that whenever there were an incorrect detection, the code in line 324 of 
[project_template.py](./pr2_robot/scripts/project_template.py) would break and exit as it can't find the `name` from 
the `object_list_param` in the `labels` list. To handle this I had to modify the introduce a 
`try-except` mechanism as follows:

```python
        # Get index of object from stored list
        obj_index = -1
        try:
            obj_index = labels.index(object_list_param[i]['name'])
        except:
            pass

        # Stop if object was not detected in the scene
        if (obj_index == -1):
            rospy.loginfo('Object not detected')
            continue
```
This fixed the issue and the code ran correctly there after.