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

The PR2 robot is fitted with an RGB-D camera that provides per-pixel depth information in addition to an RGB image. 
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
To do this we first apply _PCL’s Statistical Outlier Filter_ to get rid of the noise in the point cloud data. 
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
of interest. This is accomplished the following code using __Pass Through__ filtering, first anong `z-axis` to get rid 
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



#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  

roslaunch sensor_stick training.launch
rosrun sensor_stick capture_features.py
rosrun sensor_stick train_svm.py

rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

roslaunch pr2_robot pick_place_project.launch

rosrun pr2_robot project_template.py