# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter-------------------------
# creating voxelgrid filter object
vox = cloud.make_voxel_grid_filter()
#voxel size
LEAF_SIZE = 0.01
vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
#call filter function to get downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter-----------------------
#note how we use a different class this time. It's not just a cloud but is a filtered cloud!
passthrough = cloud_filtered.make_passthrough_filter()

#ok now we're defining a z-axis. Apparently this package has oriented the point cloud? No,
#we probably have that from before.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis) #ah so it's just a name now?
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

#and again with this filter function
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd' #pcd = point cloud data
pcl.save(cloud_filtered, filename)

# RANSAC plane segmentation-------------------
seg = cloud_filtered.make_segmenter()

#set the model type
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

max_distance = 0.01
seg.set_distance_threshold(max_distance)

inliers, coefficients = seg.segment()

# Extract inliers---------------------------
#the extract indices filter is not really a filter but it just lets you get the point cloud
#associated with the indices of interest that you've identified. 
extracted_inliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers----------------------------
outlier_filter = cloud_filtered.make_statistical_outlier_filter()
#this is where we choose the number of data points we want
outlier_filter.set_mean_k(50)
#set threshold scale factor
x = 1.0
#any point with a mean distance larger than global mean will be considered an outlier
outlier_filter.set_std_dev_mul_thresh(x)
cloud_filtered = outlier_filter.filter()

# Save pcd for tabletop objects


