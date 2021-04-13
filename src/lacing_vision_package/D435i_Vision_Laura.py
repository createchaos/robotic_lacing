#################################################
###             c.r.e.A.te Lab                ###
###          Intel RealSense D435i            ###
###           Single-Frame Capture            ###
###       Corner Detection + Point Cloud      ###
#################################################

# Collects single RGB and Depth frames; aligns them for processing
# Masks image using simple bounding box in center of RGB image
# Run corner detection on masked region of image, projects to Depth, and obtains point coordinates (point cloud)
# Visualize RGB Image with Corners, Depth with projected corner points, and Point Cloud

import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import open3d as o3d                      # 3D point library, used for plane segmentation (not working)
from mpl_toolkits.mplot3d import Axes3D
from skspatial.objects import Plane
from skspatial.objects import Points


_pipe = None
_pipe = rs.pipeline()

def get_pipe():
    global _pipe
    return _pipe

def find_corners():

    # Configure depth and color streams
    pipe = get_pipe()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipe.start(config)

    # Skip the first few frames to give the Auto-Exposure time to adjust
    for x in range(20):
        pipe.wait_for_frames()
    
    # Store next frameset for later processing
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    # Cleanup
    pipe.stop()
    print("Frames Captured")

    # Get depth sensor's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    depth_to_color_extrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(profile.get_stream(rs.stream.color))
    color_to_depth_extrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(profile.get_stream(rs.stream.depth))

    # Align the streams
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    aligned_depth_frame = frameset.get_depth_frame()
    depth_image = np.asanyarray(aligned_depth_frame.get_data())

    aligned_color_frame = frameset.get_color_frame()
    color_image = np.asanyarray(aligned_color_frame.get_data())

    # Depth data
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())



    ### CORNER DETECTION ###
    # Convert color image to gray and detect corners
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    # corners	=	cv.goodFeaturesToTrack(	image, maxCorners, qualityLevel, minDistance[, corners[, mask[, blockSize[, useHarrisDetector[, k]]]]]	)
    corners = cv2.goodFeaturesToTrack(gray, 4, 0.01, 2)

    # Create lists for coordinates
    list_x = []
    list_y = []
    list_z = []
    list_pcd = []

    # Loop for detected corner points
    for i in corners:
        x, y = i.ravel()
        depth = depth_frame.get_distance(x,y)

        # Draw corner points in color image
        cv2.circle(color_image, (x,y), 3, (255,255,255), -1)
        color_point = np.asanyarray(i)

        # Project color pixel to depth image and return depth image pixel
        depth_px = rs.rs2_project_color_pixel_to_depth_pixel(
            aligned_depth_frame.get_data(),
            depth_scale,
            depth_min=0.0,
            depth_max=5.0,
            depth_intrin=depth_intrin,
            color_intrin=color_intrin,
            depth_to_color=depth_to_color_extrin,
            color_to_depth=color_to_depth_extrin,
            from_pixel=[x,y]
        )

        xd = round(depth_px[0])
        yd = round(depth_px[1])

        # Some depth image pixels are out of range, run only for postive, non-zero values
        if xd > 0:
            ddist = depth_frame.get_distance(xd, yd)
            print("at ", xd, ",", yd, " depth is ", ddist)

            if ddist > 0 and ddist < 1:
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [xd, yd], ddist)

                print(depth_point)

                # Add coordinates to list
                list_x.append(depth_point[0])
                list_y.append(depth_point[1])
                list_z.append(depth_point[2])

                # Create list of points (future use for point cloud processing?)
                list_pcd.append(depth_point)

                cv2.circle(depth_image, (xd,yd), 3, (255,255,255), -1)
                
    # Show the color and depth frames together:
    images = np.hstack((color_image, colorized_depth))
    plt.imshow(images)

    # Configure 3d plot window
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # Plot Points
    ax.scatter(list_x, list_y, list_z, c='red')
    # Plot triangulated surfaces
    #ax.plot_trisurf(list_x, list_y, list_z, color='grey')

    plt.show()

    bestPlane = Plane.best_fit(list_pcd)
    planePt = bestPlane.point
    planeVctr = bestPlane.normal

    str_ptsList = []
    for i in range(len(list_pcd)):
        strPt = str(list_pcd[i])
        print(strPt)
        strPtSpace = strPt.replace(',', '')
        print(strPtSpace)
        str_ptsList.append(strPtSpace)


    #output = [str(planePt), str(planeVctr), ]
    output = str(planePt) + ', ' + str(planeVctr) + ', ' + ', '.join(str_ptsList)
    print(output)

    # Output formatted as list [[planePt], [planeVctr], [[pt1], [pt2], [pt3], [pt4]]]
    return output

find_corners()


"""
# Color data
color_image = np.asanyarray(color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
#plt.imshow(color)

# Depth data
colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
#plt.imshow(colorized_depth)

colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

# MASKING COLOR IMAGE - Reduce area for corner detection
# Mask area size
x = 640
y = 480
w = color_frame.get_width()
h = color_frame.get_height()

x_neg = w//2 - x//2
y_neg = h//2 - y//2
x_pos = w//2 + x//2
y_pos = h//2 + y//2
         
# Create a mask image of zeros          
mask = np.zeros(color_image.shape,np.uint8)
# Use ROI processing to add the rectangle area of the color image to the masked image
mask[y_neg:y_pos, x_neg:x_pos] = color_image[y_neg:y_pos, x_neg:x_pos]

# Show the color and depth frames together:
images = np.hstack((mask, colorized_depth))
plt.imshow(images)

# Configure 3d plot window
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

# Plot Points
ax.scatter(list_x, list_y, list_z, c='red')
# Plot triangulated surfaces
#ax.plot_trisurf(list_x, list_y, list_z, color='grey')

plt.show()
"""


"""
### TRYING TO RUN PLANE SEGMENTATION WITH OPEN3D
### NOT WORKING
# Declare a point cloud object
pcd = o3d.geometry.PointCloud()

# numpy array of 3D point coordinates
verts = np.asanyarray(list_pcd).view(np.float32).reshape(-1,3)

# Convert points to point cloud
pcd.points = pcd.points = o3d.utility.Vector3dVector(verts)

# Plane segmentation
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# Visualizer
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

"""