# 3d_Human_Pose_Estimation_ROS_Cpu


This repo contains code that takes in estimated 2d anatomical landmarks in image coordinate(u,v) and convert them to world coordinate(x,y,z).Also,these projected landmarks are displayed in rviz with the help of sphere markers(joints) that are joined together with line markers(links). Here I have used two approaches to estimate the 3d landmarks.

1.To get world coordinate(x,y,z) from camera intrinsic parameters: (see 3dposedepth.py)</br>
    Z = depth_image[v,u] </br>
    X = (u - cx) * Z / fx </br>
    Y = (v - cy) * Z / fy </br>
    
where, </br>
cx=X-axis optical center in pixels </br>
cy=Y-axis optical center in pixels. </br>
fx=X-axis focal length in meters </br>
fy=X-axis focal length in meters </br>




2.Using the existing pc2.read_points function which takes in pointcloud and 2d anatomical landmark(in img coordinate) and returns the same point in world coordinate. (see 3dposepcl.py)

</br>

## Output

<p align="left">
  <img src="out_3dpose1.gif" />
</p>

</br>
</br>
</br>

## Output First Approach
<p align="left">
  <img src="out_intrinsic.gif" />
</p>

</br>
</br>
</br>

## Output Second Approach
<p align="left">
  <img src="out_3dpose.gif" />
</p>
 
</br> 
I personally found that first approach was faster than the second one. For the second approach, after moving human model in gazebo it took some time for 3d pose to appear in Rviz.</br>

### PS: I tested these codes in Ubuntu 18 VM without GPU
