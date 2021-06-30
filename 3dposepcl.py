#!/usr/bin/env python
#!coding=utf-8
#mediapipe imports
import cv2
import time
import mediapipe as mp

#ros imports
import rospy
import os
import sys
from sensor_msgs.msg import Image,LaserScan
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
import struct

mpPose=mp.solutions.pose
pose=mpPose.Pose()
mpDraw=mp.solutions.drawing_utils
 
    
class pose3d_estim:
    def __init__(self):
        self.rgb_image_sub = rospy.Subscriber("/camera/color/image_raw", Image,self.img_callback)
        self.pcl_sub = rospy.Subscriber("/camera/depth/points", pc2.PointCloud2,self.pcl_callback)
        self.marker_pub=rospy.Publisher("marker_test", MarkerArray, queue_size=10)
        self.bridge = CvBridge()
        self.spheres,self.linelist=Marker(),Marker()
        self.marker_array= MarkerArray()
     
 
    def pose_2d_pts(self,image):

        """ Gets 2d pose from image. """
        '''
        image- rgb image 
        return:-
        pts - list of 2d pose landmarks as img coords
        image- rgb image on which the 2d pose landmarks are drawn
        '''   
        pts=[]
        imgRGB=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        results=pose.process(imgRGB)
        if results.pose_landmarks:
            mpDraw.draw_landmarks(image,results.pose_landmarks,mpPose.POSE_CONNECTIONS)
            for id,lm in enumerate(results.pose_landmarks.landmark):
                h,w,c=image.shape
                imgx,imgy=int(lm.x*w),int(lm.y*h)
                
                pts.append((imgx,imgy)) 
        return pts,image

    def create_line_list(self,point_data):
        """ Creates linelist marker in kinect_frame. """

        '''
        point_data- rgb pointcloud data
        '''

        try:
            body=[['shoulder_line',[self.rpts[11],self.rpts[12]]],['waist_line',[self.rpts[23],self.rpts[24]]],['left_shoulder_waist',[self.rpts[11],self.rpts[23]]],['right_shoulder_waist',[self.rpts[12],self.rpts[24]]],['right_thigh',[self.rpts[24],self.rpts[26]]],['left_thigh',[self.rpts[23],self.rpts[25]]],['right_leg',[self.rpts[26],self.rpts[28]]],['left_leg',[self.rpts[25],self.rpts[27]]],['right_forearm',[self.rpts[14],self.rpts[16]]],['left_forearm',[self.rpts[13],self.rpts[15]]],['right_bicep',[self.rpts[12],self.rpts[14]]],['left_bicep',[self.rpts[11],self.rpts[13]]]]
            self.linelist.points=[]
            self.linelist.header.frame_id = "kinect_frame"
            self.linelist.header.stamp = rospy.Time.now()
            self.linelist.type = Marker.LINE_LIST
        
            self.linelist.id=1
            self.linelist.action = Marker.ADD 
            self. linelist.scale.x = 0.05

            self.linelist.color.g = 1.0
            self.linelist.color.a = 1.0

            # nose_x,nose_y=self.rpts[0]
            # nose3d_pt=list(pc2.read_points(point_data,skip_nans=True,field_names=('x', 'y', 'z'),uvs=[(nose_x, nose_y)]))
            # x_n,y_n,z_n=nose3d_pt[0]
            # self.z_n=z_n

            for _,pointl in body:
                for pt in pointl:
                    ptl_x,ptl_y,ptl_z=list(pc2.read_points(point_data,skip_nans=True,field_names=('x', 'y', 'z'),uvs=[(pt[0],pt[1])]))[0]
                   
                    self.linelist_point=Point()
                    self.linelist_point.x = ptl_x
                    self.linelist_point.y = ptl_y
                    self.linelist_point.z = ptl_z
                    self.linelist.points.append(self.linelist_point)
                
        except:
            pass         
    def create_spheres(self,point_data):
        """ Creates sphere list marker in kinect_frame. """


        '''
        point_data- rgb pointcloud data
        '''

        try:
            #points=[nose,left_wrist,right,wrist,left_ankle,right ankle]
            points=[self.rpts[0],self.rpts[15],self.rpts[16],self.rpts[27],self.rpts[28]]
            self.spheres.points=[]
            self.spheres.header.frame_id = "kinect_frame"
            self.spheres.header.stamp= rospy.Time.now()
                                
            self.spheres.id = 0
            self.spheres.action =Marker.ADD
                
                    #points
            self.spheres.type = Marker.SPHERE_LIST
            self.spheres.color.r = 1.0
            self.spheres.color.a = 1.0
                    
            self.spheres.scale.x = 0.08
            self.spheres.scale.y = 0.08
            self.spheres.scale.z = 0.01
            for p in points:
                pts_x,pts_y,pts_z=list(pc2.read_points(point_data,skip_nans=True,field_names=('x', 'y', 'z'),uvs=[(p[0],p[1])]))[0]
                
                self.sphere_point=Point()
                self.sphere_point.x = pts_x
                self.sphere_point.y = pts_y
                self.sphere_point.z = pts_z
                self.spheres.points.append(self.sphere_point)
                        
        
        except:
            pass            
                    

    def img_callback(self,rgb_data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(rgb_data,"bgr8")
            self.rpts,self.rimg=self.pose_2d_pts(self.img)
            
        except CvBridgeError as e:
            print(e)
           
        cv2.imshow('image',self.rimg)
        cv2.waitKey(1)

    def pcl_callback(self,point_data):
        self.create_line_list(point_data)
        self.create_spheres(point_data)
        self.marker_array.markers.extend([self.spheres,self.linelist])
        self.marker_pub.publish(self.marker_array)
    


def main(args):
  rospy.init_node('pose_estim', anonymous=True)
  p3d= pose3d_estim()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)