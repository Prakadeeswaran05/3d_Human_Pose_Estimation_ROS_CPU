#!/usr/bin/env python
#!coding=utf-8
#mediapipe imports
import cv2
import time
import mediapipe as mp
import numpy as np
#ros imports
import rospy
import os
import sys
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
import struct
import message_filters as mf


 
    
class pose3d_estim:
    def __init__(self):
        self.rgb_image_sub = mf.Subscriber("/camera/color/image_raw", Image)
        self.depth_image_sub = mf.Subscriber("/camera/depth/image_raw",Image)
        self.camera_info_sub = mf.Subscriber("/camera/depth/camera_info",CameraInfo)
        self.mpPose=mp.solutions.pose
        self.pose=self.mpPose.Pose()
        self.mpDraw=mp.solutions.drawing_utils
        ts = mf.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub,self.camera_info_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_callback)
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
        results=self.pose.process(imgRGB)
        if results.pose_landmarks:
            self.mpDraw.draw_landmarks(image,results.pose_landmarks,self.mpPose.POSE_CONNECTIONS)
            for id,lm in enumerate(results.pose_landmarks.landmark):
                h,w,c=image.shape
                imgx,imgy=int(lm.x*w),int(lm.y*h)
                
                pts.append((imgx,imgy)) 
        return pts,image

    def create_line_list(self,depth_arr,rpts):
        """ Creates linelist marker in kinect_frame. """

        '''
        depth_arr- depth image as numpy array
        '''

        try:
            body=[['shoulder_line',[rpts[11],rpts[12]]],['waist_line',[rpts[23],rpts[24]]],['left_shoulder_waist',[rpts[11],rpts[23]]],
            ['right_shoulder_waist',[rpts[12],rpts[24]]],['right_thigh',[rpts[24],rpts[26]]],['left_thigh',[rpts[23],rpts[25]]],
            ['right_leg',[rpts[26],rpts[28]]],['left_leg',[rpts[25],rpts[27]]],['right_forearm',[rpts[14],rpts[16]]],
            ['left_forearm',[rpts[13],rpts[15]]],['right_bicep',[rpts[12],rpts[14]]],['left_bicep',[rpts[11],rpts[13]]]]
            self.linelist.points=[]
            self.linelist.header.frame_id = "kinect_frame"
            self.linelist.header.stamp = rospy.Time.now()
            self.linelist.type = Marker.LINE_LIST
            self.linelist.pose.orientation.w = 1.0
            self.linelist.id=1
            self.linelist.action = Marker.ADD 
            self. linelist.scale.x = 0.05

            self.linelist.color.g = 1.0
            self.linelist.color.a = 1.0

            

            for _,pointl in body:
                for pt in pointl:
                    depth_val=float(depth_arr[pt[1], pt[0]])
                    ptl_x,ptl_y,ptl_z=self.depth_to_xyz(pt[0],pt[1],depth_val)
                   
                    self.linelist_point=Point()
                    self.linelist_point.x = ptl_x
                    self.linelist_point.y = ptl_y
                    self.linelist_point.z = ptl_z
                    self.linelist.points.append(self.linelist_point)
                
        except:
            pass         
    def create_spheres(self,depth_arr,rpts):
        """ Creates sphere list marker in kinect_frame. """


        '''
        depth_arr-  depth image as numpy array
        '''

        try:
            #points=[nose,left_wrist,right,wrist,left_ankle,right ankle]
            points=[rpts[0],rpts[15],rpts[16],rpts[27],rpts[28]]
            self.spheres.points=[]
            self.spheres.header.frame_id = "kinect_frame"
            self.spheres.header.stamp= rospy.Time.now()
                                
            self.spheres.id = 0
            self.spheres.action =Marker.ADD
                
                    #points
            self.spheres.type = Marker.SPHERE_LIST
            self.spheres.pose.orientation.w = 1.0
            self.spheres.color.r = 1.0
            self.spheres.color.a = 1.0
                    
            self.spheres.scale.x = 0.08
            self.spheres.scale.y = 0.08
            self.spheres.scale.z = 0.01
            for p in points:
                depth_val=float(depth_arr[p[1], p[0]])
                pts_x,pts_y,pts_z=self.depth_to_xyz(p[0],p[1],depth_val)
                
                self.sphere_point=Point()
                self.sphere_point.x = pts_x
                self.sphere_point.y = pts_y
                self.sphere_point.z = pts_z
                self.spheres.points.append(self.sphere_point)
                    
        except:
            pass            
                    
    def depth_to_xyz(self,u,v,depth_val):
        """  xyz from u,v image coords """
        '''
        u - x image coordinate
        v - y image coodrinate
        depth_val - depth value at that (u,v) from depth_image
        '''

        fx=self.cam_intrin[0]
        fy=self.cam_intrin[4]
        cx=self.cam_intrin[2]
        cy=self.cam_intrin[5]

        z = float(depth_val)
        x = float((u - cx)/fx)*z
        y = float((v - cy)/fy)*z

        result = [x, y, z]
        return result

    def image_callback(self,rgb_data,depth_data,camera_data):
        try:
            img = self.bridge.imgmsg_to_cv2(rgb_data,"bgr8")
            d_img= self.bridge.imgmsg_to_cv2(depth_data,"passthrough")
            print(d_img.shape,img.shape)
            rpts,rimg=self.pose_2d_pts(img)
            depth_arr=np.array(d_img)
            
            self.cam_intrin=list(camera_data.K)
            self.create_line_list(depth_arr,rpts)
            self.create_spheres(depth_arr,rpts)
            self.marker_array.markers.extend([self.spheres,self.linelist])
            self.marker_pub.publish(self.marker_array)

            
        except CvBridgeError as e:
            print(e)
           
        cv2.imshow('image',rimg)
        cv2.waitKey(1)

   
  


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
