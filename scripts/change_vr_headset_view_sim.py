#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy , Image, CameraInfo, CompressedImage
from std_msgs.msg import String
import sys
"""
The idea of this code is to redirect different camera streams for the Vive Headset VR based on VIVE controller commands
For this to work, the topic that the VR should be listening to should be:

sub_L_str = "/vr_view/left/image"
sub_R_str = "/vr_view/right/image"
sub_i_L_str = "/vr_view/left/camera_info"
sub_i_R_str = "/vr_view/right/camera_info"

(This need to be set in the vive_node.cpp file in the windows PC)

The scripts assumes there are 3 camera views from which to switch from (one on the left of Tiago, on Tiago's head and on on right)
"""

SIMULATION = False
VIVE_CONTROLLER = 1 # 1 for setting the change vr view using left controller, 0 for right controller


class ChangeView():
    def __init__(self):
            
        # Reading arguments
        if sys.argv[1]==None or sys.argv[2]==None:
            print("Not all arguments were passed, setting default values")
            print("Using left controller and running for 3 cameras")
            
        self.vive_controller = 'left_controller' # left controller
        self.num_cameras = 4 # In real world we use 4 zeds, in simulation 4
        
        if sys.argv[1]=='left_controller':
            print("Using left controller")
            self.vive_controller = 'left_controller' # left controller
            
        elif sys.argv[1]=='right_controller':
            print("Using right controller")
            self.vive_controller = 'right_controller' # right controller
            
        if sys.argv[2]=='2':   
            print('Setting up 2 cameras')
            self.num_cameras = 2 
        
        if sys.argv[2]=='3':   
            print('Setting up 3 cameras')
            self.num_cameras = 3
        
        if sys.argv[2]=='4':
            print("Setting up 4 cameras")
            self.num_cameras = 4
                 
        # Publishing to topics in VR
        self.view_pub_left = rospy.Publisher('/zed2/zed_node/left/image_rect_color/compressed',CompressedImage, queue_size=5)
        self.view_pub_right = rospy.Publisher('/zed2/zed_node/right/image_rect_color/compressed',CompressedImage, queue_size=5)
        self.view_pub_left_info = rospy.Publisher('/zed2/zed_node/left/camera_info',CameraInfo, queue_size=5)
        self.view_pub_right_info = rospy.Publisher('/zed2/zed_node/right/camera_info',CameraInfo, queue_size=5)

        # Publish the current camera view being used
        self.current_camera_pub = rospy.Publisher('/camera_view', String, queue_size=10)
        
        # Initial configuration set for head camera
        self.camera_state = 0 # 0 Head, 1 Right, 2 Left, 3 for 3rd person view

        # Mapping vr controller mousepad
        self.record_axis = [0.0 for i in range(10)]
        self.rate = rospy.Rate(200)

        # Getting button information from left/right vive controller
        if self.vive_controller == 'left_controller':
            self.button_sub = rospy.Subscriber('/vive/my_left_controller_1/joy', Joy, self.button_cb)
            self.button_state = Joy()

        elif self.vive_controller == 'right_controller':
            self.button_sub = rospy.Subscriber('/vive/my_right_controller_1/joy', Joy, self.button_cb)
            self.button_state = Joy()

        # Getting image topics according to simulation/real world
        if self.num_cameras == 2:
            self.camera_list = [0,1] # Only two cameras to switch from
            # Left camera (ZED2 A) 
            self.zedA_left_sub = rospy.Subscriber('/zedA/zed_node_A/left/image_rect_color/compressed', CompressedImage,self.zedA_left_cb)
            self.zedA_left = Image()
            self.zedA_right_sub = rospy.Subscriber('/zedA/zed_node_A/right/image_rect_color/compressed', CompressedImage,self.zedA_right_cb)
            self.zedA_right = Image()
            self.zedA_left_info_sub = rospy.Subscriber('/zedA/zed_node_A/left/camera_info', CameraInfo,self.zedA_left_info_cb)
            self.zedA_left_info = CameraInfo()
            self.zedA_right_info_sub = rospy.Subscriber('/zedA/zed_node_A/right/camera_info', CameraInfo,self.zedA_right_info_cb)
            self.zedA_right_info = CameraInfo()
            
            # Head camera
            self.zedC_left_sub = rospy.Subscriber('/zedC/zed_node_C/left/image_rect_color/compressed', CompressedImage,self.zedC_left_cb)
            self.zedC_left = Image()
            self.zedC_right_sub = rospy.Subscriber('/zedC/zed_node_C/right/image_rect_color/compressed', CompressedImage,self.zedC_right_cb)
            self.zedC_right = Image()
            self.zedC_left_info_sub = rospy.Subscriber('/zedC/zed_node_C/left/camera_info', CameraInfo,self.zedC_left_info_cb)
            self.zedC_left_info = CameraInfo()
            self.zedC_right_info_sub = rospy.Subscriber('/zedC/zed_node_C/right/camera_info', CameraInfo,self.zedC_right_info_cb)
            self.zedC_right_info = CameraInfo()
            
        if self.num_cameras == 3:
            self.camera_list = [0,1,2] # Only three cameras to switch from
            # Left camera (ZED2 A) 
            self.zedA_left_sub = rospy.Subscriber('/zedA/zed_node_A/left/image_rect_color/compressed', CompressedImage,self.zedA_left_cb)
            self.zedA_left = Image()
            self.zedA_right_sub = rospy.Subscriber('/zedA/zed_node_A/right/image_rect_color/compressed', CompressedImage,self.zedA_right_cb)
            self.zedA_right = Image()
            self.zedA_left_info_sub = rospy.Subscriber('/zedA/zed_node_A/left/camera_info', CameraInfo,self.zedA_left_info_cb)
            self.zedA_left_info = CameraInfo()
            self.zedA_right_info_sub = rospy.Subscriber('/zedA/zed_node_A/right/camera_info', CameraInfo,self.zedA_right_info_cb)
            self.zedA_right_info = CameraInfo()

            # Right Camera (ZED2 B) 
            self.zedB_left_sub = rospy.Subscriber('/zedB/zed_node_B/left/image_rect_color/compressed', CompressedImage,self.zedB_left_cb)
            self.zedB_left = Image()
            self.zedB_right_sub = rospy.Subscriber('/zedB/zed_node_B/right/image_rect_color/compressed', CompressedImage,self.zedB_right_cb)
            self.zedB_right = Image()
            self.zedB_left_info_sub = rospy.Subscriber('/zedB/zed_node_B/left/camera_info', CameraInfo,self.zedB_left_info_cb)
            self.zedB_left_info = CameraInfo()
            self.zedB_right_info_sub = rospy.Subscriber('/zedB/zed_node_B/right/camera_info', CameraInfo,self.zedB_right_info_cb)
            self.zedB_right_info = CameraInfo()
            
            # Head camera
            self.zedC_left_sub = rospy.Subscriber('/zedC/zed_node_C/left/image_rect_color/compressed', CompressedImage,self.zedC_left_cb)
            self.zedC_left = Image()
            self.zedC_right_sub = rospy.Subscriber('/zedC/zed_node_C/right/image_rect_color/compressed', CompressedImage,self.zedC_right_cb)
            self.zedC_right = Image()
            self.zedC_left_info_sub = rospy.Subscriber('/zedC/zed_node_C/left/camera_info', CameraInfo,self.zedC_left_info_cb)
            self.zedC_left_info = CameraInfo()
            self.zedC_right_info_sub = rospy.Subscriber('/zedC/zed_node_C/right/camera_info', CameraInfo,self.zedC_right_info_cb)
            self.zedC_right_info = CameraInfo()

            
        elif self.num_cameras == 4:
            self.camera_list = [0,1,2,3] # four cameras to switch from
            # Left camera (ZED2 A) 
            self.zedA_left_sub = rospy.Subscriber('/zedA/zed_node_A/left/image_rect_color/compressed', CompressedImage,self.zedA_left_cb)
            self.zedA_left = Image()
            self.zedA_right_sub = rospy.Subscriber('/zedA/zed_node_A/right/image_rect_color/compressed', CompressedImage,self.zedA_right_cb)
            self.zedA_right = Image()
            self.zedA_left_info_sub = rospy.Subscriber('/zedA/zed_node_A/left/camera_info', CameraInfo,self.zedA_left_info_cb)
            self.zedA_left_info = CameraInfo()
            self.zedA_right_info_sub = rospy.Subscriber('/zedA/zed_node_A/right/camera_info', CameraInfo,self.zedA_right_info_cb)
            self.zedA_right_info = CameraInfo()

            # Right Camera (ZED2 B) 
            self.zedB_left_sub = rospy.Subscriber('/zedB/zed_node_B/left/image_rect_color/compressed', CompressedImage,self.zedB_left_cb)
            self.zedB_left = Image()
            self.zedB_right_sub = rospy.Subscriber('/zedB/zed_node_B/right/image_rect_color/compressed', CompressedImage,self.zedB_right_cb)
            self.zedB_right = Image()
            self.zedB_left_info_sub = rospy.Subscriber('/zedB/zed_node_B/left/camera_info', CameraInfo,self.zedB_left_info_cb)
            self.zedB_left_info = CameraInfo()
            self.zedB_right_info_sub = rospy.Subscriber('/zedB/zed_node_B/right/camera_info', CameraInfo,self.zedB_right_info_cb)
            self.zedB_right_info = CameraInfo()
            
            # Head camera
            self.zedC_left_sub = rospy.Subscriber('/zedC/zed_node_C/left/image_rect_color/compressed', CompressedImage,self.zedC_left_cb)
            self.zedC_left = Image()
            self.zedC_right_sub = rospy.Subscriber('/zedC/zed_node_C/right/image_rect_color/compressed', CompressedImage,self.zedC_right_cb)
            self.zedC_right = Image()
            self.zedC_left_info_sub = rospy.Subscriber('/zedC/zed_node_C/left/camera_info', CameraInfo,self.zedC_left_info_cb)
            self.zedC_left_info = CameraInfo()
            self.zedC_right_info_sub = rospy.Subscriber('/zedC/zed_node_C/right/camera_info', CameraInfo,self.zedC_right_info_cb)
            self.zedC_right_info = CameraInfo()
            
            # 3rd person view camera
            self.zedD_left_sub = rospy.Subscriber('/zedD/zed_node_D/left/image_rect_color/compressed', CompressedImage,self.zedD_left_cb)
            self.zedD_left = Image()
            self.zedD_right_sub = rospy.Subscriber('/zedD/zed_node_D/right/image_rect_color/compressed', CompressedImage,self.zedD_right_cb)
            self.zedD_right = Image()
            self.zedD_left_info_sub = rospy.Subscriber('/zedD/zed_node_D/left/camera_info', CameraInfo,self.zedD_left_info_cb)
            self.zedD_left_info = CameraInfo()
            self.zedD_right_info_sub = rospy.Subscriber('/zedD/zed_node_D/right/camera_info', CameraInfo,self.zedD_right_info_cb)
            self.zedD_right_info = CameraInfo()

    def button_cb(self,button_data):  
        self.record_axis.pop(0)
        self.record_axis.append(button_data.axes[0])

        if abs(min(self.record_axis)-max(self.record_axis))>1.5:
            rospy.loginfo('Camera Changed')
            self.record_axis = [0.0 for i in range(10)]

            # Jump to next camera
            self.camera_state = (self.camera_state+1)%self.num_cameras
            print(self.camera_state)
           

    def zedA_left_cb(self,camera_data):
        #self.zedA_left = camera_data
        if self.camera_state == 2:
            self.view_pub_left.publish(camera_data)
            self.current_camera_pub.publish(String(data="zedA"))
            
    def zedA_right_cb(self,camera_data):
        #self.zedA_right = camera_data
        if self.camera_state == 2:
            self.view_pub_right.publish(camera_data)
            
    def zedA_left_info_cb(self,camera_data):
        #self.zedA_left_info = camera_data
        if self.camera_state == 2:
            self.view_pub_left_info.publish(camera_data)
            
    def zedA_right_info_cb(self,camera_data):
        #self.zedA_right_info = camera_data
        if self.camera_state == 2:
            self.view_pub_right_info.publish(camera_data)
            
   
    def zedB_left_cb(self,camera_data):
        self.zedB_left = camera_data
        if self.camera_state == 1:
            self.view_pub_left.publish(self.zedB_left)
            self.current_camera_pub.publish(String(data="zedB"))
            
    def zedB_right_cb(self,camera_data):
        self.zedB_right = camera_data
        if self.camera_state == 1:
            self.view_pub_right.publish(self.zedB_right)
            
    def zedB_left_info_cb(self,camera_data):
        self.zedB_left_info = camera_data
        if self.camera_state == 1:
            self.view_pub_left_info.publish(self.zedB_left_info)
            
    def zedB_right_info_cb(self,camera_data):
        self.zedB_right_info = camera_data
        if self.camera_state == 1:
            self.view_pub_right_info.publish(self.zedB_right_info)
            
            
    def zedC_left_cb(self,camera_data):
        self.zedC_left = camera_data
        if self.camera_state == 0:
            self.view_pub_left.publish(self.zedC_left)
            self.current_camera_pub.publish(String(data="zedC"))
            
    def zedC_right_cb(self,camera_data):
        self.zedC_right = camera_data
        if self.camera_state == 0:
            self.view_pub_right.publish(self.zedC_right)
            
    def zedC_left_info_cb(self,camera_data):
        self.zedC_left_info = camera_data
        if self.camera_state == 0:
            self.view_pub_left_info.publish(self.zedC_left_info)
            
    def zedC_right_info_cb(self,camera_data):
        self.zedC_right_info = camera_data
        if self.camera_state == 0:
            self.view_pub_right_info.publish(self.zedC_right_info)
            
            
    def zedD_left_cb(self,camera_data):
        self.zedD_left = camera_data
        if self.camera_state == 3:
            self.view_pub_left.publish(self.zedD_left)
            self.current_camera_pub.publish(String(data="zedD"))
            
    def zedD_right_cb(self,camera_data):
        self.zedD_right = camera_data
        if self.camera_state == 3:
            self.view_pub_right.publish(self.zedD_right)
            
    def zedD_left_info_cb(self,camera_data):
        self.zedD_left_info = camera_data
        if self.camera_state == 3:
            self.view_pub_left_info.publish(self.zedD_left_info)
            
    def zedD_right_info_cb(self,camera_data):
        self.zedD_right_info = camera_data
        if self.camera_state == 3:
            self.view_pub_right_info.publish(self.zedD_right_info)



if __name__=="__main__":
    rospy.init_node('camera_view_changer')
    changeview = ChangeView()
    while not rospy.is_shutdown():
        changeview.rate.sleep()