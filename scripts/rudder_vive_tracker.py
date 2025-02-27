#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
import json
import math
class ViveTrackerController:
    def __init__(self, omni_drive = False):
        self.drivetype = rospy.get_param("~drivetype", "omni")
        self.driveSpeed = rospy.get_param("~driveSpeed", "constant")
        self.load_calibration_values()
        if self.driveSpeed == "constant":
            rospy.Subscriber("/vive/my_rudder_Pose", Pose, self.orientation_callback_constant)
        elif self.driveSpeed == "variable":
            rospy.Subscriber("/vive/my_rudder_Pose", Pose, self.orientation_callback_variable)
        self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self.initial_pose = None
        self.pitchThreshold = 0.15
        self.rollThreshold = 0.15
        self.yawThreshold = 0.07
        self.minPitchSpeed = 0.1
        self.maxPitchSpeed = 0.8
             
        rospy.loginfo("ViveTrackerController initialized.")

    def load_calibration_values(self):
        try:
            with open('calibration_values.json', 'r') as f:
                values = json.load(f)
                self.maxPitch = values["max_pitch"]
                self.minPitch = values["min_pitch"]
                self.maxLeftYaw = values["max_yaw"]
                self.maxRightYaw = values["min_yaw"]
                self.maxLeftRoll = values["max_roll"]
                self.maxRightRoll = values["min_roll"]

                self.maxPitch = self.check_value(self.maxPitch, 1.85)
                self.minPitch = self.check_value(self.minPitch, 1.3)
                self.maxLeftYaw = self.check_value(self.maxLeftYaw, 0.25)
                self.maxRightYaw = self.check_value(self.maxRightYaw, -0.32)
                self.maxLeftRoll = self.check_value(self.maxLeftRoll, 0.35)
                self.maxRightRoll = self.check_value(self.maxRightRoll, -0.6)

                rospy.loginfo("Loaded calibration values from file.")
        except IOError:
            rospy.logwarn("Could not read calibration values file. Using default values.")

            self.maxPitch = 1.85
            self.minPitch = 1.3
            self.maxLeftYaw = 0.25
            self.maxRightYaw = -0.32
            self.maxRightRoll = -0.6
            self.maxLeftRoll = 0.35

    def check_value(self, value, default):
        if math.isinf(value) or value == 0:
            return default
        return value

    def orientation_callback_constant(self, data):
        pitch = data.orientation.x
        yaw = data.orientation.z
        roll = data.orientation.y
        if self.initial_pose is None:
            self.initial_pose = data
            self.lastPitch = pitch
            self.lastRoll = roll
            self.lastYaw = yaw
            self.initial_pitch = pitch
            self.initial_roll = roll
            self.initial_yaw = yaw
        twist_msg = Twist()
        pitchDiff = abs(pitch) - abs(self.lastPitch)
        rollDiff = abs(roll) - abs(self.lastRoll)
        yawDiff = abs(yaw) - abs(self.lastYaw)
        if abs(pitchDiff) > abs(rollDiff) and abs(pitchDiff) > abs(yawDiff):
            if pitch > self.initial_pose.orientation.x + self.pitchThreshold:
                # rospy.loginfo("pitching B")
                twist_msg.linear.x = -0.1
            elif pitch < self.initial_pose.orientation.x - self.pitchThreshold:
                # rospy.loginfo("pitching F")
                twist_msg.linear.x = 0.1

        if abs(rollDiff) > pitchDiff and abs(rollDiff) > yawDiff:
            if roll > self.initial_pose.orientation.y + self.rollThreshold:
                # rospy.loginfo("roll r")
                # rospy.loginfo("roll: %f", roll)
                twist_msg.angular.z = 0.1
            elif roll < self.initial_pose.orientation.y - self.rollThreshold:
                # rospy.loginfo("roll l")
                # rospy.loginfo("roll: %f", roll)
                twist_msg.angular.z = -0.1

        if self.drivetype == "omni":
            if abs(yawDiff) > abs(pitchDiff) and abs(yawDiff) > abs(rollDiff):
                if (yaw) > (self.initial_pose.orientation.z) + self.yawThreshold:
                    #rospy.loginfo("yaw r")
                    #rospy.loginfo("yaw: %f", yaw)

                    twist_msg.linear.y = 0.1
                elif (yaw) < (self.initial_pose.orientation.z) - self.yawThreshold:
                    #rospy.loginfo("yaw l")
                    #rospy.loginfo("yaw: %f", yaw)
                    twist_msg.linear.y = -0.1       
     
        ##to stop the tiago from spinning in place when the tracker is pdisconnected
        if pitch == 0 and roll == 0 and yaw == 0:
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            twist_msg.linear.y = 0 


        self.cmd_pub.publish(twist_msg)
        

    def orientation_callback_variable(self, data):
        pitch = data.orientation.x
        yaw = data.orientation.z
        roll = data.orientation.y
        if self.initial_pose is None:
            self.initial_pose = data
            self.lastPitch = pitch
            self.lastRoll = roll
            self.lastYaw = yaw
            self.initial_pitch = pitch
            self.initial_roll = roll
            self.initial_yaw = yaw
        twist_msg = Twist()
        pitchDiff = abs(pitch) - abs(self.lastPitch)
        rollDiff = abs(roll) - abs(self.lastRoll)
        yawDiff = abs(yaw) - abs(self.lastYaw)
        if abs(pitchDiff) > abs(rollDiff) and abs(pitchDiff) > abs(yawDiff):
            if pitch > self.initial_pose.orientation.x + self.pitchThreshold:
                pitch_factor = (pitch - self.initial_pitch) / (self.maxPitch - self.initial_pitch)

                pitch_factor = max(min(pitch_factor, 1.0), 0.0)
                speed = -0.1 + (self.maxPitchSpeed - self.minPitchSpeed) * pitch_factor

                twist_msg.linear.x = -speed
                rospy.loginfo("pitching B")
                rospy.loginfo("pitch_factor: %f", pitch_factor)
                rospy.loginfo("speed: %f", speed)
            elif pitch < self.initial_pose.orientation.x - self.pitchThreshold:
                pitch_factor = (self.initial_pitch - pitch) / (self.initial_pitch - self.minPitch)
                #ensure pitch factor stays within 0 to 1 range
                pitch_factor = max(min(pitch_factor, 1.0), 0.0)
                speed = -0.1 + (self.maxPitchSpeed - self.minPitchSpeed) * pitch_factor

                twist_msg.linear.x = speed
                rospy.loginfo("pitching F")
                rospy.loginfo("pitch_factor: %f", pitch_factor)
                rospy.loginfo("speed: %f", speed)
        rs = self.initial_roll
        if abs(rollDiff) > pitchDiff and abs(rollDiff) > yawDiff:
            if roll > self.initial_pose.orientation.y + self.rollThreshold:
                roll_factor = (roll - rs) / (self.maxLeftRoll - rs) + 0.1
                roll_factor = max(min(roll_factor, 1.0), 0.0)
                speed = (self.maxPitchSpeed - self.minPitchSpeed) * roll_factor
                rospy.loginfo("initial roll: %f", self.initial_roll)
                rospy.loginfo("roll_factor: %f", roll_factor)
                rospy.loginfo("roll l: %f", roll)
                rospy.loginfo("speed: %f", speed)
                twist_msg.angular.z = speed
            elif roll < self.initial_pose.orientation.y - self.rollThreshold:
                roll_factor = (roll - rs) / (self.maxRightRoll - rs)
                roll_factor = max(min(roll_factor, 1.0), 0.0)
                speed = (self.maxPitchSpeed - self.minPitchSpeed) * roll_factor
                rospy.loginfo("initial roll: %f", self.initial_roll)
                rospy.loginfo("roll r %f", roll)
                rospy.loginfo("roll_factor: %f", roll_factor)
                rospy.loginfo("speed: %f", speed)
                twist_msg.angular.z = -speed

        if self.drivetype == "omni":
            if abs(yawDiff) > abs(pitchDiff) and abs(yawDiff) > abs(rollDiff):
                if (yaw) > (self.initial_pose.orientation.z) + self.yawThreshold:
                    yaw_factor = (yaw - self.initial_yaw) / (self.maxLeftYaw - self.initial_yaw)
                    yaw_factor = max(min(yaw_factor, 1.0), 0.0)
                    speed =  -0.1+ (self.maxPitchSpeed - self.minPitchSpeed) * yaw_factor

                    rospy.loginfo("yaw_factor: %f", yaw_factor)
                    rospy.loginfo("yaw r: %f", yaw)
                    rospy.loginfo("speed: %f", speed)
                    twist_msg.linear.y = speed
                elif (yaw) < (self.initial_pose.orientation.z) - self.yawThreshold:
                    yaw_factor = (yaw - self.initial_yaw) / (self.maxRightYaw - self.initial_yaw)
                    yaw_factor = max(min(yaw_factor, 1.0), 0.0)
                    speed =  -0.1 +  (self.maxPitchSpeed - self.minPitchSpeed) * yaw_factor
                    rospy.loginfo("yaw_factor: %f", yaw_factor)
                    rospy.loginfo("yaw l: %f", yaw)
                    rospy.loginfo("speed: %f", speed)
                    twist_msg.linear.y = -speed     
     
        ##to stop the tiago from spinning in place when the tracker is pdisconnected
        if pitch == 0 and roll == 0 and yaw == 0:
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            twist_msg.linear.y = 0 


        self.cmd_pub.publish(twist_msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rudder_vive_tracker', anonymous=True)
    omni_drive = True
    controller = ViveTrackerController(omni_drive)
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
