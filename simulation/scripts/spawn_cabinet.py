#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler


sdff="""
<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.7">
    <model name="cabinet_with_drawer">
    <pose>0 0 0 0 0 0</pose>


        <link name="cabinet">
            <pose>0 0 0 0 0 0</pose>
            <inertial>
              <inertia>
                <ixx>2.501</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>2.501</iyy>
                <iyz>0</iyz>
                <izz>5</izz>
              </inertia>
              <mass>120.0</mass>
          </inertial>
            <visual name="cabinet_visual">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/cabinet.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </visual>
            <collision name="cabinet_collision">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/cabinet.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </collision>
        </link>

        <joint name="world_fixed" type="fixed">
        <parent>world</parent>
        <child>cabinet</child>
        </joint>

        <link name="drawer1">
            <pose>0.013 -0.23 0.9 0 0 0</pose>
            <inertial>
              <inertia>
                <ixx>0.17689519</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.22160905</iyy>
                <iyz>0</iyz>
                <izz>0.35997466</izz>
              </inertia>
              <mass>3.0</mass>
          </inertial>
            <visual name="drawer_visual1">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/drawer.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </visual>
            <collision name="drawer_collision1">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/drawer.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </collision>
        </link>



        <joint name="drawer_joint1" type="prismatic">
        <child>drawer1</child>
        <parent>cabinet</parent>
        <axis>
          <limit>
            <lower>-0.6</lower>
            <upper>0</upper>
          </limit>
          <xyz>0 1 0</xyz>
        </axis>
       </joint>

        <link name="drawer2">
            <pose>0.013 -0.23 0.6 0 0 0</pose>
            <inertial>
              <inertia>
                <ixx>0.17689519</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.22160905</iyy>
                <iyz>0</iyz>
                <izz>0.35997466</izz>
              </inertia>
              <mass>3.0</mass>
          </inertial>
            <visual name="drawer_visual2">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/drawer.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </visual>
            <collision name="drawer_collision2">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/drawer.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </collision>
        </link>



        <joint name="drawer_joint2" type="prismatic">
        <child>drawer2</child>
        <parent>cabinet</parent>
        <axis>
          <limit>
            <lower>-0.6</lower>
            <upper>0</upper>
          </limit>
          <xyz>0 1 0</xyz>
        </axis>
       </joint>

        <link name="drawer3">
            <pose>0.013 -0.23 0.3 0 0 0</pose>
            <inertial>
              <inertia>
                <ixx>0.17689519</ixx>
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyy>0.22160905</iyy>
                <iyz>0</iyz>
                <izz>0.35997466</izz>
              </inertia>
              <mass>3.0</mass>
          </inertial>
            <visual name="drawer_visual3">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/drawer.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </visual>
            <collision name="drawer_collision3">
                <geometry>
                    <mesh>
                     <uri>model://cabinet_drawer/meshes/drawer.dae</uri>
                     <scale>1 1 1</scale>
                    </mesh>
                </geometry>
            </collision>
        </link>



        <joint name="drawer_joint3" type="prismatic">
        <child>drawer3</child>
        <parent>cabinet</parent>
        <axis>
          <limit>
            <lower>-0.6</lower>
            <upper>0</upper>
          </limit>
          <xyz>0 1 0</xyz>
        </axis>
       </joint>

    </model>
</sdf>
"""


rospy.init_node('insert_object',log_level=rospy.INFO)


position = [0,0,0]
orientation = [0,0,0]
cabinet_pose = Pose(Point(*position), Quaternion(*quaternion_from_euler(*orientation)))
rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("Cabinet", sdff, "", cabinet_pose, "world")
