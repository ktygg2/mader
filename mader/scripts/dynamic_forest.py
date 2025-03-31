#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import random
import roslib
import rospy
import math
from mader_msgs.msg import DynTraj
from snapstack_msgs.msg import Goal, State
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA

from mader_msgs.msg import DynTraj

import numpy as np
from numpy import linalg as LA
import random

# from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from pyquaternion import Quaternion
import tf

from math import sin, cos, tan

import csv
import os
import copy 
import sys

color_static=ColorRGBA(r=0,g=0,b=1,a=1);
color_dynamic=ColorRGBA(r=1,g=0,b=0,a=1);

class MovingForest:
    def __init__(self, total_num_obs):
        print(total_num_obs)
        self.total_num_obs=total_num_obs;
        self.num_of_dyn_objects=int(0.5*total_num_obs) #int(0.65*total_num_obs);
        self.num_of_stat_objects=total_num_obs-self.num_of_dyn_objects; 
        self.x_min= -10     #장애물 배치 범위 조정
        self.x_max= 10.0    #장애물 배치 범위 조정
        self.y_min= -10.0   #장애물 배치 범위 조정
        self.y_max= 10.0    #장애물 배치 범위 조정
        self.z_min= 1.0     # 최소 z 위치 #-6.0 for sphere sim 
        self.z_max= 1.0     # 최대 z 위치 #6.0 for sphere sim
        self.scale=1.0
        self.slower_min= 1.2  #장애물 이동 속도 조정
        self.slower_max= 1.2 #장애물 이동 속도 조정
        self.bbox_dynamic=[0.6, 0.6, 0.6]    #장애물 크기 조정 (동적)
        self.bbox_static_vert=[0.4, 0.4, 4]  #장애물 크기 조정 (세로형 정적) #[0.4, 0.4, 6] for sphere sim
        self.bbox_static_horiz=[0.4, 4, 0.4] #장애물 크기 조정 (가로형 정적)
        self.bbox_cylinder = [0.5, 0.5, 2]  # 장애물 크기 조정 (원기둥)
        self.percentage_vert=0.5;  #0.5 for sphere sim #전체 정적 장애물 중 세로형 장애물의 비율



class FakeSim:

    def __init__(self, total_num_obs):
        self.state=State()

        name = rospy.get_namespace()
        self.name = name[1:-1]

       #self.num_of_objects = 0;
        self.world=MovingForest(total_num_obs)
   
        available_meshes_static=["package://mader/meshes/ConcreteDamage01b/model3.dae", "package://mader/meshes/ConcreteDamage01b/model2.dae"]
        available_meshes_dynamic=["package://mader/meshes/ConcreteDamage01b/model4.dae"]
        # available_meshes=["package://mader/meshes/ConcreteDamage01b/model3.dae"]

        self.pubTraj = rospy.Publisher('/trajs', DynTraj, queue_size=self.world.total_num_obs)
        self.pubShapes_static = rospy.Publisher('/shapes_static', Marker, queue_size=1, latch=True)
        self.pubShapes_static_mesh = rospy.Publisher('/shapes_static_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/shapes_dynamic_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic = rospy.Publisher('/shapes_dynamic', Marker, queue_size=1, latch=True)

        # CSV 파일 경로
        csv_file_path = os.path.expanduser('~/Downloads/forest1.csv')
        self.csv_data = self.read_csv_file(csv_file_path)
        
        self.x_all=[];
        self.y_all=[];
        self.z_all=[];
        self.offset_all=[];
        self.slower=[];
        self.meshes=[];
        self.type=[];#"dynamic" or "static"
        self.bboxes=[]; 
        # CSV 데이터를 활용하여 장애물 설정
        for i, row in enumerate(self.csv_data):
            x, y, z = float(row[0]), float(row[1]), float(row[2])
            width, height, depth = float(row[3]), float(row[4]), float(row[5])

            # 장애물 위치 추가
            self.x_all.append(x)
            self.y_all.append(y)
            self.z_all.append(z)

            # 장애물 크기 추가
            self.bboxes.append([width, height, depth])

            # 동적 또는 정적 장애물 타입 설정 (예: 절반은 동적)
            if i < self.world.num_of_dyn_objects:
                self.type.append("dynamic")
                self.meshes.append(random.choice(available_meshes_dynamic))
                self.offset_all.append(random.uniform(-2 * math.pi, 2 * math.pi))
                self.slower.append(random.uniform(self.world.slower_min, self.world.slower_max))
            else:
                static_type = random.choice(["static_vert", "static_cylinder", "static_horiz"])
                self.type.append(static_type)
                self.meshes.append(random.choice(available_meshes_static))

    def read_csv_file(self, file_path):
        """CSV 파일을 읽어 데이터를 반환"""
        data = []
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                data.append(row)
        return data


    def pubTF(self, timer):

        # rospy.loginfo("[CallbackPy]***************In PUBTF")
        br = tf.TransformBroadcaster()

        marker_tmp=Marker();
        marker_tmp.header.frame_id="world"
        marker_tmp.type=marker_tmp.CUBE_LIST;
        marker_tmp.action=marker_tmp.ADD;

        marker_static=copy.deepcopy(marker_tmp);
        marker_dynamic=copy.deepcopy(marker_tmp);

        marker_dynamic.color=color_dynamic;
        # marker_dynamic.scale.x=self.world.bbox_dynamic[0]
        # marker_dynamic.scale.y=self.world.bbox_dynamic[1]
        # marker_dynamic.scale.z=self.world.bbox_dynamic[2]

        marker_static.color=color_static;


        ###################3
        marker_array_static_mesh=MarkerArray();
        marker_array_dynamic_mesh=MarkerArray();

        for i in range(len(self.bboxes)):
            t_ros=rospy.Time.now()
            t=rospy.get_time(); #Same as before, but it's float

            dynamic_trajectory_msg=DynTraj(); 

            bbox_i=self.bboxes[i];
            s=self.world.scale;
            print("Length of self.bboxes:", len(self.bboxes))
            if self.type[i] == "dynamic":
                [x_string, y_string, z_string] = self.trefoil(self.x_all[i], self.y_all[i], self.z_all[i], s, s, s, self.offset_all[i], self.slower[i])
                marker_dynamic.scale.x = bbox_i[0]
                marker_dynamic.scale.y = bbox_i[1]
                marker_dynamic.scale.z = bbox_i[2]
            elif self.type[i] == "static_vert" or self.type[i] == "static_horiz":
                [x_string, y_string, z_string] = self.static(self.x_all[i], self.y_all[i], self.z_all[i])
                marker_static.scale.x = bbox_i[0]
                marker_static.scale.y = bbox_i[1]
                marker_static.scale.z = bbox_i[2]
            elif self.type[i] == "static_cylinder":
                [x_string, y_string, z_string] = self.static(self.x_all[i], self.y_all[i], self.z_all[i])
                marker = Marker()
                marker.id = i + 1000
                marker.ns = "cylinder"
                marker.header.frame_id = "world"
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.pose.position.x = eval(x_string)
                marker.pose.position.y = eval(y_string)
                marker.pose.position.z = eval(z_string)
                marker.scale.x = bbox_i[0]  # 반지름
                marker.scale.y = bbox_i[0]  # 반지름
                marker.scale.z = bbox_i[2]  # 높이
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                marker_array_static_mesh.markers.append(marker)




            x = eval(x_string)
            y = eval(y_string)
            z = eval(z_string)

            dynamic_trajectory_msg.is_agent=False;
            dynamic_trajectory_msg.header.stamp= t_ros;
            dynamic_trajectory_msg.function = [x_string, y_string, z_string]
            dynamic_trajectory_msg.pos.x=x #Current position
            dynamic_trajectory_msg.pos.y=y #Current position
            dynamic_trajectory_msg.pos.z=z #Current position

            dynamic_trajectory_msg.id = 4000+ i #Current id 4000 to avoid interference with ids from agents #TODO

            self.pubTraj.publish(dynamic_trajectory_msg)
            br.sendTransform((x, y, z), (0,0,0,1), t_ros, self.name+str(dynamic_trajectory_msg.id), "world")


            #If you want to move the objets in gazebo
            # gazebo_state = ModelState()
            # gazebo_state.model_name = str(i)
            # gazebo_state.pose.position.x = x
            # gazebo_state.pose.position.y = y
            # gazebo_state.pose.position.z = z
            # gazebo_state.reference_frame = "world" 
            # self.pubGazeboState.publish(gazebo_state)  

            #If you want to see the objects in rviz
            point=Point()
            point.x=x;
            point.y=y;
            point.z=z;

            ##########################
            marker=Marker();
            marker.id=i;
            marker.ns="mesh";
            marker.header.frame_id="world"
            marker.type=marker.MESH_RESOURCE;
            marker.action=marker.ADD;

            marker.pose.position.x=x
            marker.pose.position.y=y
            marker.pose.position.z=z
            marker.pose.orientation.x=0.0;
            marker.pose.orientation.y=0.0;
            marker.pose.orientation.z=0.0;
            marker.pose.orientation.w=1.0;
            marker.lifetime = rospy.Duration.from_sec(0.0);
            marker.mesh_use_embedded_materials=True
            marker.mesh_resource=self.meshes[i]

            if(self.type[i]=="dynamic"):
                marker_dynamic.points.append(point);

                marker.scale.x=bbox_i[0];
                marker.scale.y=bbox_i[1];
                marker.scale.z=bbox_i[2];

                marker_array_dynamic_mesh.markers.append(marker);


            if(self.type[i]=="static_vert" or self.type[i]=="static_horiz"):

                marker.scale.x=bbox_i[0];
                marker.scale.y=bbox_i[1];
                marker.scale.z=bbox_i[2];
                
                marker_array_static_mesh.markers.append(marker);

                ##########################

                marker_static.points.append(point);

        self.pubShapes_dynamic_mesh.publish(marker_array_dynamic_mesh)
        self.pubShapes_dynamic.publish(marker_dynamic)


        # if(self.already_published_static_shapes==False):

        self.pubShapes_static_mesh.publish(marker_array_static_mesh)
        self.pubShapes_static.publish(marker_static)

        # self.already_published_static_shapes=True;
        self.pubShapes_static = rospy.Publisher('/shapes_static', Marker, queue_size=1, latch=True)
        self.pubShapes_static_mesh = rospy.Publisher('/shapes_static_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/shapes_dynamic_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic = rospy.Publisher('/shapes_dynamic', Marker, queue_size=1, latch=True)


    def static(self,x,y,z):
        return [str(x), str(y), str(z)]

    # Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    def trefoil(self,x,y,z,scale_x, scale_y, scale_z, offset, slower):

        #slower=1.0; #The higher, the slower the obstacles move" #offset을 조정하면 각 장애물의 시작 위치를 다르게 설정 가능
        tt='t/' + str(slower)+'+';

        x_string=str(scale_x)+'*(sin('+tt +str(offset)+') + 2 * sin(2 * '+tt +str(offset)+'))' +'+' + str(x); #'2*sin(t)' 
        y_string=str(scale_y)+'*(cos('+tt +str(offset)+') - 2 * cos(2 * '+tt +str(offset)+'))' +'+' + str(y); #'2*cos(t)' 
        z_string=str(scale_z)+'*(-sin(3 * '+tt +str(offset)+'))' + '+' + str(z);                               #'1.0'        

        # x_string='sin('+tt +str(offset)+')';
        # y_string='cos('+tt +str(offset)+')';
        # z_string='1'

        return [x_string, y_string, z_string]

    def wave_in_z(self,x,y,z,scale, offset, slower): #scale 값을 키우면 장애물의 움직이는 높이 범위가 증가

        tt='t/' + str(slower)+'+';

        x_string=str(x);
        y_string=str(y)
        z_string=str(scale)+'*(-sin( '+tt +str(offset)+'))' + '+' + str(z);                     

        return [x_string, y_string, z_string]




def startNode(total_num_obs):
    c = FakeSim(total_num_obs)
    
    rospy.Timer(rospy.Duration(0.01), c.pubTF)

    rospy.spin()

if __name__ == '__main__':

    # TODO: Use instead https://docs.python.org/3.3/library/argparse.html
    # print("********************************")
    # print(sys.argv)
    # if(len(sys.argv)<=1):
    #     # print("Usage: python dynamic_corridor.py [Num_of_obstacles]")
    #     total_num_obs=140; 
    # else:
    #     total_num_obs=int(sys.argv[1])

    # print("sys.argv[1]= ", sys.argv[1])
    total_num_obs=40 #70 for sphere sim
    ns = rospy.get_namespace()
    try:
        rospy.init_node('dynamic_obstacles')
        startNode(total_num_obs)
    except rospy.ROSInterruptException:
        pass


            # self.x_all.append(random.random());
            # self.y_all.append(4*random.random());
            # self.z_all.append(2);
            # self.x_all.append(50*random.random());
            # self.y_all.append(1*random.random());
            # self.z_all.append(2);

        # self.state.quat.x = 0
        # self.state.quat.y = 0
        # self.state.quat.z = 0
        # self.state.quat.w = 1

        # self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        # self.state.header.frame_id="world"
        # self.pubState.publish(self.state)  