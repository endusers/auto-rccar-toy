#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# The MIT License

# Copyright (c) 2023 Motoyuki Endo
# Released under the MIT license
# http://opensource.org/licenses/mit-license.php

# Copyright (c) 2001-2023 Python Software Foundation; All Rights Reserved

import sys
import threading
from typing import List
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import geographic_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import action_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from nav2_msgs.action import NavigateToPose
import PyKDL
import pandas
import geopandas
from pyproj import Proj
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QFileDialog


CONTROLER_MAIN_CYCLE = 0.1


class RoutePublisher(Node):

    def __init__(self):
        super().__init__( 'route_publisher' )

        self.geopoints : List[geographic_msgs.msg.GeoPoint] = None
        self.utms : List[geometry_msgs.msg.PoseStamped] = None
        self.poses : List[geometry_msgs.msg.PoseStamped] = None
        self.path : nav_msgs.msg.Path = None
        self.utm_frame : string = ''
        self.map_frame : string = ''
        self.reach_range : float = 0.5

        self.declare_parameter( 'utm_frame', 'utm' )
        self.declare_parameter( 'map_frame', 'map' )
        self.declare_parameter( 'reach_range', 0.5 )

        self.utm_frame = self.get_parameter( "utm_frame" ).value
        self.map_frame = self.get_parameter( "map_frame" ).value
        self.reach_range = self.get_parameter( "reach_range" ).value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener( self.tf_buffer, self )

        self.publisher_ = self.create_publisher( nav_msgs.msg.Path, 'route', 10 )

        self.nav2_client = ActionClient( self, NavigateToPose, 'navigate_to_pose' )

        self.timer = self.create_timer( CONTROLER_MAIN_CYCLE, self.mainLoop )

        self.index : int = 0
        self.isRouteValid : bool = False
        self.isGoalAccepted = False
        self.navi_handle = None

    def mainLoop(self):

        # TODO

        pass

    def createWidget(self):
        self.widget = QWidget()
        self.widget.resize( 240, 160 )
        self.widget.setWindowTitle( 'RoutePublisher' )

        self.textbox = QLineEdit( self.widget )
        self.textbox.setText( 'RouteFile.csv' )  
        self.textbox.resize( 220, 20 )
        self.textbox.move( 10, 10 )

        btnFile = QPushButton( self.widget )
        btnFile.setText( 'FileSelect' )
        btnFile.clicked.connect( self.selectRouteFile )
        btnFile.move( 10, 40 )

        btnStart = QPushButton( self.widget )
        btnStart.setText( 'NaviStart' )
        btnStart.clicked.connect(  self.startNavigation )
        btnStart.move( 10, 70 )

        btnStop = QPushButton( self.widget )
        btnStop.setText( 'NaviStop' )
        btnStop.clicked.connect(  self.stopNavigation )
        btnStop.move( 10, 100 )

        self.lblStatus = QLabel( self.widget )
        self.lblStatus.setText( '' )
        self.lblStatus.resize( 220, 20 )
        self.lblStatus.move( 10, 130 )
        self.lblStatus.setStyleSheet( 'border:1px solid gray' )

    def selectRouteFile(self):
        self.geopoints = list()
        self.utms = list()

        self.isRouteValid = False

        file,check = QFileDialog.getOpenFileName(
            None,
            'Please select a file.',
            '',
            'All Files (*);;Csv Files (*.csv)'
        )

        if check:
            self.textbox.setText( file )

            df = pandas.read_csv( file )
            wkts = geopandas.GeoSeries.from_wkt( df.WKT )

        if len(wkts) >= 2:
            for wkt in wkts:
                utm_zone = int( divmod( wkt.x, 6 )[0] ) + 31
                utm_conv = Proj( proj='utm', zone=utm_zone, ellps='WGS84' )
                utmx, utmy = utm_conv( wkt.x, wkt.y )
                if wkt.y < 0:
                    utmy = utmy + 10000000

                geopoint = geographic_msgs.msg.GeoPoint()
                geopoint.latitude = wkt.x
                geopoint.longitude = wkt.y
                geopoint.altitude = 0.0
                self.geopoints.append( geopoint )

                pose = geometry_msgs.msg.PoseStamped()
                pose.header.frame_id = self.utm_frame
                pose.pose.position.x = utmx
                pose.pose.position.y = utmy
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                self.utms.append( pose )

                self.get_logger().debug( 'Datum (latitude, longitude, altitude) is (%0.2f, %0.2f, %0.2f)'
                    % (geopoint.latitude, geopoint.longitude, geopoint.altitude)
                )
                self.get_logger().debug( 'Datum UTM coordinate is (%02d, %0.2f, %0.2f)'
                    % (utm_zone, pose.pose.position.x, pose.pose.position.y)
                )

            i = 0
            while i < len(self.utms) - 1:
                dx = self.utms[i + 1].pose.position.x - self.utms[i].pose.position.x
                dy = self.utms[i + 1].pose.position.y - self.utms[i].pose.position.y
                theta = math.atan2(dy, dx)
                q = PyKDL.Rotation.RPY( 0.0, 0.0, theta ).GetQuaternion()

                if i == 0:
                    self.utms[i].pose.orientation.x = q[0]
                    self.utms[i].pose.orientation.x = q[1]
                    self.utms[i].pose.orientation.z = q[2]
                    self.utms[i].pose.orientation.w = q[3]

                self.utms[i + 1].pose.orientation.x = q[0]
                self.utms[i + 1].pose.orientation.x = q[1]
                self.utms[i + 1].pose.orientation.z = q[2]
                self.utms[i + 1].pose.orientation.w = q[3]

                i = i + 1

            self.get_logger().info( f'Route is {file}' )

            self.isRouteValid = True

    def startNavigation(self):
        self.poses = list()
        self.path = None

        if self.isRouteValid:
            # for foxy (for geometry_msgs/Pose)
            utms : List[geometry_msgs.msg.Pose] = None
            utms = list()
            for utm in self.utms:
                pose = utm.pose
                utms.append( pose )

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.utm_frame, rclpy.time.Time(),
                    timeout=rclpy.time.Duration(seconds=10)
                )

                # for geometry_msgs/Pose
                for utm in utms:
                    pose = tf2_geometry_msgs.do_transform_pose( utm, transform )
                    posestamp = geometry_msgs.msg.PoseStamped()
                    posestamp.pose = pose
                    self.poses.append( posestamp )

                # for geometry_msgs/PoseStamped
                #for utm in self.utms:
                #    posestamp = tf2_geometry_msgs.do_transform_pose( utm, transform )
                #    self.poses.append( posestamp )

                self.path = nav_msgs.msg.Path()
                self.path.header.stamp = self.get_clock().now().to_msg()
                self.path.header.frame_id = self.map_frame
                self.path.poses = self.poses.copy()
                for pose in self.path.poses:
                    pose.header.stamp = self.path.header.stamp
                    pose.header.frame_id = self.map_frame

                self.publisher_.publish( self.path )

                self.index = 0
                self.isGoalAccepted = False
                self.navi_handle = None
                self.sendGoal( self.path.poses[self.index] )

            except tf2_ros.LookupException as e:
                self.get_logger().error( f'Failed to get transform {repr(e)}\n' )

        else:
            self.lblStatus.setText( 'Invalid route' )

    def stopNavigation(self):
        if self.navi_handle is not None:
            self.navi_handle.cancel_goal_async()

    def sendGoal( self, posestamp ):

        goal = NavigateToPose.Goal()
        goal.pose = posestamp

        self.get_logger().debug( 'Connecting for action server' )
        while not self.nav2_client.wait_for_server( timeout_sec=1.0 ):
            self.get_logger().debug( 'Waiting for action server' )
        self.get_logger().debug( 'Connected for action server' )

        navi_response = self.nav2_client.send_goal_async( goal, feedback_callback=self.feedbackCallback )
        navi_response.add_done_callback( self.responseCallback )

    def feedbackCallback( self, msg ):
        if self.isGoalAccepted and msg.feedback.distance_remaining < self.reach_range:
            if self.index + 1 < len(self.path.poses):
                self.index = self.index + 1
                self.isGoalAccepted = False
                self.sendGoal( self.path.poses[self.index] )

    def responseCallback( self, future ):
        navi_handle = future.result()

        if not navi_handle.accepted:
            str = f'Navigate point rejected {self.index + 1}/{len(self.path.poses)}'
            self.lblStatus.setText( str )
            self.get_logger().info( str )
            return

        self.navi_handle = navi_handle

        self.isGoalAccepted = True
        str = f'Navigating point {self.index + 1}/{len(self.path.poses)}'
        self.lblStatus.setText( str )
        self.get_logger().info( str )

        navi_result = self.navi_handle.get_result_async()
        navi_result.add_done_callback( self.resultCallback )

    def resultCallback( self, future ):
        #status = future.result().status
        status = self.navi_handle.status

        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            if self.index >= len(self.path.poses) - 1:
                self.navi_handle = None
                str = 'Navigate succeeded!'
                self.lblStatus.setText( str )
                self.get_logger().info( str )
        elif status == action_msgs.msg.GoalStatus.STATUS_CANCELED:
            self.navi_handle = None
            str = 'Navigate canceled!'
            self.lblStatus.setText( str )
            self.get_logger().info( str )
        elif status == action_msgs.msg.GoalStatus.STATUS_ABORTED:
            self.navi_handle = None
            str = 'Navigate aborted!'
            self.lblStatus.setText( str )
            self.get_logger().info( str )
        else:
            pass
