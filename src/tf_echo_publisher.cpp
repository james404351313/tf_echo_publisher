#include <cstdio>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <iostream>

#define _USE_MATH_DEFINES

bool map_receive=false;
bool laser_receive1=false;
bool receive=false;
geometry_msgs::Pose2D Pose2D;
nav_msgs::Path path;
nav_msgs::Path path_lsm;
geometry_msgs::PoseStamped pose_lsm;
geometry_msgs::PoseStamped pose;
geometry_msgs::PointStamped point_lsm;
geometry_msgs::PointStamped point;
nav_msgs::Odometry odometry;

void tf_callback(const tf::tfMessageConstPtr& tf)
{
 std::basic_string <char> map_frame_id="map";
 std::basic_string <char> laser_frame_id="odom";
 if(map_frame_id==tf->transforms[0].header.frame_id)
 {
   map_receive=true;
 }
 if(laser_frame_id==tf->transforms[0].header.frame_id)
 {
   laser_receive1=true;
 }
 if(map_receive && laser_receive1)
 {
   receive=true;
 }

}

void pose_callback(const geometry_msgs::Pose2DConstPtr& pose2D)
{

  float pose2D_x=pose2D->x;
  float pose2D_y=pose2D->y;
  float pose2D_theta=pose2D->theta;

  pose_lsm.pose.position.x=pose2D_x;
  pose_lsm.pose.position.y=pose2D_y;
  pose_lsm.pose.position.z=0;
  pose_lsm.pose.orientation.x=0;
  pose_lsm.pose.orientation.y=0;
  pose_lsm.pose.orientation.z=sin(pose2D_theta/2);
  pose_lsm.pose.orientation.w=cos(pose2D_theta/2);
  path_lsm.poses.push_back(pose_lsm);
  path_lsm.header.stamp = ros::Time::now();

  point_lsm.point.x=pose2D_x;
  point_lsm.point.y=pose2D_y;
  point_lsm.point.z=0;
  point_lsm.header.stamp= ros::Time::now();

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "position_publish", ros::init_options::AnonymousName);
  ros::NodeHandle tf_echo_publisher;
  ros::Publisher transform_and_theta=tf_echo_publisher.advertise<geometry_msgs::Pose2D>("/tf_echo_pose2D",10);
  ros::Publisher tf_echo_path_publisher=tf_echo_publisher.advertise<nav_msgs::Path>("/path",10);
  ros::Publisher pointstamped_publisher=tf_echo_publisher.advertise<geometry_msgs::PointStamped>("/pointstamped",10);
  ros::Publisher odometry_publisher=tf_echo_publisher.advertise<nav_msgs::Odometry>("/odometry",10);
  ros::Publisher point_lsm_publisher=tf_echo_publisher.advertise<geometry_msgs::PointStamped>("/point_lsm",10);
  ros::Publisher path_lsm_publisher=tf_echo_publisher.advertise<nav_msgs::Path>("/path_lsm",10);
  ros::Subscriber tf_subscriber=tf_echo_publisher.subscribe("/tf",10,tf_callback);
  ros::Subscriber pose2D_subscriber=tf_echo_publisher.subscribe( "/pose2D" , 10 , pose_callback);

  path.header.frame_id="map";
  point.header.frame_id="map";
  odometry.header.frame_id="map";
  point_lsm.header.frame_id="map";
  path_lsm.header.frame_id="map";

  double rate_hz=100;
  std::string source_frameid="map";
  std::string target_frameid="laser";
  ros::Rate rate(rate_hz);

  //Instantiate a local listener
  tf::TransformListener tf_listener;

  // Wait for up to one second for the first transforms to become avaiable.
//  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
  tf_listener.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(tf_echo_publisher.ok())
    {
        if(receive)
        {
            tf::StampedTransform echo_transform;
            tf_listener.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
            double yaw, pitch, roll;
            echo_transform.getBasis().getRPY(roll, pitch, yaw);
            tf::Quaternion q = echo_transform.getRotation();
            tf::Vector3 v = echo_transform.getOrigin();
            Pose2D.x=1000*v.getX();
            Pose2D.y=1000*v.getY();
            Pose2D.theta=yaw*180.0/M_PI;

            point.point.x=v.getX();
            point.point.y=v.getY();
            point.point.z=0;

            odometry.pose.pose.position.x=v.getX();
            odometry.pose.pose.position.y=v.getY();
            odometry.pose.pose.position.z=0;
            odometry.pose.pose.orientation.x=0;
            odometry.pose.pose.orientation.y=0;
            odometry.pose.pose.orientation.z=q.getZ();
            odometry.pose.pose.orientation.w=q.getW();

            pose.pose.position.x=v.getX();
            pose.pose.position.y=v.getY();
            pose.pose.position.z=0;
            pose.pose.orientation.x=0;
            pose.pose.orientation.y=0;
            pose.pose.orientation.z=q.getZ();
            pose.pose.orientation.w=q.getW();
            path.poses.push_back(pose);

            transform_and_theta.publish(Pose2D);
            tf_echo_path_publisher.publish(path);
            pointstamped_publisher.publish(point);
            odometry_publisher.publish(odometry);
            receive=false;
            map_receive=false;
            laser_receive1=false;
        }
        point_lsm_publisher.publish(point_lsm);
        path_lsm_publisher.publish(path_lsm);
        ros::spinOnce();
        rate.sleep();
    }
}
