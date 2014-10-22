/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "PurePursuitControllerNode.h"

#include <cmath>

#include <geometry_msgs/Twist.h>

#include <visualization_msgs/Marker.h>

namespace starleth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  PurePursuitControllerNode::PurePursuitControllerNode(const
      ros::NodeHandle& nh) :
      _nodeHandle(nh),
      _nextWayPoint(-1) {
    getParameters();
    
    _pathSubscriber =
      _nodeHandle.subscribe(_pathTopicName, _queueDepth,
      &PurePursuitControllerNode::pathCallback, this);
    _odometrySubscriber =
      _nodeHandle.subscribe(_odometryTopicName, _queueDepth,
      &PurePursuitControllerNode::odometryCallback, this);
      
    _cmdVelocityPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(
      _cmdVelocityTopicName, _queueDepth);
    _cmdTrajectoryPublisher =
      _nodeHandle.advertise<visualization_msgs::Marker>(
      _cmdTrajectoryTopicName, _queueDepth);
    
    _timer = nh.createTimer(ros::Duration(1.0/_frequency),
      &PurePursuitControllerNode::timerCallback, this);
  }

  PurePursuitControllerNode::~PurePursuitControllerNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  geometry_msgs::PoseStamped PurePursuitControllerNode::getCurrentPose()
      const {
    geometry_msgs::PoseStamped pose, transformedPose;
    pose.header.frame_id = _poseFrameId;
        
    try {
      _tfListener.transformPose(_currentReferencePath.header.frame_id,
        pose, transformedPose);
    }
    catch (tf::TransformException& exception) {
      ROS_ERROR_STREAM("PurePursuitControllerNode::getCurrentPose: " << 
        exception.what());
    }
    
    return transformedPose;
  }

  void PurePursuitControllerNode::pathCallback(const nav_msgs::Path& msg) {
    _currentReferencePath = msg;
    _nextWayPoint = -1;
  }

  void PurePursuitControllerNode::odometryCallback(const nav_msgs::Odometry&
      msg) {
    _currentVelocity = msg.twist.twist;
  }

  void PurePursuitControllerNode::spin() {
    ros::spin();
  }

  void PurePursuitControllerNode::timerCallback(const ros::TimerEvent&
      event) {
    geometry_msgs::Twist cmdVelocity;

    if (step(cmdVelocity)) {
      const size_t numPoints = 20;
      
      double lookAheadThreshold = getLookAheadThreshold();
      visualization_msgs::Marker cmdTrajectory;
      
      cmdTrajectory.header.frame_id = _poseFrameId;
      cmdTrajectory.header.stamp = ros::Time::now();
      cmdTrajectory.ns = "solution_trajectory";
      cmdTrajectory.type = 4;
      cmdTrajectory.action = 0;
      cmdTrajectory.scale.x = 0.12;
      cmdTrajectory.color.r = 0.0;
      cmdTrajectory.color.g = 0.0;
      cmdTrajectory.color.b = 1.0;
      cmdTrajectory.color.a = 1.0;
      cmdTrajectory.lifetime = ros::Duration(0);
      cmdTrajectory.frame_locked = true;
      cmdTrajectory.pose = geometry_msgs::Pose();
      cmdTrajectory.points.resize(numPoints);      
      
      for (int i = 0; i < numPoints; ++i) {
        geometry_msgs::Pose pose;
        double dt = lookAheadThreshold*(double)i/(double)numPoints;
        
        pose.orientation.z = cmdVelocity.angular.x*dt;
        pose.position.x = cmdVelocity.linear.x*std::cos(
          pose.orientation.z)*dt;
        pose.position.y = cmdVelocity.linear.x*std::sin(
          pose.orientation.z)*dt;
        
        cmdTrajectory.points[i] = pose.position;
      }
    }
  }
    
  bool PurePursuitControllerNode::step(geometry_msgs::Twist& twist) {
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    
    _nextWayPoint = getNextWayPoint(_nextWayPoint);

    if (_nextWayPoint >= 0) {
      geometry_msgs::PoseStamped pose;
      
      if (getInterpolatedPose(_nextWayPoint, pose)) {
        double lookAheadDistance = getLookAheadDistance(pose);
        double lookAheadAngle = getLookAheadAngle(pose);

        double angularVelocity = 0.0;
        if (std::abs(std::sin(lookAheadAngle)) >= _epsilon) {
          double radius = 0.5*(lookAheadDistance/std::sin(lookAheadAngle));
          
          double linearVelocity = _velocity;
          if (std::abs(radius) >= _epsilon)
            angularVelocity = linearVelocity/radius;

          twist.linear.x = linearVelocity;
          twist.angular.z = angularVelocity;
          
          return true;
        }
      }
    }
    
    return false;
  }
  
  double PurePursuitControllerNode::getLookAheadDistance(const
      geometry_msgs::PoseStamped& pose) const {
    geometry_msgs::PoseStamped origin = getCurrentPose();
    geometry_msgs::PoseStamped transformedPose;
    
    try {
      _tfListener.transformPose(_currentReferencePath.header.frame_id,
        pose, transformedPose);
    }
    catch (tf::TransformException& exception) {
      ROS_ERROR_STREAM("PurePursuitControllerNode::getLookAheadDistance: " << 
        exception.what());
      
      return -1.0;
    }
    
    tf::Vector3 v1(origin.pose.position.x,
                   origin.pose.position.y,
                   origin.pose.position.z);
    tf::Vector3 v2(transformedPose.pose.position.x,
                   transformedPose.pose.position.y,
                   transformedPose.pose.position.z);
    
    return tf::tfDistance(v1, v2);
  }
  
  double PurePursuitControllerNode::getLookAheadAngle(const
      geometry_msgs::PoseStamped& pose) const {
    geometry_msgs::PoseStamped origin = getCurrentPose();
    geometry_msgs::PoseStamped transformedPose;
    
    try {
      _tfListener.transformPose(_currentReferencePath.header.frame_id,
        pose, transformedPose);
    }
    catch (tf::TransformException& exception) {
      ROS_ERROR_STREAM("PurePursuitControllerNode::getLookAheadDistance: " << 
        exception.what());
      
      return -1.0;
    }
    
    tf::Vector3 v1(origin.pose.position.x,
                   origin.pose.position.y,
                   origin.pose.position.z);
    tf::Vector3 v2(transformedPose.pose.position.x,
                   transformedPose.pose.position.y,
                   transformedPose.pose.position.z);
    
    return tf::tfAngle(v1, v2);
  }
  
  double PurePursuitControllerNode::getLookAheadThreshold() const {
    return _lookAheadRatio*_currentVelocity.linear.x;
  }

  double PurePursuitControllerNode::getArcDistance(const
      geometry_msgs::PoseStamped& pose) const {
    double lookAheadDistance = getLookAheadDistance(pose);
    double lookAheadAngle = getLookAheadAngle(pose);

    if (std::abs(std::sin(lookAheadAngle)) >= _epsilon)
      return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle;
    else
      return lookAheadDistance;
  }

  int PurePursuitControllerNode::getNextWayPoint(int wayPoint) const {   
    if (!_currentReferencePath.poses.empty()) {
      if (_nextWayPoint >= 0) {
        geometry_msgs::PoseStamped origin = getCurrentPose();
        tf::Vector3 v_1(origin.pose.position.x,
                        origin.pose.position.y,
                        origin.pose.position.z);
        double lookAheadThreshold = getLookAheadThreshold();

        for (int i = _nextWayPoint; i < _currentReferencePath.poses.size();
            ++i) {                
          tf::Vector3 v_2(_currentReferencePath.poses[i].pose.position.x,
                          _currentReferencePath.poses[i].pose.position.y,
                          _currentReferencePath.poses[i].pose.position.z);
          
          if (tf::tfDistance(v_1, v_2) > lookAheadThreshold)
            return i;
        }

        return _nextWayPoint;
      }
      else
        return 0;
    }

    return -1;
  }
  
  int PurePursuitControllerNode::getClosestWayPoint() const {
    if (!_currentReferencePath.poses.empty()) {
      int closestWaypoint = -1;
      double minDistance = -1.0;
      
      for (int i = 0; i < _currentReferencePath.poses.size(); ++i) {
        double distance = getArcDistance(_currentReferencePath.poses[i]);
        
        if ((minDistance < 0.0) || (distance < minDistance)) {
          closestWaypoint = i;
          minDistance = distance;
        }
      }
      
      return closestWaypoint;
    }

    return -1;
  }
  
  bool PurePursuitControllerNode::getInterpolatedPose(int wayPoint,
      geometry_msgs::PoseStamped& interpolatedPose) const {
    if (!_currentReferencePath.poses.empty()) {
      if (wayPoint > 0) {
        double l_t = getLookAheadThreshold();
        double p_t = getLookAheadDistance(
          _currentReferencePath.poses[_nextWayPoint-1]);
        
        if (p_t < l_t) {
          geometry_msgs::PoseStamped p_0 = getCurrentPose();
          geometry_msgs::PoseStamped p_1 = 
            _currentReferencePath.poses[wayPoint-1];
          geometry_msgs::PoseStamped p_2 = 
            _currentReferencePath.poses[wayPoint];
          
          tf::Vector3 v_1(p_2.pose.position.x-p_0.pose.position.x,
                          p_2.pose.position.y-p_0.pose.position.y,
                          p_2.pose.position.z-p_0.pose.position.z);
          tf::Vector3 v_2(p_1.pose.position.x-p_0.pose.position.x,
                          p_1.pose.position.y-p_0.pose.position.y,
                          p_1.pose.position.z-p_0.pose.position.z);
          tf::Vector3 v_0(p_2.pose.position.x-p_1.pose.position.x,
                          p_2.pose.position.y-p_1.pose.position.y,
                          p_2.pose.position.z-p_1.pose.position.z);
          
          double l_0 = v_0.length();
          double l_1 = v_1.length();
          double l_2 = v_2.length();
          
          v_0.normalize();
          v_2.normalize();
          
          double alpha_1 = M_PI-tf::tfAngle(v_0, v_2);
          double beta_2 = asin(l_2*sin(alpha_1)/l_t);
          double beta_0 = M_PI-alpha_1-beta_2;
          double l_s = l_2*sin(beta_0)/sin(beta_2);
          tf::Vector3 p_s(p_1.pose.position.x+v_0[0]*l_s,
                          p_1.pose.position.x+v_0[1]*l_s,
                          p_1.pose.position.x+v_0[2]*l_s);

          interpolatedPose.pose.position.x = p_s[0];
          interpolatedPose.pose.position.y = p_s[1];
          interpolatedPose.pose.position.z = p_s[2];
          
          return true;
        }
      }

      interpolatedPose = _currentReferencePath.poses[wayPoint];
      return true;
    }
      
    return false;
  }
      
  void PurePursuitControllerNode::getParameters() {
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<std::string>("ros/path_topic_name", _pathTopicName,
      "/reference_path");
    _nodeHandle.param<std::string>("ros/odometry_topic_name",
      _odometryTopicName, "/starleth/robot_state/odometry");
    _nodeHandle.param<std::string>("ros/cmd_velocity_topic_name",
      _cmdVelocityTopicName, "/starleth/command_velocity");
    _nodeHandle.param<std::string>("ros/cmd_trajectory_topic_name",
      _cmdTrajectoryTopicName, "/local_planner_solution_trajectory");
    _nodeHandle.param<std::string>("ros/pose_frame_id", _poseFrameId, "base");
    
    _nodeHandle.param<double>("controller/frequency", _frequency, 20.0);
    _nodeHandle.param<int>("controller/initial_waypoint", _initialWayPoint, -1);
    _nodeHandle.param<double>("controller/velocity", _velocity, 0.2);
    _nodeHandle.param<double>("controller/look_ahead_ratio",
      _lookAheadRatio, 1.0);
    _nodeHandle.param<double>("controller/epsilon", _epsilon, 1e-6);
  }

}
