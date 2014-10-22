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

/** \file PurePursuitControllerNode.h
    \brief This file defines the PurePursuitControllerNode class which
           implements a pure pursuit controller.
  */

#ifndef PURE_PURSUIT_CONTROLLER_NODE_H
#define PURE_PURSUIT_CONTROLLER_NODE_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>

namespace starleth {

  /** The class PurePursuitControllerNode implements a pure pursuit controller.
      \brief Pure pursuit controller
    */
  class PurePursuitControllerNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    PurePursuitControllerNode(const ros::NodeHandle& nh);
    /// Copy constructor
    PurePursuitControllerNode(const PurePursuitControllerNode& other) = delete;
    /// Copy assignment operator
    PurePursuitControllerNode& operator =
      (const PurePursuitControllerNode& other) = delete;
    /// Move constructor
    PurePursuitControllerNode(PurePursuitControllerNode&& other) = delete;
    /// Move assignment operator
    PurePursuitControllerNode& operator =
      (PurePursuitControllerNode&& other) = delete;
    /// Destructor
    virtual ~PurePursuitControllerNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin once
    void spin();
    /// Step once
    bool step(geometry_msgs::Twist& twist);
    /// Returns the current pose of the robot
    geometry_msgs::PoseStamped getCurrentPose() const;
    /// Returns the lookahead distance for the given pose
    double getLookAheadDistance(const geometry_msgs::PoseStamped& pose) const;
    /// Returns the lookahead angle for the given pose in [rad]
    double getLookAheadAngle(const geometry_msgs::PoseStamped& pose) const;
    /// Returns the current lookahead distance threshold
    double getLookAheadThreshold() const;
    /// Returns the lookahead distance for the given pose
    double getArcDistance(const geometry_msgs::PoseStamped& pose) const;
    /// Returns the next way point by linear search from the current waypoint
    int getNextWayPoint(int wayPoint) const;    
    /// Returns the current closest waypoint
    int getClosestWayPoint() const;    
    /// Returns the interpolated pose based on the given way point
    bool getInterpolatedPose(int wayPoint, geometry_msgs::PoseStamped&
      interpolatedPose) const;    
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Retrieves parameters
    void getParameters();
    /// Path message callback
    void pathCallback(const nav_msgs::Path& msg);
    /// Odometry message callback
    void odometryCallback(const nav_msgs::Odometry& msg);
    /// Timer callback
    void timerCallback(const ros::TimerEvent& event);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Path message subscriber
    ros::Subscriber _pathSubscriber;
    /// Path message topic name
    std::string _pathTopicName;
    /// Odometry message subscriber
    ros::Subscriber _odometrySubscriber;
    /// Odometry message topic name
    std::string _odometryTopicName;
    /// Frame id of pose estimates
    std::string _poseFrameId;
    /// Queue size for receiving messages
    int _queueDepth;
    /// Current reference path
    nav_msgs::Path _currentReferencePath;
    /// Current velocity
    geometry_msgs::Twist _currentVelocity;
    /// Controller frequency
    double _frequency;
    /// Next way point
    int _nextWayPoint;
    /// Commanded velocity publisher
    ros::Publisher _cmdVelocityPublisher;
    /// Commanded velocity topic name
    std::string _cmdVelocityTopicName;
    /// Commanded trajectory publisher
    ros::Publisher _cmdTrajectoryPublisher;
    /// Commanded trajectory topic name
    std::string _cmdTrajectoryTopicName;
    /// Initial way point
    int _initialWayPoint;
    /// Velocity
    double _velocity;
    /// Lookahead ratio
    double _lookAheadRatio;
    /// Epsilon
    double _epsilon;
    /// Transform listener for robot's pose w.r.t. map
    tf::TransformListener tfListener_;
    /// Timer
    ros::Timer _timer;
    /** @}
      */

  };

}

#endif // PURE_PURSUIT_CONTROLLER_NODE_H
