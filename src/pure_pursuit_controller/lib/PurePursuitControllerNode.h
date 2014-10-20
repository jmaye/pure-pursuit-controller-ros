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

#include <geometry_msgs/PoseStamped.h>

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
    /// Returns the lookahead distance for the given position
    double getLookAdeadDistance(const geometry_msgs::Point& position) const;
    /// Returns the lookahead angle for the given position in [rad]
    double getLookAdeadAngle(const geometry_msgs::Point& position) const;
    /// Returns the current lookahead distance threshold
    double getLookAdeadThreshold() const;
    /// Returns the next waypoint 
    double getLookAdeadThreshold() const;
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Retrieves parameters
    void getParameters();
    /// Path message callback
    void pathMsgCallback(const nav_msgs::PathConstPtr& msg);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Path message subscriber
    ros::Subscriber _pathMsgSubscriber;
    /// Path message topic name
    std::string _pathMsgTopicName;
    /// Queue size for receiving messages
    int _queueDepth;
    /// Current reference path
    std::vector<geometry_msgs::PoseStamped> _currentReferencePath;
    /// Next way point
    int _nextWayPoint;
    /// Commanded velocity publisher
    ros::Publisher _cmdVelocityPublisher;
    /// Commanded velocity topic name
    std::string _cmdVelocityTopicName;
    /// Initial way point
    int _initialWayPoint;
    /// Velocity
    double _velocity;
    /// Lookahead ratio
    double _lookAheadRatio;
    /// Epsilon
    double _epsilon;
    /// Transform listener for robot's pose w.r.t. map
    tf::TransformListener _tfListener;
    /** @}
      */

  };

}

#endif // PURE_PURSUIT_CONTROLLER_NODE_H
