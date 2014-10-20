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

#include <geometry_msgs/Twist.h>

namespace starleth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  PurePursuitControllerNode::PurePursuitControllerNode(const
      ros::NodeHandle& nh) :
      _nodeHandle(nh),
      _nextWayPoint(-1) {
    getParameters();
    _pathMsgSubscriber =
      _nodeHandle.subscribe(_pathMsgTopicName, _queueDepth,
      &PurePursuitControllerNode::pathMsgCallback, this);
    _cmdVelocityPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(
      _cmdVelocityTopicName, _queueDepth);
  }

  PurePursuitControllerNode::~PurePursuitControllerNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void PurePursuitControllerNode::pathMsgCallback(const nav_msgs::PathConstPtr&
      msg) {
    _currentReferencePath = msg->poses;
    _nextWayPoint = -1;
  }

  void PurePursuitControllerNode::spin() {
    ros::spin();
  }

  double getLookAdeadAngle(const geometry_msgs::Point& position) const {
  }
  
  double getLookAdeadThreshold() const {
  }

  double getLookAdeadDistance(const geometry_msgs::Point& position) const {
    return 0.0;
  }

  void PurePursuitControllerNode::getParameters() {
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<std::string>("ros/path_msg_topic_name", _pathMsgTopicName,
      "/reference_path");
    _nodeHandle.param<std::string>("ros/cmd_velocity_topic_name",
      _cmdVelocityTopicName, "/starleth/command_velocity");
    _nodeHandle.param<int>("controller/initial_waypoint", _initialWayPoint, -1);
    _nodeHandle.param<double>("controller/velocity", _velocity, 0.2);
    _nodeHandle.param<double>("controller/look_ahead_ratio",
      _lookAheadRatio, 1.0);
    _nodeHandle.param<double>("controller/epsilon", _epsilon, 1e-6);
  }

}
