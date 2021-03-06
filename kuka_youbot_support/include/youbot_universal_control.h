/******************************************************************************
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov, Yury Brodskiy
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of GPS GmbH and University of Twente nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/


#ifndef JOINT_POSITION_CONTROLLER_H
#define JOINT_POSITION_CONTROLLER_H

#include <vector>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <pr2_controller_interface/controller.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointTorques.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include "joint_state_observer_gazebo.h"

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class JointTrajectoryAction;
//class JointStateObserverGazebo;

namespace controller {

    class YouBotUniversalController : public pr2_controller_interface::Controller {
    public:

        enum ControlMode {
            POSITION,
            VELOCITY,
            TORQUE
        };

        ControlMode currentControlMode;

        YouBotUniversalController();
        ~YouBotUniversalController();

        bool init(pr2_mechanism_model::RobotState *robotPtr, const std::string &jointName, const control_toolbox::Pid &pid);
        bool init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle);
        void starting();
        void update();

    private:
        ros::NodeHandle nodeHandle;
        pr2_mechanism_model::RobotState *robotPtr;
        std::vector<pr2_mechanism_model::JointState*> joints;
        std::vector<control_toolbox::Pid> pids;
        ros::Time lastTime;
        std::vector <double> setPoints;
        std::vector <double> filteredVelocity;

        ros::Subscriber positionCommandSubscriber;
        ros::Subscriber velocityCommandSubscriber;
        ros::Subscriber torqueCommandSubscriber;
        JointTrajectoryAction* jointTrajectoryAction;
        Server* trajectoryServer;

        void pr2JointStateToJointStateMsgs(const std::vector<pr2_mechanism_model::JointState*>& pr2_joint_states, sensor_msgs::JointState& joint_states);
        void updateJointPosition(double setPoint, pr2_mechanism_model::JointState* joint_state_, control_toolbox::Pid* pid_controller_, const ros::Duration& dt);
        void updateJointTorque(double setPoint, pr2_mechanism_model::JointState* joint_state_, control_toolbox::Pid* pid_controller_, const ros::Duration& dt);
        void updateJointVelocity(double setPoint, pr2_mechanism_model::JointState* joint_state_, control_toolbox::Pid* pid_controller_, const ros::Duration& dt, const int& jointIndex);
        
        void positionCommand(const brics_actuator::JointPositions &jointPositions);
        void velocityCommand(const brics_actuator::JointVelocities &jointVelocities);
        void torqueCommand(const brics_actuator::JointTorques &jointTorques);
        void executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as);

        friend class JointStateObserverGazebo;
    };

} // namespace

#endif
