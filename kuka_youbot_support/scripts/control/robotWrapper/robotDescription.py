#!/usr/bin/env python
# -*- coding: utf-8 -*-
from kinematic import getTransfromMatrixFromDH, getTransformMatrix
from transformations import decompose_matrix, quaternion_from_matrix
import numpy as np
import json


class RobotDesription:

    def __init__(self, pathname):
        with open(pathname) as json_file:
            data = json.load(json_file)
            self.name = data["name"]
            self.jointDesriptions = []
            for link in data["links"]:
                self.jointDesriptions.append({
                    "name": link["name"],
                    "parent": link["parent"],
                    "child": link["child"],
                    "theta": link["theta"] / 180 * np.pi,
                    "d": link["d"] / 1000,
                    "a": link["a"] / 1000,
                    "alpha": link["alpha"] / 180 * np.pi,
                })
            self.jointCnt = len(self.jointDesriptions)
            self.jointData = {
                "positions": [0] * self.jointCnt,
                "velocities": [0] * self.jointCnt,
                "efforts": [0] * self.jointCnt,
            }
            self.baseData = {
                "positions": [0] * 3,
            }
            self.jointRanges = data["jointRanges"]
            self.baseToArmTransform = getTransformMatrix([
                data["baseTranform"]["translation"]["x"],
                data["baseTranform"]["translation"]["y"],
                data["baseTranform"]["translation"]["z"]
            ],
                [
                    data["baseTranform"]["rotation"]["x"],
                    data["baseTranform"]["rotation"]["y"],
                    data["baseTranform"]["rotation"]["z"],
                ],
                data["baseTranform"]["rotation"]["w"]
            )

            self.candlePos = [2.01, 1.09, -2.44, 1.74, 2.96]

    def getBFJointData(self):
        bfPositions = []
        bfVelocities = []
        for i in range(self.jointCnt):
            bfPositions.append(self.jointData["positions"][i] / np.pi * 180)
            bfVelocities.append(self.jointData["velocities"][i] / np.pi * 180)
        return {
            "positions": bfPositions,
            "velocities": bfVelocities,
        }

    def getBFBaseData(self):
        bfPositions = []
        for i in range(2):
            bfPositions.append(self.baseData["positions"][i] * 1000)

        bfPositions.append(self.baseData["positions"][2] / np.pi * 180)

        return {
            "positions": bfPositions,
        }

    def setJointData(self, positions, velocities, efforts):

        for i in range(min(len(positions), self.jointCnt)):
            self.jointData["positions"][i] = positions[i]
            self.jointData["velocities"][i] = velocities[i]
            self.jointData["efforts"][i] = efforts[i]

    def setBaseData(self, positions):

        for i in range(min(len(positions), self.jointCnt)):
            self.baseData["positions"][i] = positions[i]

    def setBaseDataFromRos(self, data):
        positions = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z]
        self.setBaseData(positions)

    def getLinkTranformMatrixFromDHParams(self, joint_num, positions=None):
        if positions == None:
            positions = self.jointData["positions"]

        return np.matmul(self.baseToArmTransform, getTransfromMatrixFromDH(joint_num, self.jointDesriptions, positions))

    def getEndEffectorCartesianPositionFromDHParams(self, positions=None):
        return self.getJointCartesianPositionFromDHParams(self.jointCnt, positions)

    def getEndEffectorEulerAnglesFromDHParams(self, positions=None):
        return self.getJointEulerAnglesFromDHParams(self.jointCnt, positions)

    def getJointCartesianPositionFromDHParams(self, joint_num, positions=None):
        return decompose_matrix(self.getLinkTranformMatrixFromDHParams(joint_num, positions))[3]

    def getJointEulerAnglesFromDHParams(self, joint_num, positions=None):
        return decompose_matrix(self.getLinkTranformMatrixFromDHParams(joint_num, positions))[2]

    def getJointQuaternionFromDHParams(self, joint_num, positions=None):
        return quaternion_from_matrix(self.getLinkTranformMatrixFromDHParams(joint_num, positions))

    def setJointDataFromRos(self, data):
        joint_names = data.name
        positions = []
        efforts = []
        velocities = []
        for joint in self.jointDesriptions:
            n = joint_names.index(joint["name"])
            positions.append(data.position[n])
            efforts.append(data.effort[n])
            velocities.append(data.velocity[n])

        self.setJointData(positions, velocities, efforts)

    def checkCartesianPositionEnabled(self, pos):
        """
            проверка, что положение допустимо
        :param pos: координаты в декартовом пространстве
        :return: True/False
        """
        x = pos[0]
        y = pos[1]
        z = pos[2]
        if z < 200:
            return False
        elif z < 300:
            return x ** 2 + y ** 2 > 350 ** 2
        return True

    def getEndEffectorPositionFromDH(self, positions=None):
        """
            получить положение энд-эффектора
        :return: Массив декартовых координат [x,y,z]
        """
        return self.getJointCartesianPositionFromDHParams(self.jointCnt - 1, positions)

    def checkJointPositionEnabled(self, positions=None):
        """
             проверка, что положение допустимо
        :param joints: обобщённые координаты
        :return: True/False
        """
        return self.checkCartesianPositionEnabled(self.getEndEffectorPositionFromDH(positions))
