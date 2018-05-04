#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для управления кукой
"""
import json
import datetime
from wx import wx

import numpy as np
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import nav_msgs.msg
from kinematic import getTransfromMatrix


class RobotDesription:

    def __init__(self, pathname):
        with open(pathname) as json_file:
            data = json.load(json_file)
            self.name = data["name"]
            self.jointDesriptions = []
            for link in data["links"]:
                self.jointDesriptions.append({
                    "name": link["name"],
                    "theta": link["theta"] / 180 * np.pi,
                    "d": link["d"],
                    "a": link["a"],
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

    def setBaseDataFromRos(self, data):
        positions = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z]
        self.setBaseData(positions)


class RobotController:
    """
    Родительский класс для управления кукой
    """

    def setJointCommands(self, jointCommands):
        """
            задаём положения джоинтов в радианах
        :param joints: массив из пяти элементов с желаемыми положениями
        """
        msg = std_msgs.msg.Float64()
        for i in range(self.rd.jointCnt):
            if jointCommands[i][0]:
                msg.data = jointCommands[i][1]
                self.jointPublisherArr[i].publish(msg)

        rospy.sleep(0.1)

    def setJointBfCommands(self, jointCommands):
        """
            задаём положения джоинтов в радианах
        :param joints: массив из пяти элементов с желаемыми положениями
        """
        if self.controlType != "Effort":
            self.setJointCommands([[jointCommand[0], jointCommand[1] / 180 * np.pi] for jointCommand in jointCommands])
        else:
            self.setJointCommands(jointCommands)

    def warn(self, message, caption='Ае!'):
        """
            Показать сообщение
        :param message: Текст сообщения
        :param caption: Заголовок сообщения
        """
        dlg = wx.MessageDialog(None, message, caption, wx.OK | wx.ICON_WARNING)
        dlg.ShowModal()
        dlg.Destroy()

    def checkCartesianPositionEnable(self, pos):
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

    def checkJointPositionEnabled(self, joints):
        """
             проверка, что положение допустимо
        :param joints: обобщённые координаты
        :return: True/False
        """
        xyz = self.getCartesianPositionByJoints(joints)
        return self.checkCartesianPositionEnable(xyz)

    def getEndEffectorPosition(self):
        """
            получить положение энд-эффектора
        :return: Массив декартовых координат [x,y,z]
        """
        tf = self.getTransformationMatrix()
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def getTransformationMatrix(self):
        """
            Текущая матрица преобразования робота
        :return: Матрица преобразования 4х4
        """
        return getTransfromMatrix(self.rd.jointCnt, self.rd.jointDesriptions, self.rd.jointData["positions"])

    def getCartesianPositionByJoints(self, joints):
        """
            Возвращает положение энд-эффектора по обобщённым координтатам
        :param joints:  массив из пяти элементов с обобщёнными координатами робота
        :return: Массив декартовых координат [x,y,z]
        """
        tf = self.getTransfromMatrix(self.rd.jointCnt, self.rd.jointDesriptions, joints)
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def jointStateCallback(self, data):
        """
            обработчик пришедших показаний датчика из роса
        :param data: Структура роса с показаниями датчика
        """
        self.rd.setJointDataFromRos(data)

    def odomCallback(self, data):
        """
            обработчик пришедших показаний датчика из роса
        :param data: Структура роса с показаниями датчика
        """
        self.rd.setBaseDataFromRos(data)

    def stopBase(self):
        self.setBaseVelocity(0, 0, 0)

    def setBaseVelocity(self, x, y, alpha):
        """
            задать скорость каретке
        :param x, y: линейные перемещения
        :param z:поворот
        """
        print(x, y, alpha)
        # создаём сообщение
        msg = geometry_msgs.msg.Twist()
        # заполняем его данными
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = alpha
        # публикуем сообщение в топик
        self.basePublisher.publish(msg)
        # выполняем задержку (ебаный рос)
        rospy.sleep(0.1)

    def setBfBaseVelocity(self, x, y, alpha):
        """
            задать скорость каретке
        :param x, y: линейные перемещения
        :param z:поворот
        """
        self.setBaseVelocity(x / 1000, y / 1000, alpha / 180 * np.pi)

    def __init__(self, controlType):
        """
            конструктор
        """
        self.controlType = controlType
        self.rd = RobotDesription('../../config/dh_params/kuka_youbot.json')

        # переменные для публикации в топики
        self.jointPublisherArr = [
            rospy.Publisher("/kuka_youbot/arm_joint_" + str(i) + "_" + controlType + "_controller/command",
                            std_msgs.msg.Float64, queue_size=1)
            for i in range(1, self.rd.jointCnt + 1)]

        self.basePublisher = rospy.Publisher("/kuka_youbot/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        self.jointStateSubscriber = rospy.Subscriber("kuka_youbot/joint_states", sensor_msgs.msg.JointState,
                                                     self.jointStateCallback)

        self.odomSubscriber = rospy.Subscriber("kuka_youbot/odom", nav_msgs.msg.Odometry,
                                               self.odomCallback)
        dt = datetime.datetime.now()

        date = dt.strftime("%d_%m_%Y_%I_%M%p")
        self.outLog = open('../../logs/' + date + '.csv', 'wb')

        rospy.sleep(0.1)
        rospy.loginfo("Kuka control program runed")
