#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для управления кукой
"""
import datetime
from wx import wx
import numpy as np
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import nav_msgs.msg

from robotDescription import RobotDesription


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
            self.setJointCommands(
                [[jointCommand[0], jointCommand[1] / 180 * np.pi] for jointCommand in jointCommands])
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

    def tfCallback(self, data):
        self.rd.setTfData(data)

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

    def destroy(self):
        pass
