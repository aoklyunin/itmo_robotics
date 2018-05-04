#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для управления кукой
"""
import json
import math
import random
import datetime
from wx import wx
import rospy
import brics_actuator.msg
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import nav_msgs.msg
from kinematic import getDHMatrix, getG
import time


class RobotDesription:

    def __init__(self, pathname):
        with open(pathname) as json_file:
            data = json.load(json_file)
            self.name = data["name"]
            self.joints = data["links"]
            self.jointCnt = len(self.joints)
            self.jointData = {
                "positions": [0] * self.jointCnt,
                "velocities": [0] * self.jointCnt,
                "efforts": [0] * self.jointCnt,
                "accelerations": [0] * self.jointCnt,
            }
            self.baseData = {
                "positions": [0] * 3,
                "velocities": [0] * 3,
                "accelerations": [0] * 3,
            }
        self.prevGettingJointDataTime = time.time()
        self.prevGettingBaseDataTime = self.prevGettingJointDataTime
        self.startTime = self.prevGettingJointDataTime

    def setJointData(self, positions, velocities, efforts):
        curtime = time.time()
        deltaTime = curtime - self.prevGettingJointDataTime
        self.prevGettingJointDataTime = curtime

        for i in range(min(len(positions), self.jointCnt)):
            oldVelocity = self.jointData["velocities"][i]
            self.jointData["positions"][i] = positions[i]
            self.jointData["velocities"][i] = velocities[i]
            self.jointData["accelerations"][i] = (self.jointData["velocities"][i] - oldVelocity) / deltaTime
            self.jointData["efforts"][i] = efforts[i]

    def setBaseData(self, positions):
        curtime = int(round(time.time() * 1000))
        deltaTime = curtime - self.prevGettingBaseDataTime
        self.prevGettingBaseDataTime = curtime

        for i in range(min(len(positions), self.jointCnt)):
            oldPosition = self.baseData["positions"][i]
            oldVelocity = self.baseData["velocities"][i]
            newPosition = positions[i]
            newVelocity = (newPosition - oldPosition) / deltaTime
            newAcceleration = (newVelocity - oldVelocity) / deltaTime
            self.baseData["positions"][i] = newPosition
            self.baseData["velocities"][i] = newVelocity
            self.baseData["accelerations"][i] = newAcceleration

    def setJointDataFromRos(self, data):
        joint_names = data.name
        positions = []
        efforts = []
        velocities = []
        for joint in self.joints:
            n = joint_names.index(joint["name"])
            positions.append(data.position[n])
            efforts.append(data.effort[n])
            velocities.append(data.velocity[n])

        self.setJointData(positions, velocities, efforts)

    def setBaseDataFromRos(self, data):
        pass


class RobotController:
    """
    Родительский класс для управления кукой
    """

    # Максимальная скорость звеньев робота
    maxJointVelocity = math.radians(90)

    # диапазон допустимых значений джоинтов
    jointsRange = [
        [0.011, 5.840],
        [0.011, 2.617],
        [-5.0, -0.02],
        [0.03, 3.42],
        [0.15, 5.641],
    ]
    L = [33, 147, 155, 135, 217.5]

    task = [0, 0, 0, 0, 0, 0, 0]

    TARGET_TYPE_MANY_JOINTS = 0
    TARGET_TYPE_ONE_JOINT = 1
    TARGET_TYPE_NO_TARGET = -1

    candlePos = [2.01, 1.09, -2.44, 1.74, 2.96]

    targetJPoses = [0, 0, 0, 0, 0]
    targetJPos = 0
    targetJposNum = -1
    targetType = TARGET_TYPE_NO_TARGET
    startTime = 0

    jointOffsets = [
        -170.0 / 180 * math.pi,
        -65.0 / 180 * math.pi,
        (90 + 60.0) / 180 * math.pi,
        0.0 / 180 * math.pi,
        -90.0 / 180 * math.pi
    ]

    flgReached = True

    # диапазон допустимых значений движения гриппера
    gripperRange = [0.0, 0.011499]
    # константы типов сообщений
    TYPE_JOINT_POSITIONS = 0  # положение джоинта
    TYPE_JOINT_VELOCITIES = 1  # скорость джоинта
    TYPE_JOINT_TORQUES = 2  # момент джоинта
    TYPE_GRIPPER_POSITION = 3  # положение гриппера
    TYPE_GRIPPER_VELOCITY = 4  # скорость гриппера
    TYPE_GRIPPER_TORQUE = 5  # сила гриппера
    # ответные данные, полученные от куки
    jointState = sensor_msgs.msg.JointState()

    overG = [0] * 5
    G = [0] * 5
    F = [0] * 3

    G_ERROR_RANGE = [1, 1.8, 0.95, 0.65, 0.4]
    G_K = [0.2, 0.1, 0.07, 0.3, 0.5]
    MAX_V = [0.4, 0.4, 0.4, 0.4, 0.4]

    reachedAction = False

    def warn(self, message, caption='Ае!'):
        """
            Показать сообщение
        :param message: Текст сообщения
        :param caption: Заголовок сообщения
        """
        dlg = wx.MessageDialog(None, message, caption, wx.OK | wx.ICON_WARNING)
        dlg.ShowModal()
        dlg.Destroy()

    def inCandleWithWaiting(self):
        """
            В свечу с режимом ожидания
        """
        self.setJointPositions(self.candlePos)
        self.targetType == self.TARGET_TYPE_MANY_JOINTS
        self.targetJPoses = self.candlePos
        while self.targetType == self.TARGET_TYPE_MANY_JOINTS:
            rospy.sleep(0.1)

    def checkPositionXYZEnable(self, pos):
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

    #
    def checkPositionJEnabled(self, joints):
        """
             проверка, что положение допустимо
        :param joints: обобщённые координаты
        :return: True/False
        """
        xyz = self.getEndEffectorPosByJ(joints)

        # print(xyz)
        return self.checkPositionXYZEnable(xyz)

    #
    def warmUpLink(self, n, t):
        """
            разогрев
        :param n: номер звена
        :param t: время разогрева
        """
        startTime = time.time()
        while (time.time() - startTime < t * 60):
            pos = random.uniform(self.jointsRange[n - 1][0], self.jointsRange[n - 1][1])
            self.setJointPosition(n, pos)
            rospy.sleep(0.5)

    def setPosAndWait(self, joints):
        """
             отправить в позицию и ждать, пока робот в неё не придёт
        :param joints: обобщённые координаты
        """
        if self.checkPositionJEnabled(joints):
            self.setJointPositions(joints)
            self.targetType == self.TARGET_TYPE_MANY_JOINTS
            self.targetJPoses = joints
            for k in range(5):
                self.task[k] = joints[k]
            cnt = 0
            while self.targetType == self.TARGET_TYPE_MANY_JOINTS and cnt < 5:
                rospy.sleep(0.5)
                cnt += 1
            return True
        return False

    def checkCurPositionEnabled(self):
        """
             проверить, что текущее положение допустимо(это нужно для проверки рассчётов допустимой зоны)
        :return: True/False
        """
        xyz = self.getEndEffectorPosByJ(self.jointState.position)
        # print(xyz)
        return self.checkPositionXYZEnable(xyz)

    def randomPoints(self, n, tp):
        """
            отправить робота в рандомные положения
        :param n: кол-во положений
        :param tp: пауза в секндах между командами
        :return:
        """
        for i in range(n):
            print(i)
            rospy.sleep(tp)
            flgCreated = False
            while not flgCreated:
                joints = []

                for j in range(5):
                    joints.append(random.uniform(self.jointsRange[j][0], self.jointsRange[j][1]))

                if self.checkPositionJEnabled(joints):
                    flgCreated = True
                    self.setJointPositions(joints)
                    self.targetType == self.TARGET_TYPE_MANY_JOINTS
                    self.targetJPoses = joints
                    for k in range(5):
                        self.task[k] = joints[k]

            while self.targetType == self.TARGET_TYPE_MANY_JOINTS:
                rospy.sleep(1)

            self.task = [0] * 7

        self.warn("Робот отработал все точки")

    def getDH(self, position):
        """
            по положению получаем матрицу перехода из энд-эффектора в асолютную систему координат
        :param position: массив из пяти элементов с обобщёнными координатами робота
        :return: Матрица преобразования 4х4
        """
        DH1 = getDHMatrix(math.pi / 2, self.L[0], self.L[1], position[0] + self.jointOffsets[0])
        DH2 = getDHMatrix(0, self.L[2], 0, position[1] + self.jointOffsets[1] + math.pi / 2)
        DH3 = getDHMatrix(0, self.L[3], 0, position[2] + self.jointOffsets[2])
        DH4 = getDHMatrix(math.pi / 2, 0, 0, position[3] + self.jointOffsets[3])
        DH5 = getDHMatrix(0, 0, self.L[4], position[4] + self.jointOffsets[4])
        return DH1 * DH2 * DH3 * DH4 * DH5

    def getTF(self):
        """
            Текущая матрица преобразования робота
        :return: Матрица преобразования 4х4
        """
        return self.getDH(self.jointState.position)

    def getEndEffectorPos(self):
        """
            получить положение энд-эффектора
        :return: Массив декартовых координат [x,y,z]
        """
        # tf = self.getTF()
        # return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]
        return [10.01, 200.2, -300.2]

    def getEndEffectorPosByJ(self, joints):
        """
            Возвращает положение энд-эффектора по обобщённым координтатам
        :param joints:  массив из пяти элементов с обобщёнными координатами робота
        :return: Массив декартовых координат [x,y,z]
        """
        tf = self.getDH(joints)
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def checkIfListIsZero(self, lst):
        """
            проверить, что список состоит из нулей
        :param lst: массив(список)
        :return: True/False
        """
        flg = True
        for i in range(len(lst)):
            if lst[i] != 0:
                flg = False
        return flg

    def calculateOverG(self):
        """
             посчитать избыточные моменты
        :return: массив(список) избыточных моментов на каждом звене
        """
        #
        # for i in range(5):
        #     self.overG[i] = self.jointState.effort[i] - self.G[i]
        #     if abs(self.overG[i]) < self.G_ERROR_RANGE[i]:
        #         self.overG[i] = 0

    def getF(self):
        self.F[0] = self.overG[0]
        self.F[1] = self.overG[1]
        self.F[2] = self.overG[2]

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

    def __init__(self, controlType):
        """
            конструктор
        """
        self.controlType = controlType
        self.rd = RobotDesription('../../config/dh_params/kuka_youbot.json')

        # переменные для публикации в топики
        self.armPubArr = [
            rospy.Publisher("/kuka_youbot/arm_joint_" + str(i) + "_" + controlType + "_controller/command",
                            std_msgs.msg.Float64, queue_size=1) for i in range(1, self.rd.jointCnt + 1)]

        self.cartVelPub = rospy.Publisher("/kuka_youbot/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        self.gripperPub = [rospy.Publisher("/kuka_youbot/gripper_finger_joint_l_" + controlType +
                                           "_controller/command", std_msgs.msg.Float64, queue_size=1),
                           rospy.Publisher("/kuka_youbot/gripper_finger_joint_r_" + controlType +
                                           "_controller/command", std_msgs.msg.Float64, queue_size=1)
                           ]

        self.jointStateSubscriber = rospy.Subscriber("kuka_youbot/joint_states", sensor_msgs.msg.JointState,
                                                     self.jointStateCallback)
        self.odomSubscriber = rospy.Subscriber("kuka_youbot/odom", nav_msgs.msg.Odometry,
                                               self.odomCallback)
        dt = datetime.datetime.now()

        date = dt.strftime("%d_%m_%Y_%I_%M%p")
        self.outLog = open('../../logs/' + date + '.csv', 'wb')

        self.startTime = time.time()
        # пауза необходима для правильной обработки пакетов
        rospy.sleep(1)
        rospy.loginfo("Kuka created")

    def getUnitValue(self, tp):
        """
            получаем по типу топика размерность
        :param tp: Топик
        :return: размерность измеряемой величины
        """
        if tp == self.TYPE_JOINT_POSITIONS:  # положение джоинтов
            return "rad"
        elif tp == self.TYPE_JOINT_VELOCITIES:  # скорость джоинтов
            return "s^-1 rad"
        elif tp == self.TYPE_JOINT_TORQUES:  # силы джоинтов
            return "m^2 kg s^-2 rad^-1"
        elif tp == self.TYPE_GRIPPER_POSITION:  # положение гриппера
            return "m"
        elif tp == self.TYPE_GRIPPER_VELOCITY:  # сокрость гриппера
            return "s^-1 m"
        elif tp == self.TYPE_GRIPPER_TORQUE:  # сила гриппера
            return "m kg s^-2"
        else:
            return None

    def generateJoinVal(self, joint_num, val, tp):
        """
            генерируем сообщение по одному звену для роса
        :param joint_num: номер звена
        :param val: значение
        :param tp: единица измерения
        :return: сообщение по одному звену
        """
        # создаём сообщение джоинта
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        # имя джоинта
        jv.joint_uri = "arm_joint_" + str(joint_num)
        # размерность
        jv.unit = self.getUnitValue(tp)
        # непосредственное значение
        jv.value = val
        return jv

    def setCarrigeVel(self, x, y, z):
        """
            задать скорость каретке
        :param x, y: линейные перемещения
        :param z:поворот
        """
        # создаём сообщение
        msg = geometry_msgs.msg.Twist()
        # заполняем его данными
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = z
        # публикуем сообщение в топик
        self.cartVelPub.publish(msg)
        # выполняем задержку (ебаный рос)
        rospy.sleep(1)

    def setGripperPositions(self, leftG, rightG):
        """
            управляет положением гриппера в миллиметрах
        :param leftG:  положение левого пальца
        :param rightG: положение правого пальцев
        """
        # формируем сообщение положения джоинтов
        msg = brics_actuator.msg.JointPositions()
        # положения джоинтов - это список
        msg.positions = []
        # создаём объект описания жвижения жоинта
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        # тип объекта - левый палец
        jv.joint_uri = "gripper_finger_joint_l"
        # размерность
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_POSITION)
        # значение делим на 1000, так как масимальное смещение гриппера 110 мм
        jv.value = leftG / 1000
        # добавляем объект в сообщение
        msg.positions.append(jv)
        # делаем тоже самое для правого пальца
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_POSITION)
        jv.value = rightG / 1000
        msg.positions.append(jv)
        # публикуем сообщение в топике
        self.positionGripperPub.publish(msg)
        # делаем задержку
        rospy.sleep(1)

    def setGripperVelocities(self, leftG, rightG):
        """
            управляет скоростью гриппера в мм/с
        :param leftG:  скорость левого пальца
        :param rightG: скорость правого пальцев
        """
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = []
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_l"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_VELOCITY)
        jv.value = leftG / 1000
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.velocities.append(jv)
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_VELOCITY)
        jv.value = rightG / 1000
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.velocities.append(jv)
        self.velocityGripperPub.publish(msg)
        rospy.sleep(1)

    def setGripperTorques(self, leftG, rightG):
        """
            управляет силой гриппера в Н
        :param leftG:  сила левого пальца
        :param rightG: сила правого пальцев
        """
        msg = brics_actuator.msg.JointTorques()
        msg.torques = []
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_l"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_TORQUE)
        jv.value = leftG
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.torques.append(jv)
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_TORQUE)
        jv.value = rightG
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.torques.append(jv)
        self.forceGripperPub.publish(msg)
        rospy.sleep(1)

    def setRobotToCandle(self):
        """
            Поставить робота в свечку
        """
        self.setJointPositions(self.candlePos)

    def setJointCommands(self, jointCommands):
        """
            задаём положения джоинтов в радианах
        :param joints: массив из пяти элементов с желаемыми положениями
        """
        print(jointCommands)

        msg = std_msgs.msg.Float64()
        for i in range(self.rd.jointCnt):
            if jointCommands[i][0]:
                msg.data = jointCommands[i][1]
                self.armPubArr[i].publish(msg)

        rospy.sleep(0.1)

    def setJointPosition(self, joint_num, value):
        """
             задаёт положение джоинта в радианах
        :param joint_num: номер джоинта
        :param value: угол поворота
        """
        # проверяем, что джоинт подходит
        if not joint_num in range(1, 5):
            rospy.logerror("Звено с номером " + str(joint_num) + " не определено")
            return

        msg = brics_actuator.msg.JointPositions()
        msg.positions = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_POSITIONS)]
        self.positionArmPub.publish(msg)

        rospy.sleep(1)

    def setJointTorques(self, joints):
        """
            задаёт моменты джоинтов в Н*м
        :param joints: массив из пяти элементов с желаемыми моментами
        """
        msg = brics_actuator.msg.JointTorques()
        msg.torques = []
        for i in range(5):
            jv = self.generateJoinVal(i + 1, joints[i], self.TYPE_JOINT_TORQUES)
            msg.torques.append(jv)
        self.torqueArmPub.publish(msg)
        rospy.sleep(1)

    def setJointTorque(self, joint_num, value):
        """
             задаёт момент джоинта в Н*м
        :param joint_num: номер джоинта
        :param value: желаемый момент
        """
        msg = brics_actuator.msg.JointTorques()
        msg.torques = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_TORQUES)]
        self.torqueArmPub.publish(msg)
        rospy.sleep(1)

    def setJointVelocity(self, joint_num, value):
        """
            задаёт скорости джоинтов в рад/с
        :param joints: массив из пяти элементов с желаемыми скорости
        """
        self.task[joint_num - 1] = value
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_VELOCITIES)]
        self.velocityArmPub.publish(msg)
        rospy.sleep(0.1)

    def setJointVelocities(self, joints):
        """
             задаёт скорость джоинта в рад/с
        :param joint_num: номер джоинта
        :param value: желаемая скорость
        """
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = []
        for i in range(5):
            jv = self.generateJoinVal(i + 1, joints[i], self.TYPE_JOINT_VELOCITIES)
            msg.velocities.append(jv)
        self.velocityArmPub.publish(msg)
        rospy.sleep(1)
