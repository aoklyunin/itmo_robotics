#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Модуль управление Кукой
"""
import random
import numpy as np

import datetime
import rospy

from kukaWrapper.robotController import RobotController


class KukaController(RobotController):
    """
       Класс управления кукой
    """

    def forceControl(self):
        """
            режим Force-Control
        """
        targetVel = [0] * 7
        while (True):
            for i in range(5):
                targetVel[i] = -self.overG[i] * self.G_K[i]
                if abs(targetVel[i]) > 0.2:
                    targetVel[i] = np.sign(targetVel[i]) * 0.1

            print(targetVel)
            self.setJointVelocities(targetVel)
            rospy.sleep(0.1)

    def fullFriction(self):
        """
            Эксперимент по трениям по всем джоинтам
        """
        data = [
            [2, 3],
            [1, 1.5],
            [1.2, 2],
            [1.5, 3],
            [2, 3],
        ]
        for i in range(5):
            print("exp №" + str(i))
            self.makeTrapezeSimpleCiclic(i, data[i][0], data[i][1])

    def makeTrapezeSimpleCiclic(self, jointNum, arange, maxW):
        """
             Эксперимент с трапециями
        :param jointNum: Номер звена
        :param arange: Диапазон
        :param maxW: максимальная скорость
        """
        print("big")
        for i in range(int(maxW) * 5 - 5):
            self.inCandleWithWaiting()
            print("in candle")
            rospy.sleep(0.5)
            print(maxW - float(i) / 5)
            self.makeSimpleTrapeze(jointNum, arange, maxW - float(i) / 5, 20)

        print("middle")
        for i in range(9):
            self.inCandleWithWaiting()
            print("in candle")
            rospy.sleep(0.5)
            print(1 - float(i) / 10)
            self.makeSimpleTrapeze(jointNum, arange, 1 - float(i) / 10, 10)

        print("little")
        for i in range(10):
            self.inCandleWithWaiting()
            print("in candle")
            rospy.sleep(0.5)
            print(0.1 - float(i) / 100)
            self.makeSimpleTrapeze(jointNum, arange, 0.1 - float(i) / 100, 5)

    def makeSimpleTrapeze(self, jointNum, arange, maxW, repeatCnt):
        """
            Один прогон трапеции
        :param jointNum: номер джоинта
        :param arange: диапазон положений
        :param maxW:  максимальная скорость
        :param repeatCnt:  кол-во повторений
        """
        angleStart = self.jointState.position[jointNum - 1]
        curPos = angleStart

        # влев
        while curPos - angleStart < arange:
            curPos = self.jointState.position[jointNum - 1]
            self.setJointVelocity(jointNum, maxW)

        self.setJointVelocity(jointNum, 0)

        for i in range(repeatCnt):
            rospy.sleep(0.5)

            while curPos - angleStart > -arange:
                curPos = self.jointState.position[jointNum - 1]
                self.setJointVelocity(jointNum, -maxW)

            self.setJointVelocity(jointNum, 0)

            rospy.sleep(0.5)

            # влев
            while curPos - angleStart < arange:
                curPos = self.jointState.position[jointNum - 1]
                self.setJointVelocity(jointNum, maxW)

            self.setJointVelocity(jointNum, 0)

        # в начало
        while curPos < angleStart:
            curPos = self.jointState.position[jointNum - 1]
            self.setJointVelocity(jointNum, maxW)

        self.setJointVelocity(jointNum, 0)

        rospy.sleep(0.5)

    def gravitationFind(self):
        """
            Эксперимент для гравитации
        """
        for y in range(int((self.jointsRange[1][0] - 0) * 5), int((self.jointsRange[1][1] - 0) * 5)):
            valJ2 = float(y) / 5
            for k in range(int((self.jointsRange[2][0] + 0) * 5), int((self.jointsRange[2][1] - 0) * 5)):
                valJ3 = float(k) / 5
                for j in range(int((self.jointsRange[3][0] + 0.5) * 5), int((self.jointsRange[3][1] - 0) * 5)):
                    valJ4 = float(j) / 5
                    for i in range(4):
                        valJ5 = 3.14 / 4 * float(i)
                        print("%.3f %.3f %.3f %.3f" % (valJ5, valJ4, valJ3, valJ2,))
                        if self.setPosAndWait([2.01, valJ2, valJ3, valJ4, valJ5]):
                            rospy.sleep(2.5)
                            # for i in range(int((self.jointsRange[4][0] + 1.5) * 5), int((self.jointsRange[4][1] - 1.5) * 5)):
                            #         val = float(i) / 10
                            #         print(val)



    def gravitationFindQ(self):
        Q = [
            [0.6230, 0.2000, - 0.3276, 0.2350, 3.2646],
            [2.1252, 2.2310, - 2.1152, 1.6990, 0.3876],
            [1.8840, 1.7770, - 1.1896, 2.2242, 2.3580],
            [1.1580, 0.9710, - 1.8840, 2.6750, 0.5950],
            [2.4330, 2.0046, - 1.4616, 2.5430, 1.1750],
            [2.8092, 1.7540, - 0.1866, 2.1420, 2.0996],
            [2.2140, 2.2290, - 1.0156, 0.7946, 1.2190],
            [5.3060, 2.2940, - 2.2600, 2.1400, 3.3730],
            [0.5520, 0.3000, - 1.5156, 1.4160, 2.2110],
            [2.2532, 0.7960, - 2.0546, 0.6040, 0.3116],
            [1.0850, 2.0784, - 2.0096, 2.2032, 3.0980],
            [5.5300, 1.1550, - 4.4470, 0.9050, 2.3940],
        ]
        for i in range(len(Q)):
            print (i)
            if self.setPosAndWait([Q[i][0], Q[i][1], Q[i][2], Q[i][3], Q[i][4]]):
                rospy.sleep(4)

    def gravitationFindR(self):
        for i in range(400):
            self.moveToRandomConf3(10)
            print(i)


    def warmUpRobot(self):
        for i in range(100):
            self.moveToRandomConf3(0)
            print(i)


    def moveToRandomConf3(self, sleepTime):
        flgMoved = False
        pos = self.jointState.position

        while not flgMoved:
            j = [0, 0, 0, 0, 0]
            for i in range(5):
                j[i] = random.uniform(self.jointsRange[i][0], self.jointsRange[i][1])
            j[0] = pos[0]
            j[4] = pos[4]
            if self.setPosAndWait(j):
                rospy.sleep(sleepTime)
                flgMoved = True

    def moveToRandomConf(self, sleepTime):
        flgMoved = False
        while not flgMoved:
            j = [0, 0, 0, 0, 0]
            for i in range(5):
                j[i] = random.uniform(self.jointsRange[i][0], self.jointsRange[i][1])

            if self.setPosAndWait(j):
                rospy.sleep(sleepTime)
                flgMoved = True

    def zeroMomentA(self, j):
        """
            поиск нулевого момента, режим А
        :param j: номер звена
        """
        print("A")
        D = 0.3
        candlePosJ4 = 1.74
        for i in range(20):
            print(i)
            offset = random.uniform(-D, D)
            targetPos = candlePosJ4 + offset
            self.setJointPosition(j, targetPos)
            rospy.sleep(3)


    def zeroMomentB(self, j):
        """
                    поиск нулевого момента, режим B
                :param j: номер звена
                """
        print("B")
        D = 0.3
        candlePosJ4 = 1.74
        for i in range(int(D * 200)):
            targetPos = (D + candlePosJ4 - float(i) / 100)
            print(targetPos)
            self.setJointPosition(j, targetPos)
            rospy.sleep(3)

        for i in range(int(D * 200)):
            targetPos = (candlePosJ4 - D + float(i) / 100)
            print(targetPos)
            self.setJointPosition(j, targetPos)
            rospy.sleep(3)

    def zeroMomentInJoint(self, j):
        """
        эксперимент по поиску нулевого момента
        :param j: номер звена
        :return:
        """
        for i in range(30):
            self.zeroMomentA(j)
            self.zeroMomentB(j)
