#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Модуль управление Кукой
"""
from robotWrapper.robotController import RobotController


class KukaController(RobotController):
    """
       Класс управления кукой
    """


    def __init__(self, control_type):
        RobotController.__init__(self, control_type)

    def robotTimer(self):
        # print("+++++++++++++++++++++++++++++++++++++++++")
        # for i in range(3):
        #     posDH = self.rd.getJointCartesianPositionFromDHParams(i)
        #     posROS = self.rd.getJointCartesianPositionFromRos(i)
        #     print(posDH)
        #     print(posROS)
        #     print("------------------------------------------")
        pass
    # print("test")
