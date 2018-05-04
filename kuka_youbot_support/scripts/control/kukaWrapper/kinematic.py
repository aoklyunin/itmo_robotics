#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Модуль математических вычислений
"""

import numpy as np
from numpy import cos, sin


def sign(a):
    if a > 0:
        return 1
    elif a < 0:
        return -1
    return 0


def getDHMatrix(theta, d, a, alpha):
    """
        Возвращает матрицу преобразования
    :param alpha, a, d, theta: DH параметры
    :return: Матрица перехода  4х4
    """
    return np.matrix(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1]
         ])


def getTransfromMatrix(jointCnt, jointDescriptions, position):
    """
        по положению получаем матрицу перехода из энд-эффектора в асолютную систему координат
    :param position: массив из пяти элементов с обобщёнными координатами робота
    :return: Матрица преобразования 4х4
    """

    transformationMatrix = np.identity(4)


    for i in range(jointCnt):
        jointTransformationMatrix = getDHMatrix(jointDescriptions[i]["theta"] + position[i], jointDescriptions[i]["d"],
                                                jointDescriptions[i]["a"],
                                                jointDescriptions[i]["alpha"])

        transformationMatrix = transformationMatrix*jointTransformationMatrix

    return transformationMatrix
