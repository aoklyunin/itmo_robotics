#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Модуль математических вычислений
"""

import numpy as np
from numpy import cos, sin

import functools,operator

from transformations import rotation_matrix, translation_matrix


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


def printMatrix(matrix,prefix=""):
    print(prefix+"[")
    for row in matrix.tolist():
        printList(row,prefix+"  ")
    print(prefix+"]")

def printList(lst,prefix=""):
    print(prefix+"["+functools.reduce(operator.add, ["%5.2f " % e for e in lst])+"]")


def getTransfromMatrixFromDH(joint_num, jointDescriptions, position):
    """
        по положению получаем матрицу перехода из энд-эффектора в асолютную систему координат
    :param position: массив из пяти элементов с обобщёнными координатами робота
    :return: Матрица преобразования 4х4
    """

    transformationMatrix = np.identity(4)
    # print("+++++++++++++++++++++++++++++++++++++++++++")
    for i in range(joint_num + 1):
        jointTransformationMatrix = getDHMatrix(jointDescriptions[i]["theta"] + position[i],
                                                jointDescriptions[i]["d"],
                                                jointDescriptions[i]["a"],
                                                jointDescriptions[i]["alpha"])

        transformationMatrix = np.matmul(transformationMatrix, jointTransformationMatrix)
    #     printList([jointDescriptions[i]["theta"] + position[i],
    #            jointDescriptions[i]["d"],
    #            jointDescriptions[i]["a"],
    #            jointDescriptions[i]["alpha"]])
    #
    #     printMatrix(jointTransformationMatrix)
    #
    # printMatrix(transformationMatrix)

    return transformationMatrix


def getTransformMatrix(translation, rotationAxe, rotationAngle):
    rm = rotation_matrix(rotationAngle, rotationAxe)
    rm[0, 3] = translation[0]
    rm[1, 3] = translation[1]
    rm[2, 3] = translation[2]
    return rm
