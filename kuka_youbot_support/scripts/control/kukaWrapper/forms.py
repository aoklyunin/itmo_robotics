#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для описания окна приложения
"""

from abc import ABCMeta, abstractmethod, abstractproperty

import wx
import wx.grid as gridlib

from robotController import RobotController


class Frame(wx.Frame):
    """
        Класс для описания окна приложения
    """

    @abstractmethod
    def initExpItems(self):
        """Инициализация элементов управления экспериментами"""

    @abstractmethod
    def OnTest(self, event):
        """Тестовый метод"""

    @abstractmethod
    def ExpTimer(self):
        """Таймер для экспериментов. Срабатывает раз в пол-секунды"""

    def initGrid(self):
        """
            инициализация таблицу
        """
        # создаём таблицу
        self.robotGrid = gridlib.Grid(self.panel)
        # кол-во строк и столбцов
        # нумерация в таблице начинается с ячейки (0,0)
        self.robotGrid.CreateGrid(9, 4)
        # задаём имена столбцов
        self.robotGrid.SetColLabelValue(3, "Усилие")
        self.robotGrid.SetColLabelValue(2, "Ускорение")
        self.robotGrid.SetColLabelValue(1, "Скорость")
        self.robotGrid.SetColLabelValue(0, "Положение")
        # задаём ширину второго столбца больше, т.к. положение не "влезает"
        self.robotGrid.SetColSize(2, 90)
        self.robotGrid.SetRowLabelValue(0, "arm_joint_1")
        self.robotGrid.SetRowLabelValue(1, "arm_joint_2")
        self.robotGrid.SetRowLabelValue(2, "arm_joint_3")
        self.robotGrid.SetRowLabelValue(3, "arm_joint_4")
        self.robotGrid.SetRowLabelValue(4, "arm_joint_5")
        self.robotGrid.SetRowLabelValue(5, "gripper_joint_r")
        self.robotGrid.SetRowLabelValue(6, "gripper_joint_l")
        self.robotGrid.SetRowLabelValue(7, "base_x")
        self.robotGrid.SetRowLabelValue(8, "base_y")
        self.robotGrid.SetRowLabelValue(9, "base_alpha")
        # задаём ширину столбца с именами строк
        self.robotGrid.SetRowLabelSize(130)

    def onClose(self, event):
        """
            закрытие формы
        """
        # останавливаем таймер
        self.timer.Stop()
        self.Close()

    def setDataToGrid(self, rd):
        """
        записать полученную от куки дату в таблицу
        """
        for i in range(rd.jointCnt):
            self.robotGrid.SetCellValue(i, 0, ("%.2f" % rd.jointData["positions"][i]))
            self.robotGrid.SetCellValue(i, 1, ("%.2f" % rd.jointData["velocities"][i]))
            self.robotGrid.SetCellValue(i, 2, ("%.2f" % rd.jointData["accelerations"][i]))
            self.robotGrid.SetCellValue(i, 3, ("%.2f" % rd.jointData["efforts"][i]))

    def OnSendJointCommands(self, event):
        """
            управление положением по джоинтам
        """
        vals = [
            self.joint1CommandEntry.GetValue(),
            self.joint2CommandEntry.GetValue(),
            self.joint3CommandEntry.GetValue(),
            self.joint4CommandEntry.GetValue(),
            self.joint5CommandEntry.GetValue(),
        ]
        commands = [[False, 0] if val == "" else [True, float(val)] for val in vals]
        self.kuka.setJointCommands(commands)

    def OnTimer(self, event):
        """
            события по таймеру 2Гц
                """
        self.setDataToGrid(self.kuka.rd)
        self.setDHChords()
        self.ExpTimer()

    def OnUpdateJointCommands(self, event):
        """
         Обновить положения робота в полях ввода
                """
        if self.kuka.controlType == "Effort":
            data = self.kuka.rd.jointData["efforts"]
        elif self.kuka.controlType == "Velocity":
            data = self.kuka.rd.jointData["velocities"]
        else:
            data = self.kuka.rd.jointData["positions"]

        self.joint1CommandEntry.Clear()
        self.joint2CommandEntry.Clear()
        self.joint3CommandEntry.Clear()
        self.joint4CommandEntry.Clear()
        self.joint5CommandEntry.Clear()
        self.joint1CommandEntry.AppendText(str(round(data[0], 2)))
        self.joint2CommandEntry.AppendText(str(round(data[1], 2)))
        self.joint3CommandEntry.AppendText(str(round(data[2], 2)))
        self.joint4CommandEntry.AppendText(str(round(data[3], 2)))
        self.joint5CommandEntry.AppendText(str(round(data[4], 2)))

    def setDHChords(self):
        """
            Задать координаты в декартовом пространстве
                """
        endEffectorPos = self.kuka.getEndEffectorPos()
        self.posXText.SetLabel("X: "+str(round(endEffectorPos[0])))
        self.posYText.SetLabel("Y: "+str(round(endEffectorPos[1])))
        self.posZText.SetLabel("Z: "+str(round(endEffectorPos[2])))

    def OnClearJointCommands(self, event):
        """
            обнулить положения робота
                """
        self.joint1CommandEntry.Clear()
        self.joint2CommandEntry.Clear()
        self.joint3CommandEntry.Clear()
        self.joint4CommandEntry.Clear()
        self.joint5CommandEntry.Clear()

    def __init__(self, parent=None, id=-1, title='', pos=(0, 0), size=(690, 900)):
        """
            конструктор
        """
        # создаём фрейм
        wx.Frame.__init__(self, parent, id, title, pos, size)

        # добавляем на фрейм панель
        self.panel = wx.Panel(self)
        # инициализируем панель
        self.initGrid()
        # добавляем прокрутку на таблицу (пока что почему-то не работает, мб потому что таблица помещается полностью)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.robotGrid)
        self.panel.SetSizer(self.sizer)
        # инициализируем элементы управления
        self.initControlItems()
        # запускаем таймер
        self.timer = wx.Timer(self, -1)
        # раз в 0.5 секунды
        self.timer.Start(100)
        # указываем функцию, которая будет вызываться по таймеру
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)

    def initControlItems(self):
        """
            инициализирует элементы управления
        """

        # создаём поля ввода для джоинтов
        self.joint1CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(40, 270), size=(50, 30))
        self.joint2CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(100, 270), size=(50, 30))
        self.joint3CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(160, 270), size=(50, 30))
        self.joint4CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(220, 270), size=(50, 30))
        self.joint5CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(280, 270), size=(50, 30))

        # задаём фон заливки
        self.joint1CommandEntry.SetBackgroundColour('#DDFFEE')
        self.joint2CommandEntry.SetBackgroundColour('#DDFFEE')
        self.joint3CommandEntry.SetBackgroundColour('#DDFFEE')
        self.joint4CommandEntry.SetBackgroundColour('#DDFFEE')
        self.joint5CommandEntry.SetBackgroundColour('#DDFFEE')

        # конпки управления джоинтами и привязка методов к ним
        self.updateJointCommandsBtn = wx.Button(self.panel, label="Обновить", pos=(250, 310), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnUpdateJointCommands, self.updateJointCommandsBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.clearJointCommandsBtn = wx.Button(self.panel, label="Очистить", pos=(135, 310), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnClearJointCommands, self.clearJointCommandsBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.sendJointCommandsBtn = wx.Button(self.panel, label="Старт", pos=(20, 310), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJointCommands, self.sendJointCommandsBtn)

        # Положение в Декартовом пространстве
        self.posXText = wx.StaticText(self.panel, -1, "X", (60, 350))
        self.posYText = wx.StaticText(self.panel, -1, "Y", (130, 350))
        self.posZText = wx.StaticText(self.panel, -1, "Z", (200, 350))

        # добавляем на фрейм панель
        self.expPanel = wx.Panel(self.panel, pos=(0, 600), size=(690, 500))
        # добавляем элементы из эксперимента
        self.initExpItems()
