#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для описания окна приложения
"""

from abc import ABCMeta, abstractmethod, abstractproperty

import wx
import wx.grid as gridlib


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

    def initRobotGrid(self):
        """
            инициализация таблицу
        """
        # создаём таблицу
        self.robotGrid = gridlib.Grid(self.panel)
        # кол-во строк и столбцов
        # нумерация в таблице начинается с ячейки (0,0)
        self.robotGrid.CreateGrid(8, 4)
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
        self.robotGrid.SetRowLabelValue(5, "base_x")
        self.robotGrid.SetRowLabelValue(6, "base_y")
        self.robotGrid.SetRowLabelValue(7, "base_alpha")
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
        bfJointData = rd.getBFJointData()
        for i in range(rd.jointCnt):
            self.robotGrid.SetCellValue(i, 0, ("%.2f" % bfJointData["positions"][i]))
            self.robotGrid.SetCellValue(i, 1, ("%.2f" % bfJointData["velocities"][i]))
            self.robotGrid.SetCellValue(i, 3, ("%.2f" % rd.jointData["efforts"][i]))

        bfBaseData = rd.getBFBaseData()
        for i in range(len(bfBaseData["positions"])):
            self.robotGrid.SetCellValue(i + rd.jointCnt, 0, ("%.2f" % bfBaseData["positions"][i]))

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
        self.kuka.setJointBfCommands(commands)

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
        bfJointData = self.kuka.rd.getBFJointData()

        if self.kuka.controlType == "Effort":
            data = bfJointData["efforts"]
        elif self.kuka.controlType == "Velocity":
            data = bfJointData["velocities"]
        else:
            data = bfJointData["positions"]

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
        endEffectorPos = self.kuka.getEndEffectorPosition()
        self.posXText.SetLabel("X: " + str(round(endEffectorPos[0])))
        self.posYText.SetLabel("Y: " + str(round(endEffectorPos[1])))
        self.posZText.SetLabel("Z: " + str(round(endEffectorPos[2])))

    def OnClearJointCommands(self, event):
        """
            обнулить положения робота
                """
        self.joint1CommandEntry.Clear()
        self.joint2CommandEntry.Clear()
        self.joint3CommandEntry.Clear()
        self.joint4CommandEntry.Clear()
        self.joint5CommandEntry.Clear()

    def __init__(self, kuka, parent=None, id=-1, title='', pos=(0, 0), size=(460, 900)):
        """
            конструктор
        """
        self.kuka = kuka
        # создаём фрейм
        wx.Frame.__init__(self, parent, id, title, pos, size)

        # добавляем на фрейм панель
        self.panel = wx.Panel(self)
        # инициализируем панель
        self.initRobotGrid()
        # добавляем прокрутку на таблицу (пока что почему-то не работает, мб потому что таблица помещается полностью)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.robotGrid)
        self.panel.SetSizer(self.sizer)
        # инициализируем элементы управления
        self.initControlItems()
        # запускаем таймер
        self.timer = wx.Timer(self, -1)
        self.deltaTime = 0.1
        # раз в 0.5 секунды
        self.timer.Start(int(self.deltaTime * 1000))

        # указываем функцию, которая будет вызываться по таймеру
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)

    def OnSendBaseCommandsBtn(self, event):
        self.kuka.setBfBaseVelocity(float(self.baseXCommandEntry.GetValue()),
                                    float(self.baseYCommandEntry.GetValue()),
                                    float(self.baseAlphaCommandEntry.GetValue()))

    def OnStopBaseCommandBtn(self, event):
        self.kuka.stopBase()

    def initControlItems(self):
        """
            инициализирует элементы управления
        """
        # создаём поля ввода для джоинтов
        self.joint1CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(80, 270), size=(50, 30))
        self.joint2CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(140, 270), size=(50, 30))
        self.joint3CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(200, 270), size=(50, 30))
        self.joint4CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(260, 270), size=(50, 30))
        self.joint5CommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(320, 270), size=(50, 30))

        # конпки управления джоинтами и привязка методов к ним
        self.sendJointCommandsBtn = wx.Button(self.panel, label="Старт", pos=(60, 310), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJointCommands, self.sendJointCommandsBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.clearJointCommandsBtn = wx.Button(self.panel, label="Очистить", pos=(175, 310), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnClearJointCommands, self.clearJointCommandsBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.updateJointCommandsBtn = wx.Button(self.panel, label="Обновить", pos=(290, 310), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnUpdateJointCommands, self.updateJointCommandsBtn)

        # Положение в Декартовом пространстве
        self.posXText = wx.StaticText(self.panel, -1, "X", (120, 350))
        self.posYText = wx.StaticText(self.panel, -1, "Y", (190, 350))
        self.posZText = wx.StaticText(self.panel, -1, "Z", (260, 350))

        self.jointControlLabel = wx.StaticText(self.panel, -1, "Manipulator Control", (160, 245))

        self.baseControlLabel = wx.StaticText(self.panel, -1, "Base Velocity Control", (160, 380))

        # создаём поля ввода для джоинтов
        self.baseXCommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(20, 410), size=(50, 30))
        self.baseYCommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(80, 410), size=(50, 30))
        self.baseAlphaCommandEntry = wx.TextCtrl(self.panel, -1, '0', pos=(140, 410), size=(50, 30))

        # добавляем на фрейм панель
        self.expPanel = wx.Panel(self.panel, pos=(0, 600), size=(690, 500))
        # добавляем элементы из эксперимента
        self.initExpItems()

        # конпки управления джоинтами и привязка методов к ним
        self.sendBaseCommandsBtn = wx.Button(self.panel, label="Старт", pos=(200, 410), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendBaseCommandsBtn, self.sendBaseCommandsBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.stopBaseCommandBtn = wx.Button(self.panel, label="Стоп", pos=(320, 410), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnStopBaseCommandBtn, self.stopBaseCommandBtn)
