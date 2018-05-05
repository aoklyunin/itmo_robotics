#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
    Модуль для создания окна
"""

from kukaController import KukaController
from robotWrapper.forms import Frame
import wx


class KukaFrame(Frame):
    """
        Класс графического окна
    """

    def initExpItems(self):
        """
            Инициализация элементов окна
        """
        pass

    def ExpTimer(self):
        """
            Таймер, срабатывает два раза в секунду. Всё, что сюда напишешь будет выполняться каждые 500 мсек
        """
        self.kuka.robotTimer()

