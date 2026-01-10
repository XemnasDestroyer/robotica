#!/usr/bin/python3

# -*- coding: utf-8 -*-
#    Copyright (C) 2025 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import time
import traceback
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
import itertools
import math
import os

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

# Estructura de la Red Neuronal
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, 3, 1)
        self.conv2 = nn.Conv2d(32, 64, 3, 1)
        self.fc1 = nn.Linear(9216, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = self.conv1(x)
        x = F.relu(x)
        x = self.conv2(x)
        x = F.relu(x)
        x = F.max_pool2d(x, 2)
        x = torch.flatten(x, 1)
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        output = F.log_softmax(x, dim=1)
        return output

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        # Variable para guardar el resultado de la detección
        self.detected_number = -1
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Cargar el modelo entrenado una sola vez
        try:
            self.model = Net().to(self.device)
            self.model.load_state_dict(torch.load("Modelo_MNIST.pt", map_location=self.device))
            self.model.eval()
            print("Modelo cargado correctamente")

        except Exception as e:
            print(f"Error cargando el modelo: {e}")

        if startup_check:
            self.startup_check()
        else:
            started_camera = False
            while not started_camera:
                try:
                    print("Connecting to Camera360RGB...")
                    self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                    print("Camera specs:")
                    print(" width:", self.rgb_original.width)
                    print(" height:", self.rgb_original.height)
                    print(" focalx", self.rgb_original.focalx)
                    print(" focaly", self.rgb_original.focaly)
                    print(" period", self.rgb_original.period)
                    print(" ratio {:.2f}".format(self.rgb_original.width / self.rgb_original.height))
                    started_camera = True
                    print("Connected to Camera360RGB")
                except Ice.Exception as e:
                    traceback.print_exc()
                    print(e, "Trying again CAMERA...")

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    @QtCore.Slot()
    def compute(self):

        image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
        color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
   
        # Procesamiento de imagen
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # Binarizar para encontrar cuadrados negros (invertido)
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500: # Filtrar ruido pequeño
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = float(w) / h
            
                # Comprobar si parece un cuadrado
                if 0.8 < aspect < 1.2:
                    # Extracción de la ROI y redimensionado
                    roi = gray[y:y+h, x:x+w]
                    roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
                  
                    # Preparar tensor para PyTorch
                    transform = transforms.Compose([
                        transforms.ToTensor(),
                        transforms.Normalize((0.1307,), (0.3081,))
                    ])
                    input_tensor = transform(roi).unsqueeze(0).to(self.device)

                    # Predicción (Forward pass)
                    with torch.no_grad():
                        output = self.model(input_tensor)
                        pred = output.argmax(dim=1, keepdim=True)
                        self.detected_number = pred.item()
                     
                    # Feedback visual
                    cv2.rectangle(color, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    found = True
                    break
      
        # Reiniciar detección si no se encuentra nada
        if not found:
            self.detected_number = -1
            
        cv2.imshow("Camera360RGB", color)
        cv2.waitKey(1)

    ####################################################################

    def startup_check(self):
        print(f"Testing RoboCompCamera360RGB.TRoi from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TRoi()
        print(f"Testing RoboCompCamera360RGB.TImage from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TImage()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods the component implements ==================

    # ===================================================================
    #
    # IMPLEMENTATION of getNumber method from MNIST interface
    #
    def MNIST_getNumber(self):
        
        #
        # call DNN and return detection result
        #
        
        return self.detected_number
    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCamera360RGB you can call this methods:
    # RoboCompCamera360RGB.TImage self.camera360rgb_proxy.getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
    ######################
    # From the RoboCompCamera360RGB you can use this types:
    # ifaces.RoboCompCamera360RGB.TRoi
    # ifaces.RoboCompCamera360RGB.TImage
