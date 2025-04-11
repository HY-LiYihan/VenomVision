# src/Detect/armor.py
import numpy as np
import cv2
from enum import Enum

class ArmorType(Enum):
    SMALL = "small"
    LARGE = "large"
    INVALID = "invalid"

class Armor:
    def __init__(self, l1=None, l2=None):
        if l1 and l2:
            if l1.center[0] < l2.center[0]:
                self.left_light = l1
                self.right_light = l2
            else:
                self.left_light = l2
                self.right_light = l1
            self.center = ((np.array(self.left_light.center) + np.array(self.right_light.center)) / 2).tolist()
        else:
            self.left_light = None
            self.right_light = None
            self.center = None
        self.type = ArmorType.INVALID
        self.number_img = None
        self.number = ""
        self.confidence = 0.0
        self.classfication_result = ""