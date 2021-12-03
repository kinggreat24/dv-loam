'''
Author: kinggreat24
Date: 2021-08-18 22:23:01
LastEditTime: 2021-08-18 22:55:56
LastEditors: kinggreat24
Description: 
FilePath: /d2vl_slam/orb_slam2/Scripts/drawWeights.py
可以输入预定的版权声明、个性签名、空行等
'''

#! /usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import matplotlib.pyplot as plt
from   matplotlib.colors import LinearSegmentedColormap
import matplotlib.collections as mcoll
import matplotlib.path as mpath
import os
import sys
import argparse
import math
import pdb
from PIL import Image
import random

import matplotlib.pyplot as plt


class TDistributionWeight():
    def __init__(self, dof, mu, delta):
        self.dof   = dof
        self.mu    = mu
        self.delta = delta
    def weight(self,res):
        rr = (res-self.mu)/self.delta
        w = (1 + self.dof)/(self.dof + rr*rr)
        return w

def HuberWeight():
    pass

def TukeyWeight():
    pass


def plotWeightCurve():
    t_distribu = TDistributionWeight(5,0.0,3.0)
    

