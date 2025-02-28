import tkinter as tk
from tkinter import ttk
from tkinter import scrolledtext
import matplotlib.axes
import matplotlib.axis
import serial.tools.list_ports
from serial import Serial
import struct
import xml.etree.ElementTree as ET
import csv
import time
import threading
import datetime
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as tf
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def plot_robot(x,y,theta):
    fig, ax = plt.subplots()
    robot_width = 0.3
    robot_body_polygon = patches.Rectangle((x - robot_width/2, y - robot_width/2), robot_width, robot_width, angle=np.rad2deg(theta), rotation_point='center' ,facecolor='red', edgecolor='black', alpha=0.7)
    dx = 0.2*np.cos(theta)
    dy = 0.2*np.sin(theta)
    robot_pointer = patches.Arrow(x, y, dx, dy, color="black",width=0.1)
    ax.add_patch(robot_body_polygon)
    ax.add_patch(robot_pointer)
    ax.set_xlim((-5,5))
    ax.set_ylim((-5,5))
    ax.grid(True)
    plt.show()

if __name__ == "__main__":
    plot_robot(0.5,0.5,1)