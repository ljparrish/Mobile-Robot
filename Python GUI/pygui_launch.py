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

class MobileRobotGUIApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Serial Monitor")
        self.master.geometry("1200x800")
        matplotlib.use("TkAgg")

        # Setup Frames
        # Occupancy Grid Frame
        self.Robot_Env_Frame = tk.Frame(self.master, width=500, height=500, relief="ridge", bd=5)
        self.Robot_Env_Frame.grid(row=0, column=0, padx=5, pady=5, sticky="nesw")

        # Serial Connection Frame
        self.Serial_Com_Frame = tk.Frame(self.master, width=500, height=300, relief="ridge", bd=5)
        self.Serial_Com_Frame.grid(row=1, column=0, padx=5, pady=5, sticky="nesw")

        # Control Frame
        self.Control_Frame = tk.Frame(self.master, width=300, height=300, relief="ridge", bd=5)
        self.Control_Frame.grid(row=1, column=1, pady=5, sticky="nesw")

        # Time Plots Frame
        self.Time_Plots_Frame = tk.Frame(self.master, width=500, height=500, relief="ridge", bd=5)
        self.Time_Plots_Frame.grid(row=0, column=1, pady=5, sticky="nesw")

        self.robot = MobileRobot()

        self.create_serial_widgets()
        self.create_control_widgets()
        self.create_timeseries_plot_widgets()
        self.create_robot_env_plot_widgets()

        # Flag to indicate if the serial connection is active
        self.connection_active = False

    def create_serial_widgets(self):
        self.port_combobox_label = ttk.Label(self.Serial_Com_Frame, text="Select Port:")
        self.port_combobox_label.grid(row=0, column=0, padx=5, pady=5, sticky="nw")

        self.populate_ports()
        
        self.baud_combobox_label = ttk.Label(self.Serial_Com_Frame, text="Select Baud Rate:")
        self.baud_combobox_label.grid(row=1, column=0, padx=5, pady=5, sticky="nw")

        self.baud_combobox = ttk.Combobox(self.Serial_Com_Frame, values=["2400","4800","9600","14400", "115200"], state="readonly")
        self.baud_combobox.set("115200")
        self.baud_combobox.grid(row=1, column=1, padx=5, pady=5, sticky="nw")

        self.connect_button = ttk.Button(self.Serial_Com_Frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=3, padx=5, pady=5, sticky="nw")

        self.disconnect_button = ttk.Button(self.Serial_Com_Frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_button.grid(row=1, column=3, padx=5, pady=5, sticky="nw")

        self.log_text = scrolledtext.ScrolledText(self.Serial_Com_Frame, wrap=tk.WORD, width=60, height=10)
        self.log_text.grid(row=2, column=0, columnspan=4, padx=5, pady=5, sticky="nwse")

        self.export_txt_button = ttk.Button(self.Serial_Com_Frame, text="Export as TXT", command=self.export_txt, state=tk.DISABLED)
        self.export_txt_button.grid(row=3, column=0, padx=5, pady=5, sticky="nwse")

        self.export_csv_button = ttk.Button(self.Serial_Com_Frame, text="Export as CSV", command=self.export_csv, state=tk.DISABLED)
        self.export_csv_button.grid(row=3, column=1, padx=5, pady=5, sticky="nwse")

        self.export_xml_button = ttk.Button(self.Serial_Com_Frame, text="Export as XML", command=self.export_xml, state=tk.DISABLED)
        self.export_xml_button.grid(row=3, column=2, padx=5, pady=5, sticky="nwse")

    def create_control_widgets(self):

        self.cmd_entry_1 = ttk.Entry(self.Control_Frame, state=tk.DISABLED)
        self.cmd_entry_1.grid(row=0, column=0, padx=5, pady=5, sticky="nesw")

        self.cmd_entry_2 = ttk.Entry(self.Control_Frame, state=tk.DISABLED)
        self.cmd_entry_2.grid(row=0, column=1, padx=5, pady=5, sticky="nesw")

        self.send_cmd_button = ttk.Button(self.Control_Frame, text="Send", command=self.send_cmd, state=tk.DISABLED)
        self.send_cmd_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nesw")

    def create_timeseries_plot_widgets(self):
        self.plot_figure, self.plot_ax = plt.subplots(3,1,sharex="all")

        self.plot1_canvas = FigureCanvasTkAgg(self.plot_figure, master=self.Time_Plots_Frame)
        self.plot1_canvas.get_tk_widget().grid(row=0, column=0, padx=5, pady=5, sticky="nesw")

        self.x_position_data = list()
        self.y_position_data = list()
        self.t_position_data = list()
        self.plot_time = list()

    def create_robot_env_plot_widgets(self):
        self.robot_env_figure, self.robot_env_ax = plt.subplots()

        self.robot_env_canvas = FigureCanvasTkAgg(self.robot_env_figure, master=self.Robot_Env_Frame)
        self.robot_env_canvas.get_tk_widget().grid(row=0, column=0, padx=5, pady=5, sticky="nesw")
        self.robot_env_ax.set_xbound(-5, 5)
        self.robot_env_ax.set_ybound(-5, 5)
        self.robot_env_ax.grid(True)

        self.robot_body_polygon = patches.Rectangle((self.robot.x_position - self.robot.width/2, self.robot.y_position - self.robot.width/2), self.robot.width, self.robot.width, angle=np.rad2deg(self.robot.theta), rotation_point='center' ,facecolor='red', edgecolor='black', alpha=0.7)
        self.robot_env_ax.add_patch(self.robot_body_polygon)
    
    def update_plots(self):
        while self.connection_active:
            try:
                self.plot_ax[0].plot(self.plot_time, self.x_position_data, color='b')
                self.plot_ax[1].plot(self.plot_time, self.y_position_data, color='r')
                self.plot_ax[2].plot(self.plot_time, self.t_position_data, color='y')
                self.plot1_canvas.draw()

                time.sleep(0.05)
            except Exception as e:
                print(f"Error Plotting Graphs: {str(e)}\n")

    def robot_env_plot_update(self):
        while self.connection_active:
            try:
                self.robot_body_polygon.set_xy((self.robot.x_position - self.robot.width/2, self.robot.y_position - self.robot.width/2))
                self.robot_body_polygon.set_angle(np.rad2deg(self.robot.theta))
                    
                dx = 0.2*np.cos(self.robot.theta)
                dy = 0.2*np.sin(self.robot.theta)
                self.robot_pointer = patches.Arrow(self.robot.x_position, self.robot.y_position, dx, dy, color="black",width=0.1)
                self.robot_env_ax.add_patch(self.robot_pointer)
                
                self.robot_env_canvas.draw()
                time.sleep(0.1)

            except Exception as e:
                print(f"Error UpdatingRobot Env Graphs: {str(e)}\n")

    def clear_plots(self):
        self.plot_time.clear()
        self.x_position_data.clear()
        self.y_position_data.clear()
        self.t_position_data.clear()

        self.plot_ax[0].cla()
        self.plot_ax[1].cla()
        self.plot_ax[2].cla()

        self.plot1_canvas.draw()

        #self.robot_env_ax.cla()
        #self.robot_env_ax.draw()

    def populate_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox = ttk.Combobox(self.Serial_Com_Frame, values=ports, state="readonly")
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="nw")

    def connect(self):
        port = self.port_combobox.get()
        baud = int(self.baud_combobox.get())
        try:
            self.ser = Serial(port, baud)
            self.log_text.delete(1.0, tk.END)
            self.log_text.insert(tk.END, f"Connected to {port} at {baud} baud\n")
            self.disconnect_button["state"] = tk.NORMAL
            self.connect_button["state"] = tk.DISABLED
            self.export_txt_button["state"] = tk.NORMAL
            self.export_csv_button["state"] = tk.NORMAL
            self.export_xml_button["state"] = tk.NORMAL
            self.send_cmd_button["state"] = tk.NORMAL
            self.cmd_entry_1["state"] = tk.NORMAL
            self.cmd_entry_2["state"] = tk.NORMAL

            self.connection_active = True

            self.serial_read_thread = threading.Thread(target=self.read_from_port)
            self.serial_read_thread.start()

            self.clear_plots()
            self.plot_update_thread = threading.Thread(target=self.update_plots)
            self.plot_update_thread.start()

            self.robot_env_plot_update_thread = threading.Thread(target=self.robot_env_plot_update)
            self.robot_env_plot_update_thread.start()
        except Exception as e:
            self.log_text.insert(tk.END, f"Error: {str(e)}\n")

    def disconnect(self):
        self.connection_active = False  # Set the flag to False to stop the reading thread
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.connect_button["state"] = tk.NORMAL
        self.disconnect_button["state"] = tk.DISABLED
        self.export_txt_button["state"] = tk.DISABLED
        self.export_csv_button["state"] = tk.DISABLED
        self.export_xml_button["state"] = tk.DISABLED
        self.send_cmd_button["state"] = tk.DISABLED
        self.cmd_entry_1["state"] = tk.DISABLED
        self.cmd_entry_2["state"] = tk.DISABLED
        self.log_text.insert(tk.END, "Disconnected\n")

    def read_from_port(self):
        while self.connection_active:  # Check the flag in the reading loop
            if self.ser.in_waiting > 36:
                try:
                    line = struct.unpack('<3i2b3x1B18x',self.ser.read(36))
                    print(f"Received: {line}")
                    if line:
                        self.log_text.insert(tk.END, f"{datetime.datetime.now()} ")
                        self.log_text.insert(tk.END, line)
                        self.log_text.insert(tk.END, "\n")
                        self.log_text.see(tk.END)

                        # Update robot's position and wheel speed
                        self.robot.update_position(line[0] / 1000.0, line[1] / 1000.0, np.arctan2(np.sin(line[2] / 1000.0), np.cos(line[2] / 1000.0)))
                        self.robot.update_wheelspeed(line[3], line[4])
                        self.update_timeseries_data()
                        #print(f"Robot Position is: X - {self.robot.x_position} Y - {self.robot.y_position} T - {self.robot.theta}")

                except Exception as e:
                    if self.connection_active:  # Only log errors if the connection is still active
                        self.log_text.insert(tk.END, f"Error reading from port: {str(e)}\n")
                        print(f"Error reading from port: {str(e)}\n")
                    break

    def update_timeseries_data(self):
        self.x_position_data.append(self.robot.x_position)
        self.y_position_data.append(self.robot.y_position)
        self.t_position_data.append(self.robot.theta)
        self.plot_time.append(datetime.datetime.now())
        print(f"Number of Timeseries Datapoints = {len(self.x_position_data)}")

    def export_txt(self):
        data = self.log_text.get(1.0, tk.END)
        filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.txt"
        with open(filename, "w") as file:
            file.write(data)
        self.log_text.insert(tk.END, f"Log exported as TXT: {filename}\n")

    def export_csv(self):
        data = self.log_text.get(1.0, tk.END)
        filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.csv"
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerows([line.split() for line in data.splitlines()])
        self.log_text.insert(tk.END, f"Log exported as CSV: {filename}\n")

    def export_xml(self):
        data = self.log_text.get(1.0, tk.END)
        filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.xml"
        root = ET.Element("LogData")
        lines = data.splitlines()
        for line in lines:
            entry = ET.SubElement(root, "Entry")
            ET.SubElement(entry, "Data").text = line
        tree = ET.ElementTree(root)
        tree.write(filename)
        self.log_text.insert(tk.END, f"Log exported as XML: {filename}\n")

    def send_cmd(self):
            tx_data = [np.int8(int(self.cmd_entry_1.get())), np.int8(int(self.cmd_entry_2.get()))]
            self.ser.write(tx_data)
            print(f"Sent: {tx_data}")
            self.cmd_entry_1.delete(0,tk.END)
            self.cmd_entry_2.delete(0,tk.END)


class MobileRobot:
    def __init__(self):
        self.x_position = 1000
        self.y_position = 1000
        self.theta = np.pi/2
        self.omega_r = 0
        self.omega_l = 0
        self.width = 0.3

    def update_position(self, x, y, t):
        self.x_position = x
        self.y_position = y
        self.theta = t

    def update_wheelspeed(self, w_l, w_r):
        self.omega_l = w_l
        self.omega_r = w_r

    def draw_robot(self, plotAxes, plotCanvas):
        pass

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = MobileRobotGUIApp(root)
        root.mainloop()
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected Ending Program.")