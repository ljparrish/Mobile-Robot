import tkinter as tk
from tkinter import ttk
from tkinter import scrolledtext
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
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from config import Config

class ThreadManager:
    def __init__(self):
        self.active_threads = []
        self.thread_lock = threading.Lock()

    def start_threads(self, *targets):
        with self.thread_lock:
            for target in targets:
                thread = threading.Thread(target=target, daemon=True)
                thread.start()
                self.active_threads.append(thread)
    
    def stop_threads(self):
        with self.thread_lock:
            for thread in self.active_threads:
                if thread.is_alive():
                    # Implement thread termination logic if needed
                    pass
            self.active_threads.clear()

# MVC Pattern Implementation
class RobotModel:
    def __init__(self):
        self.x_position = 1000
        self.y_position = 1000
        self.theta = np.pi/2
        self.omega_r = 0
        self.omega_l = 0
        self.width = 0.3
        self.x_data = []
        self.y_data = []
        self.t_data = []
        self.plot_time = []

    def update_position(self, x, y, t):
        self.x_position = x
        self.y_position = y
        self.theta = t

    def update_wheelspeed(self, w_l, w_r):
        self.omega_l = w_l
        self.omega_r = w_r

    def append_plot_data(self, x, y, t, timestamp):
        self.x_data.append(x)
        self.y_data.append(y)
        self.t_data.append(t)
        self.plot_time.append(timestamp)

class RobotView:
    def __init__(self, master):
        self.master = master
        self.setup_ui()
        self.setup_plots()
        self.robot_pointer = None
        
    def setup_ui(self):
        # Frame setup with improved grid configuration
        self.Robot_Env_Frame = tk.Frame(self.master, relief="ridge", bd=5)
        self.Robot_Env_Frame.grid(row=0, column=0, padx=5, pady=5, sticky="nesw")

        self.Serial_Com_Frame = tk.Frame(self.master, relief="ridge", bd=5)
        self.Serial_Com_Frame.grid(row=1, column=0, padx=5, pady=5, sticky="nesw")

        self.Control_Frame = tk.Frame(self.master, relief="ridge", bd=5)
        self.Control_Frame.grid(row=1, column=1, pady=5, sticky="nesw")

        self.Time_Plots_Frame = tk.Frame(self.master, relief="ridge", bd=5)
        self.Time_Plots_Frame.grid(row=0, column=1, pady=5, sticky="nesw")

        # Add grid weight configuration for proper resizing
        self.master.grid_rowconfigure(0, weight=1)
        self.master.grid_rowconfigure(1, weight=1)
        self.master.grid_columnconfigure(0, weight=1)
        self.master.grid_columnconfigure(1, weight=1)

    def setup_plots(self):
        # Timeseries plot setup
        self.plot_figure, self.plot_ax = plt.subplots(3,1,sharex="all")
        self.plot1_canvas = FigureCanvasTkAgg(self.plot_figure, master=self.Time_Plots_Frame)
        self.plot1_canvas.get_tk_widget().grid(row=0, column=0, padx=5, pady=5, sticky="nesw")

        # Robot environment plot setup
        self.robot_env_figure, self.robot_env_ax = plt.subplots()
        self.robot_env_plot_frame = tk.Frame(self.Robot_Env_Frame, relief="ridge", bd=3)
        self.robot_env_plot_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nesw")
        self.robot_env_canvas = FigureCanvasTkAgg(self.robot_env_figure, master=self.robot_env_plot_frame)
        self.robot_env_canvas.get_tk_widget().grid(row=0, column=0, padx=5, pady=5, sticky="nesw")
        
        # Create robot body polygon
        self.robot_body_polygon = patches.Rectangle((0, 0), 0.3, 0.3, angle=0, color="blue", fill=False)
        self.robot_env_ax.add_patch(self.robot_body_polygon)

        # Control widgets for robot environment
        self.robot_env_plot_control = tk.Frame(self.Robot_Env_Frame, relief="ridge", bd=3)
        self.robot_env_plot_control.grid(row=1, column=0, padx=5, pady=5, sticky="nesw")

        # Add control elements
        self.follow_robot = tk.BooleanVar()
        self.follow_robot_checkbox = ttk.Checkbutton(self.robot_env_plot_control, text="Follow Robot", variable=self.follow_robot)
        self.follow_robot_checkbox.grid(row=0, column=0, padx=5, pady=5, sticky="w")

        self.robot_env_ax_boxwidth_scale = ttk.Scale(self.robot_env_plot_control, orient="horizontal", length=100, from_=1.0, to=15.0)
        self.robot_env_ax_boxwidth_scale.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.robot_env_ax_boxwidth_scale.set(10)

class RobotController:
    def __init__(self, model, view):
        self.model = model
        self.view = view
        self.connection_active = False
        self.simulation_active = False
        self.simulation_thread = None
        self.create_serial_widgets()
        self.create_control_widgets()
        self.setup_plot_bindings()
        
    def create_serial_widgets(self):
        # Port selection
        ttk.Label(self.view.Serial_Com_Frame, text="Select Port:").grid(row=0, column=0, padx=5, pady=5, sticky="nw")
        self.port_combobox = ttk.Combobox(self.view.Serial_Com_Frame, state="readonly")
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="nw")
        self.populate_ports()

        # Baud rate selection
        ttk.Label(self.view.Serial_Com_Frame, text="Select Baud Rate:").grid(row=1, column=0, padx=5, pady=5, sticky="nw")
        self.baud_combobox = ttk.Combobox(self.view.Serial_Com_Frame, values=["2400","4800","9600","14400", "115200"], state="readonly")
        self.baud_combobox.set("115200")
        self.baud_combobox.grid(row=1, column=1, padx=5, pady=5, sticky="nw")

        # Simulation mode toggle
        self.simulation_mode = tk.BooleanVar()
        self.simulation_checkbox = ttk.Checkbutton(self.view.Serial_Com_Frame, text="Simulation Mode", variable=self.simulation_mode, command=self.toggle_simulation_mode)
        self.simulation_checkbox.grid(row=2, column=0, padx=5, pady=5, sticky="nw")

        # Connection buttons
        self.connect_button = ttk.Button(self.view.Serial_Com_Frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=3, padx=5, pady=5, sticky="nw")

        self.disconnect_button = ttk.Button(self.view.Serial_Com_Frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_button.grid(row=1, column=3, padx=5, pady=5, sticky="nw")

        # Log display
        self.log_text = scrolledtext.ScrolledText(self.view.Serial_Com_Frame, wrap=tk.WORD, width=60, height=10)
        self.log_text.grid(row=3, column=0, columnspan=4, padx=5, pady=5, sticky="nwse")
        self.data_manager = DataManager(self.log_text)
        # Export buttons
        self.export_txt_button = ttk.Button(self.view.Serial_Com_Frame, text="Export as TXT", command=self.export_txt, state=tk.DISABLED)
        self.export_txt_button.grid(row=4, column=0, padx=5, pady=5, sticky="nwse")
        self.export_csv_button = ttk.Button(self.view.Serial_Com_Frame, text="Export as CSV", command=self.export_csv, state=tk.DISABLED)
        self.export_csv_button.grid(row=4, column=1, padx=5, pady=5, sticky="nwse")
        self.export_xml_button = ttk.Button(self.view.Serial_Com_Frame, text="Export as XML", command=self.export_xml, state=tk.DISABLED)
        self.export_xml_button.grid(row=4, column=2, padx=5, pady=5, sticky="nwse")

    def create_control_widgets(self):
        # Command entry fields
        self.cmd_entry_1 = ttk.Entry(self.view.Control_Frame, state=tk.DISABLED)
        self.cmd_entry_1.grid(row=0, column=0, padx=5, pady=5, sticky="nesw")

        self.cmd_entry_2 = ttk.Entry(self.view.Control_Frame, state=tk.DISABLED)
        self.cmd_entry_2.grid(row=0, column=1, padx=5, pady=5, sticky="nesw")

        # Send command button
        self.send_cmd_button = ttk.Button(self.view.Control_Frame, text="Send", command=self.send_cmd, state=tk.DISABLED)
        self.send_cmd_button.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nesw")

        # Simulation controls
        self.simulation_control_frame = tk.Frame(self.view.Control_Frame, relief="ridge", bd=3)
        self.simulation_control_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="nesw")

        ttk.Label(self.simulation_control_frame, text="Left Wheel Speed:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.simulation_left_wheel = ttk.Scale(self.simulation_control_frame, orient="horizontal", length=200, from_=-100, to=100)
        self.simulation_left_wheel.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.simulation_left_wheel.set(0)

        ttk.Label(self.simulation_control_frame, text="Right Wheel Speed:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.simulation_right_wheel = ttk.Scale(self.simulation_control_frame, orient="horizontal", length=200, from_=-100, to=100)
        self.simulation_right_wheel.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        self.simulation_right_wheel.set(0)

    def setup_plot_bindings(self):
        # Add plot update bindings
        self.view.robot_env_ax.set_xbound(-self.view.robot_env_ax_boxwidth_scale.get()/2, self.view.robot_env_ax_boxwidth_scale.get()/2)
        self.view.robot_env_ax.set_ybound(-self.view.robot_env_ax_boxwidth_scale.get()/2, self.view.robot_env_ax_boxwidth_scale.get()/2)
        self.view.robot_env_ax.grid(True)

    def populate_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox.config(values=ports)

    def toggle_simulation_mode(self):
        if self.simulation_mode.get():
            # Enable simulation mode
            self.log_text.delete(1.0, tk.END)
            self.log_text.insert(tk.END, "Simulation mode enabled\n")
            self.disconnect_button.config(state=tk.NORMAL)
            self.connect_button.config(state=tk.DISABLED)
            self.export_buttons_enable(True)
            self.send_cmd_button_enable(True)
            
            # Add visual indicator for simulation mode
            self.view.master.configure(bg='lightyellow')
            
            self.simulation_active = True
            self.connection_active = True  # Reuse the same flag to control plotting threads
            
            # Start simulation thread
            self.simulation_thread = threading.Thread(target=self.simulation_data_generator, daemon=True)
            self.simulation_thread.start()
            
            # Start plotting threads
            self.thread_manager = ThreadManager()
            self.thread_manager.start_threads(
                self.update_plots, 
                self.robot_env_plot_update
            )
        else:
            # Disable simulation mode
            self.log_text.insert(tk.END, "Simulation mode disabled\n")
            self.view.master.configure(bg='SystemButtonFace')  # Reset to default background
            self.simulation_active = False
            self.connection_active = False
            if self.simulation_thread and self.simulation_thread.is_alive():
                self.simulation_thread.join()
            self.disconnect()

    def connect(self):
        # If simulation mode is active, don't try to connect to a real port
        if self.simulation_mode.get():
            self.log_text.insert(tk.END, "Cannot connect while in simulation mode. Disable simulation mode first.\n")
            return
            
        port = self.port_combobox.get()
        baud = int(self.baud_combobox.get())
        try:
            self.ser = Serial(port, baud)
            self.log_text.delete(1.0, tk.END)
            self.log_text.insert(tk.END, f"Connected to {port} at {baud} baud\n")
            self.disconnect_button.config(state=tk.NORMAL)
            self.connect_button.config(state=tk.DISABLED)
            self.export_buttons_enable(True)
            self.send_cmd_button_enable(True)

            self.connection_active = True

            self.thread_manager = ThreadManager()
            self.thread_manager.start_threads(
                self.read_from_port, 
                self.update_plots, 
                self.robot_env_plot_update
            )
        except Exception as e:
            self.log_text.insert(tk.END, f"Error: {str(e)}\n")

    def disconnect(self):
        # If in simulation mode, just stop simulation
        if self.simulation_mode.get():
            self.simulation_active = False
            self.connection_active = False
            self.thread_manager.stop_threads()
            if self.simulation_thread and self.simulation_thread.is_alive():
                self.simulation_thread.join()
            self.connect_button.config(state=tk.NORMAL)
            self.disconnect_button.config(state=tk.DISABLED)
            self.export_buttons_enable(False)
            self.send_cmd_button_enable(False)
            self.log_text.insert(tk.END, "Simulation stopped\n")
            return
            
        self.connection_active = False
        self.thread_manager.stop_threads()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.export_buttons_enable(False)
        self.send_cmd_button_enable(False)
        self.log_text.insert(tk.END, "Disconnected\n")

    def read_from_port(self):
        while self.connection_active:
            if self.ser.in_waiting > 36:
                try:
                    line = struct.unpack(Config.SERIAL_DATA_FORMAT, self.ser.read(36))
                    if line:
                        self.data_manager.add_entry(line)
                        # Update model with new data
                        x_pos = line[0] / 1000.0
                        y_pos = line[1] / 1000.0
                        theta = np.arctan2(np.sin(line[2] / 1000.0), np.cos(line[2] / 1000.0))
                        self.model.update_position(x_pos, y_pos, theta)
                        self.model.update_wheelspeed(line[3], line[4])
                        # Append data for plotting
                        self.model.append_plot_data(x_pos, y_pos, theta, time.time())
                except Exception as e:
                    if self.connection_active:
                        self.log_text.insert(tk.END, f"Error reading from port: {str(e)}\n")
                    break

    def update_plots(self):
        while self.connection_active:
            try:
                self.view.plot_ax[0].cla()
                self.view.plot_ax[1].cla()
                self.view.plot_ax[2].cla()
                
                self.view.plot_ax[0].plot(self.model.plot_time, self.model.x_data, color='b')
                self.view.plot_ax[1].plot(self.model.plot_time, self.model.y_data, color='r')
                self.view.plot_ax[2].plot(self.model.plot_time, self.model.t_data, color='y')
                
                self.view.plot1_canvas.draw()
                time.sleep(0.05)
            except Exception as e:
                print(f"Error Plotting Graphs: {str(e)}\n")

    def robot_env_plot_update(self):
        while self.connection_active:
            try:
                # Update robot body position
                self.view.robot_body_polygon.set_xy((self.model.x_position - self.model.width/2, self.model.y_position - self.model.width/2))
                self.view.robot_body_polygon.set_angle(np.rad2deg(self.model.theta))
                
                # Add robot pointer
                dx = 0.2*np.cos(self.model.theta)
                dy = 0.2*np.sin(self.model.theta)
                # if hasattr(self.view, 'robot_pointer'):
                    # self.view.robot_pointer.remove()
                self.view.robot_pointer = patches.Arrow(self.model.x_position, self.model.y_position, dx, dy, color="black", width=0.1)
                self.view.robot_env_ax.add_patch(self.view.robot_pointer)
                
                # Update plot bounds if following robot
                if self.view.follow_robot.get():
                    self.view.robot_env_ax.set_xbound(self.model.x_position - self.view.robot_env_ax_boxwidth_scale.get()/2, 
                                                  self.model.x_position + self.view.robot_env_ax_boxwidth_scale.get()/2)
                    self.view.robot_env_ax.set_ybound(self.model.y_position - self.view.robot_env_ax_boxwidth_scale.get()/2, 
                                                  self.model.y_position + self.view.robot_env_ax_boxwidth_scale.get()/2)
                self.view.robot_env_canvas.draw()
                time.sleep(0.1)
            except Exception as e:
                print(f"Error Updating Robot Env Graphs: {str(e)}\n")

    def export_buttons_enable(self, state):
        self.export_txt_button.config(state=tk.NORMAL if state else tk.DISABLED)
        self.export_csv_button.config(state=tk.NORMAL if state else tk.DISABLED)
        self.export_xml_button.config(state=tk.NORMAL if state else tk.DISABLED)

    def send_cmd_button_enable(self, state):
        self.send_cmd_button.config(state=tk.NORMAL if state else tk.DISABLED)

    def export_txt(self):
        self.data_manager.export_txt()
    def export_csv(self):
        self.data_manager.export_csv()
    def export_xml(self):
        self.data_manager.export_xml()
    def send_cmd(self):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            self.log_text.insert(tk.END, "Error: Not connected to a serial port.\n")
            return

        try:
            cmd1 = int(self.cmd_entry_1.get())
            cmd2 = int(self.cmd_entry_2.get())
        except ValueError as e:
            self.log_text.insert(tk.END, f"Invalid command value: {str(e)}\n")
            return
        except Exception as e:
            self.log_text.insert(tk.END, f"Error sending command: {str(e)}\n")
            return

        if not (-128 <= cmd1 <= 127) or not (-128 <= cmd2 <= 127):
            self.log_text.insert(tk.END, "Error: Command values must be between -128 and 127 for int8.\n")
            return

        tx_data = [np.int8(cmd1), np.int8(cmd2)]
        try:
            self.ser.write(tx_data)
            self.log_text.insert(tk.END, f"Sent commands: {tx_data}\n")
        except Exception as e:
            self.log_text.insert(tk.END, f"Error sending commands: {str(e)}\n")

        self.cmd_entry_1.delete(0, tk.END)
        self.cmd_entry_2.delete(0, tk.END)

    def simulation_data_generator(self):
        # Initialize simulated robot parameters
        sim_x = 1000
        sim_y = 1000
        sim_theta = np.pi/2
        start_time = time.time()
        counter = 0
        
        while self.simulation_active:
            try:
                # Get wheel speeds from simulation controls
                sim_omega_l = int(self.simulation_left_wheel.get())
                sim_omega_r = int(self.simulation_right_wheel.get())
                
                # Update position based on wheel speeds using differential drive kinematics
                # Time step for simulation
                dt = 0.1
                
                # Robot parameters (should match real robot)
                wheel_base = 0.3  # Distance between wheels (meters)
                
                # Calculate linear and angular velocities
                v = (sim_omega_l + sim_omega_r) / 2.0  # Linear velocity
                w = (sim_omega_r - sim_omega_l) / wheel_base  # Angular velocity
                
                # Update robot pose
                sim_theta += w * dt
                sim_x += v * np.cos(sim_theta) * dt
                sim_y += v * np.sin(sim_theta) * dt
                
                # Normalize theta to be between -pi and pi
                sim_theta = np.arctan2(np.sin(sim_theta), np.cos(sim_theta))
                
                # Create simulated data packet matching the format from the robot
                # Using the format from config.py: '<3i2b3x1B18x' (3 integers, 2 bytes, 3 padding bytes, 1 unsigned byte, 18 padding bytes)
                sim_data = struct.pack(Config.SERIAL_DATA_FORMAT, 
                    int(sim_x * 1000),      # x position (scaled to match real robot format)
                    int(sim_y * 1000),      # y position (scaled to match real robot format)
                    int(sim_theta * 1000),  # theta (orientation) (scaled to match real robot format)
                    int(sim_omega_l / Config.PPS_TO_RADS_PER_SECOND),       # left wheel speed
                    int(sim_omega_r / Config.PPS_TO_RADS_PER_SECOND),       # right wheel speed
                    int(counter)
                )
                
                # Convert packed data to tuple for data manager
                unpacked_data = struct.unpack(Config.SERIAL_DATA_FORMAT, sim_data)
                
                # Add to data manager and update model
                self.data_manager.add_entry(unpacked_data)
                # Update model with scaled values to match real robot data format
                self.model.update_position(sim_x, sim_y, sim_theta)
                self.model.update_wheelspeed(sim_omega_l, sim_omega_r)
                # Append data for plotting (using the same scaled values)
                self.model.append_plot_data(sim_x, sim_y, sim_theta, time.time())
                
                time.sleep(0.1)  # Update at 10Hz
            except Exception as e:
                print(f"Error in simulation data generator: {str(e)}\n")
                break
        
class DataManager:
    def __init__(self, log_text):
        self.log_text = log_text
        self.entries = []

    def add_entry(self, entry):
        if isinstance(entry, tuple):
            formatted_entry = " ".join(str(x) for x in entry)
        else:
            formatted_entry = str(entry)
        self.entries.append(formatted_entry)
        self.log_text.insert(tk.END, f"{datetime.datetime.now()} {formatted_entry}\n")

    def export_txt(self):
        data = "\n".join(self.entries)
        filename = f"serial_log_{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.txt"
        with open(filename, "w") as file:
            file.write(data)
        self.log_text.insert(tk.END, f"Log exported as TXT: {filename}\n")

    def export_csv(self):
        data = "\n".join(self.entries)
        filename = Config.LOG_EXPORT_DIR + Config.EXPORT_FILE_PREFIX + datetime.datetime.now().strftime('%Y%m%d%H%M%S') + Config.EXPORT_FILE_SUFFIX['csv']
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file, delimiter=",")
            for entry in self.entries:
                writer.writerow(entry.split())
        self.log_text.insert(tk.END, f"Log exported as CSV: {filename}\n")

    def export_xml(self):
        root = ET.Element("LogData")
        for entry in self.entries:
            entry_elem = ET.SubElement(root, "Entry")
            ET.SubElement(entry_elem, "Data").text = " ".join(entry.split())
        tree = ET.ElementTree(root)
        filename = Config.LOG_EXPORT_DIR + Config.EXPORT_FILE_PREFIX + datetime.datetime.now().strftime('%Y%m%d%H%M%S') + Config.EXPORT_FILE_SUFFIX['xml']
        tree.write(filename)
        self.log_text.insert(tk.END, f"Log exported as XML: {filename}\n")

if __name__ == "__main__":
    try:
        model = RobotModel()
        view = RobotView(tk.Tk())
        controller = RobotController(model, view)
        view.master.mainloop()
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected Ending Program.")
