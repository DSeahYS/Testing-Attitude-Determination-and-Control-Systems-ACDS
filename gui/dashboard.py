import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class ADCS_Dashboard:
    def __init__(self, satellite_physics, environment):
        self.satellite = satellite_physics
        self.environment = environment

        self.root = tk.Tk()
        self.root.title("ADCS Simulator Dashboard")

        # Labels for current state
        self.attitude_label = ttk.Label(self.root, text="Attitude: ")
        self.attitude_label.pack()

        self.omega_label = ttk.Label(self.root, text="Angular Velocity: ")
        self.omega_label.pack()

        self.mode_label = ttk.Label(self.root, text="Mode: ")
        self.mode_label.pack()

        # Plot for attitude over time
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()

        self.time_data = []
        self.attitude_data = []

        # Update button
        self.update_button = ttk.Button(self.root, text="Update", command=self.update_display)
        self.update_button.pack()

    def update_display(self):
        attitude, omega, mode = self.satellite.get_state()

        self.attitude_label.config(text=f"Attitude: {attitude}")
        self.omega_label.config(text=f"Angular Velocity: {omega}")
        self.mode_label.config(text=f"Mode: {mode}")

        # Simulate step
        self.satellite.step(0.1)

        # Update plot
        self.time_data.append(len(self.time_data) * 0.1)
        self.attitude_data.append(attitude[0])  # plot q0

        self.ax.clear()
        self.ax.plot(self.time_data, self.attitude_data)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Attitude q0')
        self.canvas.draw()

    def run(self):
        self.root.mainloop()