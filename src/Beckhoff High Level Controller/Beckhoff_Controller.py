import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import threading
import time
import csv
from datetime import datetime
import pyads

# Optional: matplotlib
try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# ==============================
# Beckhoff System
# ==============================
class BeckhoffSystem:
    def __init__(self):
        self.plc = None
        self.connected = False
        self.ams_net_id = ""
        self.port = 851
        self.last_error = ""

    def connect(self, ams_net_id, port=851):
        self.ams_net_id = ams_net_id
        self.port = port
        try:
            self.plc = pyads.Connection(ams_net_id, port)
            self.plc.open()
            self.plc.read_state()  # confirm connection
            self.connected = True
            return True, "Connected to PLC."
        except Exception as e:
            self.connected = False
            self.last_error = str(e)
            self.plc = None
            return False, f"Connection failed: {e}"

    def disconnect(self):
        try:
            if self.plc:
                self.plc.close()
        except Exception:
            pass
        self.connected = False
        self.plc = None

    def write_bool(self, var_name, value):
        if not self.connected:
            return
        try:
            self.plc.write_by_name(var_name, value, pyads.PLCTYPE_BOOL)
        except Exception as e:
            self.connected = False
            self.last_error = str(e)

    def write_float(self, var_name, value):
        if not self.connected:
            return
        try:
            self.plc.write_by_name(var_name, float(value), pyads.PLCTYPE_LREAL)
        except Exception as e:
            self.connected = False
            self.last_error = str(e)

    def read_bool(self, var_name):
        if not self.connected:
            return False
        try:
            return bool(self.plc.read_by_name(var_name, pyads.PLCTYPE_BOOL))
        except Exception as e:
            self.connected = False
            self.last_error = str(e)
            return False

# ==============================
# GUI
# ==============================
class BeckhoffApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Beckhoff PLC Control Panel")
        self.root.geometry("650x700")

        # System
        self.system = BeckhoffSystem()
        self.conveyor_state = False
        self.gripper_state = False
        self.auto_sequence_running = False
        self.was_connected = False
        self.connect_thread = None
        self.last_reconnect_attempt = 0

        # Data logging
        self.data_samples = []
        self.last_log_time = 0

        # Tk variables
        self.speed_var = tk.DoubleVar(value=50.0)
        self.auto_reconnect_var = tk.BooleanVar(value=True)
        self.auto_mode_var = tk.BooleanVar(value=False)

        self._init_ui()

        # Polling
        self.root.after(500, self.poll_sensor)  # slower polling to reduce connect issues

    # ---------- UI ----------
    def _init_ui(self):
        # Connection frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="AMS Net ID:").pack(side="left")
        self.ams_entry = ttk.Entry(conn_frame, width=20)
        self.ams_entry.insert(0, "5.155.180.125.1.1")
        self.ams_entry.pack(side="left", padx=5)

        ttk.Label(conn_frame, text="Port:").pack(side="left")
        self.port_entry = ttk.Entry(conn_frame, width=6)
        self.port_entry.insert(0, "851")
        self.port_entry.pack(side="left", padx=5)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)

        ttk.Checkbutton(conn_frame, text="Auto reconnect", variable=self.auto_reconnect_var).pack(side="left", padx=5)

        self.lbl_conn_status = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.lbl_conn_status.pack(side="left", padx=5)

        self.status_canvas = tk.Canvas(conn_frame, width=16, height=16, highlightthickness=0)
        self.status_canvas.pack(side="left", padx=5)
        self.status_led = self.status_canvas.create_oval(2, 2, 14, 14, fill="red")

        # Conveyor
        conv_frame = ttk.LabelFrame(self.root, text="Conveyor Control", padding=10)
        conv_frame.pack(fill="x", padx=10, pady=5)
        self.btn_conveyor = ttk.Button(conv_frame, text="Start Conveyor", command=self.toggle_conveyor)
        self.btn_conveyor.pack(fill="x", pady=5)
        ttk.Label(conv_frame, text="Speed Setpoint (%):").pack(anchor="w")
        ttk.Scale(conv_frame, from_=0, to=100, orient="horizontal",
                  variable=self.speed_var, command=self.update_speed).pack(fill="x", pady=5)
        self.lbl_speed = ttk.Label(conv_frame, text="50.0%")
        self.lbl_speed.pack(anchor="e")

        # Gripper
        grip_frame = ttk.LabelFrame(self.root, text="Gripper Control", padding=10)
        grip_frame.pack(fill="x", padx=10, pady=5)
        self.btn_gripper = ttk.Button(grip_frame, text="Activate Suction", command=self.toggle_gripper)
        self.btn_gripper.pack(fill="x", pady=5)

        # Sensor indicator
        sensor_frame = ttk.LabelFrame(self.root, text="Product Sensor", padding=10)
        sensor_frame.pack(fill="x", padx=10, pady=5)
        self.canvas_sensor = tk.Canvas(sensor_frame, width=40, height=40)
        self.canvas_sensor.pack(side="left", padx=10)
        self.sensor_light = self.canvas_sensor.create_oval(5, 5, 35, 35, fill="gray")
        self.lbl_sensor_text = ttk.Label(sensor_frame, text="No Product")
        self.lbl_sensor_text.pack(side="left", padx=10)

        # E-STOP
        safety_frame = ttk.LabelFrame(self.root, text="Safety", padding=10)
        safety_frame.pack(fill="x", padx=10, pady=5)
        self.btn_estop = ttk.Button(safety_frame, text="EMERGENCY STOP", command=self.emergency_stop)
        self.btn_estop.pack(fill="x", pady=5)

        # Auto mode
        auto_frame = ttk.LabelFrame(self.root, text="Auto Control", padding=10)
        auto_frame.pack(fill="x", padx=10, pady=5)
        ttk.Checkbutton(auto_frame, text="Enable Auto Pick Cycle", variable=self.auto_mode_var).pack(anchor="w")

        # Data & logging
        data_frame = ttk.LabelFrame(self.root, text="Data & Logging", padding=10)
        data_frame.pack(fill="x", padx=10, pady=5)
        ttk.Button(data_frame, text="Export CSV", command=self.export_csv).pack(side="left", padx=5)
        ttk.Button(data_frame, text="Show Speed Graph", command=self.show_speed_graph).pack(side="left", padx=5)

        # Log area
        log_frame = ttk.LabelFrame(self.root, text="System Log", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.log_area = scrolledtext.ScrolledText(log_frame, height=10, state="disabled")
        self.log_area.pack(fill="both", expand=True)

    # ---------- Logging ----------
    def log(self, message):
        self.log_area.config(state="normal")
        self.log_area.insert(tk.END, f"> {message}\n")
        self.log_area.see(tk.END)
        self.log_area.config(state="disabled")

    def set_led(self, color):
        self.status_canvas.itemconfig(self.status_led, fill=color)

    # ---------- Connection ----------
    def toggle_connection(self):
        if not self.system.connected:
            self.start_connect()
        else:
            self.system.disconnect()
            self.was_connected = False
            self.lbl_conn_status.config(text="Disconnected", foreground="red")
            self.set_led("red")
            self.btn_connect.config(text="Connect")
            self.log("Disconnected.")

    def start_connect(self):
        ams = self.ams_entry.get().strip()
        try:
            port = int(self.port_entry.get().strip())
        except:
            port = 851
        self.lbl_conn_status.config(text="Connecting...", foreground="orange")
        self.set_led("yellow")

        def worker():
            success, msg = self.system.connect(ams, port)
            self.root.after(0, lambda: self.finish_connect(success, msg))

        threading.Thread(target=worker, daemon=True).start()

    def finish_connect(self, success, msg):
        if success:
            self.lbl_conn_status.config(text="Connected", foreground="green")
            self.set_led("green")
            self.btn_connect.config(text="Disconnect")
            self.log(msg)
            self.was_connected = True
        else:
            self.lbl_conn_status.config(text="Disconnected", foreground="red")
            self.set_led("red")
            self.log(msg)

    def reconnect_if_needed(self):
        if not self.system.connected and self.auto_reconnect_var.get():
            now = time.time()
            if now - self.last_reconnect_attempt > 5:
                self.last_reconnect_attempt = now
                self.log("Auto reconnecting...")
                self.start_connect()

    # ---------- Conveyor ----------
    def toggle_conveyor(self):
        if not self.system.connected:
            self.log("Error: Not connected.")
            return
        self.conveyor_state = not self.conveyor_state
        self.system.write_bool("MAIN.bConveyorRun", self.conveyor_state)
        self.btn_conveyor.config(text="Stop Conveyor" if self.conveyor_state else "Start Conveyor")
        if self.conveyor_state:
            self.update_speed(self.speed_var.get())
            self.log("Conveyor started.")
        else:
            self.log("Conveyor stopped.")

    def update_speed(self, val):
        speed = float(val)
        self.lbl_speed.config(text=f"{speed:.1f}%")
        if self.system.connected and self.conveyor_state:
            self.system.write_float("MAIN.fConveyorSpeed", speed)

    # ---------- Gripper ----------
    def toggle_gripper(self):
        if not self.system.connected:
            self.log("Error: Not connected.")
            return
        self.gripper_state = not self.gripper_state
        self.system.write_bool("MAIN.bGripperValve", self.gripper_state)
        self.btn_gripper.config(text="Release Suction" if self.gripper_state else "Activate Suction")
        self.log("Suction activated." if self.gripper_state else "Suction released.")

    # ---------- E-STOP ----------
    def emergency_stop(self):
        if not self.system.connected:
            self.log("Error: Not connected.")
            return
        self.log("EMERGENCY STOP pressed!")
        if self.conveyor_state:
            self.toggle_conveyor()
        if self.gripper_state:
            self.toggle_gripper()
        self.system.write_bool("MAIN.bEmergencyStop", True)
        self.root.after(300, lambda: self.system.write_bool("MAIN.bEmergencyStop", False))

    # ---------- Auto Cycle ----------
    def start_auto_cycle(self):
        if self.auto_sequence_running or not self.system.connected:
            return
        self.auto_sequence_running = True
        self.log("Auto cycle started.")
        if not self.conveyor_state:
            self.toggle_conveyor()
        self.root.after(2000, self.auto_step2)

    def auto_step2(self):
        if self.conveyor_state:
            self.toggle_conveyor()
        if not self.gripper_state:
            self.toggle_gripper()
        self.log("Product picked.")
        self.root.after(1500, self.auto_step3)

    def auto_step3(self):
        if self.gripper_state:
            self.toggle_gripper()
        self.log("Auto cycle complete.")
        self.auto_sequence_running = False

    # ---------- Polling ----------
    def poll_sensor(self):
        if self.was_connected and not self.system.connected:
            self.log("Connection lost.")
            self.lbl_conn_status.config(text="Disconnected", foreground="red")
            self.set_led("red")
            self.was_connected = False
        self.reconnect_if_needed()
        sensor_val = self.system.read_bool("MAIN.bProductDetected") if self.system.connected else False
        self.canvas_sensor.itemconfig(self.sensor_light, fill="#00FF00" if sensor_val else "gray")
        self.lbl_sensor_text.config(text="PRODUCT DETECTED" if sensor_val else "No Product")
        if self.system.connected and self.auto_mode_var.get() and sensor_val and not self.auto_sequence_running:
            self.start_auto_cycle()
        now = time.time()
        if now - self.last_log_time > 1:
            self.last_log_time = now
            self.data_samples.append((now, float(self.speed_var.get()), int(self.conveyor_state), int(sensor_val)))
        self.root.after(500, self.poll_sensor)

    # ---------- CSV Export ----------
    def export_csv(self):
        if not self.data_samples:
            self.log("No data to export.")
            return
        filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        if not filename:
            return
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_iso", "elapsed_s", "speed", "conveyor", "sensor"])
            t0 = self.data_samples[0][0]
            for t, speed, conv, sensor in self.data_samples:
                writer.writerow([datetime.fromtimestamp(t).isoformat(), f"{t-t0:.2f}", speed, conv, sensor])
        self.log(f"Data exported: {filename}")

    # ---------- Graph ----------
    def show_speed_graph(self):
        if not MATPLOTLIB_AVAILABLE:
            self.log("Matplotlib not installed.")
            return
        if len(self.data_samples) < 2:
            self.log("Not enough data yet.")
            return
        t0 = self.data_samples[0][0]
        times = [s[0]-t0 for s in self.data_samples]
        speeds = [s[1] for s in self.data_samples]
        win = tk.Toplevel(self.root)
        win.title("Speed Graph")
        win.geometry("600x400")
        fig, ax = plt.subplots(figsize=(6,4))
        ax.plot(times, speeds)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (%)")
        ax.grid(True)
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

# ==============================
# RUN
# ==============================
if __name__ == "__main__":
    root = tk.Tk()
    app = BeckhoffApp(root)
    root.mainloop()
