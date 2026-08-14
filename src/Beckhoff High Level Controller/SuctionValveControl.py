import tkinter as tk
from tkinter import messagebox
import pyads

# PLC Configuration
PLC_AMS = "5.155.180.125.1.1"  
PLC_PORT = 851  

class PLCApp:
    def __init__(self, root):
        self.root = root
        self.root.title("PLC Valve Test App")
        self.root.geometry("300x150")

        # Initialize connection variable
        self.plc = None
        self.is_connected = False

        # Create UI components
        self.create_widgets()

    def create_widgets(self):
        # Connection status label
        self.status_label = tk.Label(self.root, text="Not connected", fg="red")
        self.status_label.pack(pady=10)

        # Connect button
        self.connect_button = tk.Button(self.root, text="Connect", command=self.connect_plc)
        self.connect_button.pack(pady=5)

        # Test valve control button
        self.test_button = tk.Button(self.root, text="Test Valve", command=self.test_valve, state="disabled")
        self.test_button.pack(pady=10)

    def connect_plc(self):
        """ Connect to the PLC """
        try:
            self.plc = pyads.Connection(PLC_AMS, PLC_PORT)
            self.plc.open()

            self.is_connected = True
            self.status_label.config(text="Connected to PLC", fg="green")
            self.connect_button.config(text="Disconnect", command=self.disconnect_plc)

            # Enable the test button once connected
            self.test_button.config(state="normal")

            print("Connected to PLC!")

        except Exception as e:
            self.status_label.config(text=f"Connection failed: {e}", fg="red")
            print(f"Connection failed: {e}")

    def disconnect_plc(self):
        """ Disconnect from the PLC """
        if self.plc:
            self.plc.close()
            self.is_connected = False
            self.status_label.config(text="Disconnected", fg="red")
            self.connect_button.config(text="Connect", command=self.connect_plc)

            # Disable the test button when disconnected
            self.test_button.config(state="disabled")

            print("Disconnected safely.")

    def test_valve(self):
        """ Test valve control (toggle the valve) """
        if self.is_connected:
            try:
                # Toggle the valve state
                current_valve_state = self.plc.read_by_name("MAIN.bGripperValve", pyads.PLCTYPE_BOOL)  # Replace with your symbol
                new_valve_state = not current_valve_state  # Toggle the state

                # Write the new state to the PLC
                self.plc.write_by_name("MAIN.bGripperValve", new_valve_state, pyads.PLCTYPE_BOOL)  # Replace with your symbol
                messagebox.showinfo("Valve State", f"Valve is now {'ON' if new_valve_state else 'OFF'}")

                print(f"Valve state toggled. New state: {'ON' if new_valve_state else 'OFF'}")

            except Exception as e:
                messagebox.showerror("Error", f"Failed to communicate with PLC: {e}")
                print(f"Failed to communicate with PLC: {e}")
        else:
            messagebox.showerror("Error", "Not connected to PLC")

# Run the application
if __name__ == "__main__":
    root = tk.Tk()
    app = PLCApp(root)
    root.mainloop()
