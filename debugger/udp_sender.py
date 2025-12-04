"""this file implements a UDP sender GUI for controlling camera position in Isaac Sim"""

# ==================== imports ====================
import math
import time
import struct
import socket
import threading
import tkinter as tk
from typing import List


# ==================== consts ====================
PACKET_FORMAT = "<6d"
HEADER1 = 0xAC
HEADER2 = 0xDC
SEND_RATE_HZ = 10


# ==================== socket Setup ====================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


#==================== the CameraControlGUI class ====================
class CameraControlGUI:
    """this class implements the GUI for controlling camera position via UDP packets"""
    
    def __init__(self, root: tk.Tk) -> None:
        """initialize the GUI for controlling camera position via UDP packets"""

        self.root = root
        self.root.title("Isaac Sim UDP Camera Control")

        # Default values
        self.defaults = {
            "Latitude": 32.22481,
            "Longitude": 35.25621,
            "Altitude": 1000.0,
            "Roll [deg]": 0.0,
            "Pitch [deg]": 0.0,
            "Yaw [deg]": 0.0,
            "IP Address": "127.0.0.1",
            "UDP Port": 33333
        }

        self.fields = {k: tk.DoubleVar(value=v) if isinstance(v, float) else tk.IntVar(value=v) if isinstance(v, int) else tk.StringVar(value=v)
                       for k, v in self.defaults.items()}
        self.locks = {k: tk.BooleanVar(value=False) for k in self.defaults if isinstance(self.defaults[k], float)}

        self.step_sizes = {
            "Latitude": 0.001,
            "Longitude": 0.001,
            "Altitude": 5.0,
            "Roll [deg]": 5.0,
            "Pitch [deg]": 5.0,
            "Yaw [deg]": 5.0
        }

        # GUI layout
        for i, (label, var) in enumerate(self.fields.items()):
            tk.Label(root, text=label).grid(row=i, column=0, sticky="w")

            if label in self.step_sizes:
                tk.Checkbutton(root, variable=self.locks[label]).grid(row=i, column=1)
                tk.Button(root, text="◀", width=2, command=lambda l=label: self.adjust_value(l, -1)).grid(row=i, column=2)
                tk.Entry(root, textvariable=var, width=15).grid(row=i, column=3)
                tk.Button(root, text="▶", width=2, command=lambda l=label: self.adjust_value(l, 1)).grid(row=i, column=4)
            else:
                tk.Entry(root, textvariable=var, width=20).grid(row=i, column=2, columnspan=3)

        # Control buttons
        self.pause = False
        self.pause_button = tk.Button(root, text="Pause", command=self.toggle_pause)
        self.pause_button.grid(row=len(self.fields), column=0, columnspan=2, pady=5)

        tk.Button(root, text="Reset to Default", command=self.reset_fields).grid(row=len(self.fields), column=2, columnspan=3, pady=5)

        # Packet log
        tk.Label(root, text="Last Packets Sent:").grid(row=len(self.fields)+1, column=0, columnspan=5, sticky="w")
        self.packet_log = tk.Text(root, height=3, width=100, state="disabled", bg="#f0f0f0")
        self.packet_log.grid(row=len(self.fields)+2, column=0, columnspan=5, padx=5, pady=(0, 10))

        # Packet structure table
        tk.Label(root, text="Packet Structure").grid(row=len(self.fields)+3, column=0, columnspan=5, sticky="w", pady=(10, 0))
        structure = [
            "Byte Index | Field      | Size[bytes] | Type     | Description",
            "0          | Header1    | 1           | uint8    | Fixed: 0xAC",
            "1          | Header2    | 1           | uint8    | Fixed: 0xDC",
            "2-9        | latitude   | 8           | float64  | world latitude",
            "10-17      | longitude  | 8           | float64  | world longitude",
            "18-25      | altitude   | 8           | float64  | world alt (sea=0)",
            "26-33      | roll       | 8           | float64  | world axis roll [deg]",
            "34-41      | pitch      | 8           | float64  | world axis pitch [deg]",
            "42-49      | yaw        | 8           | float64  | world axis yaw [deg]",
            "50         | Checksum   | 1           | uint8    | XOR of payload bytes"
        ]

        self.structure_box = tk.Text(root, height=len(structure), width=100, state="disabled", bg="#e8e8e8")
        self.structure_box.grid(row=len(self.fields)+4, column=0, columnspan=5, padx=5, pady=(0, 10))
        self.structure_box.config(state="normal")
        self.structure_box.insert(tk.END, "\n".join(structure))
        self.structure_box.config(state="disabled")

        self.running = True
        threading.Thread(target=self.send_loop, daemon=True).start()
        threading.Thread(target=self.send_initial_packet, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)


    def adjust_value(self, label: str, direction: int) -> None:
        """adjust the value of a numeric field by its step size in the given direction"""

        if self.locks[label].get():
            return
        step = self.step_sizes[label]
        current = self.fields[label].get()
        self.fields[label].set(round(current + direction * step, 6))


    def toggle_pause(self) -> None:
        """toggle the pause/resume state of packet sending"""

        self.pause = not self.pause
        self.pause_button.config(text="Resume" if self.pause else "Pause")


    def reset_fields(self) -> None:
        """reset all the input fields to their default values"""

        for label, default in self.defaults.items():
            self.fields[label].set(default)

    def send_initial_packet(self) -> None:
        """send a single packet after a short delay on startup"""

        time.sleep(0.5)
        self.send_packet()


    def send_loop(self) -> None:
        """continuously send packets at the defined rate while gui is running"""

        while self.running:
            if not self.pause:
                self.send_packet()
            time.sleep(1.0 / SEND_RATE_HZ)


    def send_packet(self) -> None:
        """construct and send a UDP packet with the current field values"""

        try:
            lat = self.fields["Latitude"].get()
            lon = self.fields["Longitude"].get()
            alt = self.fields["Altitude"].get()
            roll = math.radians(self.fields["Roll [deg]"].get())
            pitch = math.radians(self.fields["Pitch [deg]"].get())
            yaw = math.radians(self.fields["Yaw [deg]"].get())
            ip = self.fields["IP Address"].get()
            port = self.fields["UDP Port"].get()

            fields = [lat, lon, alt, roll, pitch, yaw]
            payload = struct.pack(PACKET_FORMAT, *fields)

            checksum = 0
            for byte in payload:
                checksum ^= byte

            packet = bytes([HEADER1, HEADER2]) + payload + bytes([checksum])
            sock.sendto(packet, (ip, port))

            self.update_packet_log(fields)

        except Exception as e:
            print(f"Error sending packet: {e}")


    def update_packet_log(self, fields: List[float]) -> None:
        """update the packet log display with the latest sent packet values"""

        msg = f"lat={fields[0]:.6f}, lon={fields[1]:.6f}, alt={fields[2]:.2f}, roll={fields[3]:.4f}, pitch={fields[4]:.4f}, yaw={fields[5]:.4f}"
        self.packet_log.config(state="normal")
        lines = self.packet_log.get("1.0", tk.END).strip().split("\n")
        lines.append(msg)
        if len(lines) > 3:
            lines = lines[-3:]
        self.packet_log.delete("1.0", tk.END)
        self.packet_log.insert(tk.END, "\n".join(lines))
        self.packet_log.config(state="disabled")


    def on_close(self) -> None:
        """handle GUI window close event"""

        self.running = False
        self.root.destroy()


# ==================== main func ====================
def main() -> None:
    """run the UDP sender GUI application"""

    root = tk.Tk()
    app = CameraControlGUI(root)
    root.mainloop()


# ==================== run the main ====================
if __name__ == "__main__":
    main()
