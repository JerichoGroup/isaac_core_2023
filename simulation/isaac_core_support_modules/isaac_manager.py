"""this file defines a class to be used as a context manager for isaac sim"""

# ==================== imports ====================
import os
import time
import itertools
import subprocess


# ==================== Helper - wait with msg (animated) ====================
def wait_with_msg_for_log(path: str,log: str,
                          load_msg: str, end_msg: str,
                          buffer: float = 0.25, timeout: float = 300):
    """print a loading msg and run a 'loading' animation until the given log is found in the process output"""

    start_time = time.time()
    symbols_iter = itertools.cycle(["-", "\\", "|", "/"])
    
    with open(path, "r") as file:
    
        while True:

            file.seek(0)
            content = file.read()

            if log in content:
                print(end_msg)
                break

            if time.time() - start_time > timeout:
                print(f"timeout reached while waiting for log: {log}")
                break

            print(f"{load_msg}  {next(symbols_iter)}", end="\r")
            time.sleep(buffer)


# ==================== the IsaacManager class ====================
class IsaacManager:
    """a context manager to start and stop isaac sim"""

    FLAG_MAP = {
        "usd_path": "--usd-path",
        "headless": "--headless",
        "com_ros": "--com-ros",
        "com_udp": "--com-udp",
        "distance_sensor": "--distance-sensor",
        "bbox_publisher": "--bbox-publisher",
        "sat": "--sat"
    }
    
    def __init__(self, usd_path: str = "./usd/maps/earth/earth.usda", headless: bool = False,
                 com_ros: bool = False, com_udp: bool = False, distance_sensor: bool = False,
                 bbox_publisher: bool = False, sat: bool = False):
        """initialize the context manager with the command to start isaac sim"""

        self.flags = {
            "usd_path": usd_path,
            "headless": headless,
            "com_ros": com_ros,
            "com_udp": com_udp,
            "distance_sensor": distance_sensor,
            "bbox_publisher": bbox_publisher,
            "sat": sat
        }
        self.process = None
        self.isaac_core_cmd = ["/home/ofer/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh", "./simulation/main_sim.py"]

    
    def _build_isaac_core_cmd(self) -> None:
        """build the isaac core command with the given flags"""

        for key, value in self.flags.items():

            if not value:
                continue

            flag = self.FLAG_MAP[key]

            if isinstance(value, str):
                self.isaac_core_cmd.extend([flag, value])
            elif isinstance(value, bool) and value:
                self.isaac_core_cmd.append(flag)


    def get_log_path(self) -> str:
        """get the path to the last opened isaac sim log file"""

        log_dir = os.path.expanduser("~/.nvidia-omniverse/logs/Kit/Isaac-Sim/2023.1")
        files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".log")]

        if not files:
            raise FileNotFoundError("No IsaacSim log files found.")
        
        return max(files, key=os.path.getctime)
    

    def __enter__(self):
        """starts isaac sim and wait for it to finish loading"""

        print("starting IsaacSim...")
        self._build_isaac_core_cmd()
        cmd_str = " ".join(self.isaac_core_cmd)
        print (f"current isaac cmd: |{cmd_str}|")


        self.process = subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", f"{cmd_str}; exec bash"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        log_path = self.get_log_path()
        print(f"Monitoring log file: {log_path}")
        time.sleep(5)  # give some time for the log file to be created
        wait_with_msg_for_log(log_path, "rclpy loaded", "waiting for IsaacSim to load...", "IsaacSim loaded!")
        time.sleep(50)

        return self
    

    def __exit__(self, exc_type, exc_value, exs_traceback):
        """kill all isaac sim precesses after exiting the context"""

        if exc_type:
            print(f"an exception occurred while in IsaacManager context: {exc_value}")

        print("closing IsaacSim...")
        subprocess.run(["pkill", "-f", "main_sim.py"])


if __name__ == "__main__":
    with IsaacManager(com_ros=True):
        print("IsaacSim is running...")
        time.sleep(5)
    print("IsaacSim has been closed.")
