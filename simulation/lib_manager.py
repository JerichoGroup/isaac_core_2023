"""This file defines a manager for simulation libraries."""

# ==================== imports ====================
import time
import json
import argparse
import threading
from typing import Dict, List, Type

from libraries.sim_lib import SimLibBase
from libraries.ros_image_to_rtp_lib import ImageRTPStreamer


# ==================== Library Registry ====================
SIM_LIB_REGISTRY: Dict[str, Type[SimLibBase]] = {
    "image_rtp": ImageRTPStreamer,
}


# ==================== the SimLibManager class ====================
class SimLibManager:
    """Manager that instantiates and controls multiple simulation libraries."""

    def __init__(self, libs_to_add: Dict[str, List]) -> None:
        """Initialize the SimLibManager with the specified libraries to start."""

        self._libs: List[SimLibBase] = []
        self._threads: List[threading.Thread] = []

        for lib_key, args in libs_to_add.items():
            cls = SIM_LIB_REGISTRY.get(lib_key)
            if cls is None:
                print(f"[SimLibManager] Unknown lib key: {lib_key}, skipping")
                continue

            try:
                lib_instance = cls(*args)
            except Exception as e:
                print(f"[SimLibManager] Failed to instantiate {lib_key}: {e}")
                continue

            self._libs.append(lib_instance)

    def start_all_libs(self) -> None:
        """Start all simulation libraries, each in its own thread."""

        for lib in self._libs:
            t = threading.Thread(target=lib.start, daemon=True)
            t.start()
            self._threads.append(t)

        print(f"[SimLibManager] Started {len(self._libs)} libs")

    def shutdown_all_libs(self) -> None:
        """Shutdown all simulation libraries."""
        for lib in self._libs:
            try:
                lib.shutdown()
            except Exception as e:
                print(f"[SimLibManager] Error shutting down lib {lib}: {e}")

        print("[SimLibManager] All libs shutdown")


# ==================== Main Function ====================
def main() -> None:
    """Main to run the SimLibManager, will be ran from main_sim in a subprocess."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--libs", type=str, required=True, help="JSON string of libraries to add")
    args = parser.parse_args()

    libs_to_add = json.loads(args.libs)
    print("[LibManager] Received libs:", libs_to_add)

    manager = SimLibManager(libs_to_add)
    manager.start_all_libs()

    print("[LibManager] Running.")

    try:
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\n[LibManager] Caught KeyboardInterrupt, shutting down...")
    except Exception as e:
        print(f"\n[LibManager] Caught exception: {e}, shutting down...")
    finally:
        manager.shutdown_all_libs()


# ==================== Entrypoint ====================
if __name__ == "__main__":
    main()
