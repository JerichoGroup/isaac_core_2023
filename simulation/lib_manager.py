"""This file defines a manager for simulation libraries."""

# ==================== imports ====================
from typing import Dict, List, Type
import threading

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
