"""This file Defines a base class for capture modules in Isaac Core."""

# ==================== Imports ====================
import threading
from abc import ABC, abstractmethod

import rclpy
from rclpy.executors import MultiThreadedExecutor


# ==================== The BaseCapture class ====================
class BaseCapture(ABC):
    """Abstract base class for capture modules in Isaac Core."""

    def __init__(self) -> None:
        """Initialize the BaseCapture class."""
        
        self._executor = MultiThreadedExecutor()
        self._spin_thread = None

    @abstractmethod
    def start_capture(self) -> None:
        """Start the capture process."""
        pass

    @abstractmethod
    def stop_capture(self) -> None:
        """Stop the capture process."""
        pass

    @abstractmethod
    def save_data_to(self, file_path: str) -> None:
        """Saves the captured data to the specified file path."""
        pass

    def spin(self) -> None:
        """Spin the Node in a separate thread to process incoming messages."""

        if self._spin_thread is None:
            self._executor.add_node(self)
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

    def shutdown(self) -> None:
        """Shutdown the Node and clean up resources."""
        self._executor.shutdown()
        self.destroy_node()
        self._spin_thread = None
