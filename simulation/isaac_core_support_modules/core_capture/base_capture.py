"""This file Defines a base class for capture modules in Isaac Core."""

# ==================== Imports ====================
from abc import ABC, abstractmethod


# ==================== The BaseCapture class ====================
class BaseCapture(ABC):
    """Abstract base class for capture modules in Isaac Core."""

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