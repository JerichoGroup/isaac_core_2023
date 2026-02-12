"""This file defines a abstract simulation library base class."""

# ==================== Imports ====================
from abc import ABC, abstractmethod


# ==================== The SimLibBase class ====================
class SimLibBase(ABC):
    """Abstract base class for simulation libraries."""

    @abstractmethod
    def start(self) -> None:
        """Start the simulation library."""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Shutdown the simulation library."""
        pass
