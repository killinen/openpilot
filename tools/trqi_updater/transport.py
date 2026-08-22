from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class CanFrame:
  address: int
  data: bytes
  src: int
  bus_time: int = 0


class CanTransport(ABC):
  @abstractmethod
  def send(self, address: int, data: bytes, bus: int) -> None:
    pass

  @abstractmethod
  def recv(self, timeout: float) -> CanFrame | None:
    pass

  @abstractmethod
  def flush(self) -> None:
    pass

  @abstractmethod
  def close(self) -> None:
    pass
