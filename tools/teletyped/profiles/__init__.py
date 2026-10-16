"""Register vehicle statistics profiles here; the service remains car agnostic."""

from openpilot.tools.teletyped.profiles.base import SteeringStatsConfig, VehicleStatsProfile
from openpilot.tools.teletyped.profiles.hyundai_i30 import HyundaiI30StatsProfile
from openpilot.tools.teletyped.profiles.lexus_ls600h import LexusLS600hStatsProfile

VEHICLE_PROFILES: tuple[VehicleStatsProfile, ...] = (HyundaiI30StatsProfile(), LexusLS600hStatsProfile())


def select_vehicle_profile(car_name: str = "", fingerprint: str = "") -> VehicleStatsProfile | None:
  for profile in VEHICLE_PROFILES:
    if profile.matches(car_name, fingerprint):
      return profile
  return None


__all__ = ["SteeringStatsConfig", "VehicleStatsProfile", "select_vehicle_profile"]
