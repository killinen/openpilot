"""Vehicle statistics profiles supported by this checkout."""

from openpilot.tools.teletyped.profiles.base import SteeringStatsConfig, VehicleStatsProfile


class MotorhomeStatsProfile(VehicleStatsProfile):
  name = "motorhome_ssc"
  car_names = frozenset({"motorhome"})
  steering_mode = "legacy_ssc"
  supports_blinker_stats = False


VEHICLE_PROFILES: tuple[VehicleStatsProfile, ...] = (MotorhomeStatsProfile(),)


def select_vehicle_profile(car_name: str = "", fingerprint: str = "") -> VehicleStatsProfile | None:
  return next((profile for profile in VEHICLE_PROFILES if profile.matches(car_name, fingerprint)), None)


__all__ = ["SteeringStatsConfig", "VehicleStatsProfile", "select_vehicle_profile"]
