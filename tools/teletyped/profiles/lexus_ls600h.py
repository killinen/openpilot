"""LS600h engagement statistics with integrated distance and no intervention signal."""

from openpilot.tools.teletyped.profiles.base import VehicleStatsProfile


class LexusLS600hStatsProfile(VehicleStatsProfile):
  name = "lexus_ls600h_2013_17"
  car_names = frozenset({"toyota"})
  fingerprints = frozenset({"LEXUS_LS600H", "LEXUS LS 600H 2013-17"})
  steering_mode = "hrr_torque"
