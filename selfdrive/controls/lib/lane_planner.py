import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from system.hardware import EON, TICI
from system.swaglog import cloudlog


TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera. Empirically 
# the model knows the difference between TICI and EON
# so a path offset is not needed
PATH_OFFSET = 0.00
if EON:
  CAMERA_OFFSET = -0.06
elif TICI:
  CAMERA_OFFSET = 0.04
else:
  CAMERA_OFFSET = 0.0


class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.edge_t = np.zeros((TRAJECTORY_SIZE,))
    self.left_edge_y = np.zeros((TRAJECTORY_SIZE,))
    self.right_edge_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.edge_data_valid = False

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

  def parse_model(self, md):
    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

    self.edge_data_valid = False
    road_edges = md.roadEdges
    if len(road_edges) >= 2 and len(road_edges[0].t) == TRAJECTORY_SIZE and len(road_edges[1].t) == TRAJECTORY_SIZE:
      self.edge_t = np.array(road_edges[0].t)
      self.left_edge_y = np.array(road_edges[0].y) + self.camera_offset
      self.right_edge_y = np.array(road_edges[1].y) + self.camera_offset

      self.edge_data_valid = True

  def get_d_path(self, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz

  def get_road_edge_path(self, v_ego, path_t, path_xyz):
    if not self.edge_data_valid:
      return path_xyz, False

    width_pts = self.right_edge_y - self.left_edge_y
    if not np.all(np.isfinite(width_pts)):
      return path_xyz, False

    avg_width = float(np.clip(np.mean(width_pts), 2.5, 6.0))
    center_y = (self.left_edge_y + self.right_edge_y) / 2.0

    valid = np.isfinite(self.edge_t) & np.isfinite(center_y)
    if not np.any(valid):
      return path_xyz, False

    lane_path_y_interp = np.interp(path_t, self.edge_t[valid], center_y[valid])

    path_xyz[:, 1] = lane_path_y_interp

    # Treat road edges as high-confidence lane guidance
    self.lane_width_estimate.update(avg_width)
    self.lane_width_certainty.update(1.0)
    self.lane_width = float(self.lane_width_estimate.x)
    self.d_prob = 1.0

    return path_xyz, True
