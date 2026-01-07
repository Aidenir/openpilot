#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime
from pathlib import Path

from openpilot.common.conversions import Conversions as CV
from openpilot.frogpilot.common.frogpilot_utilities import calculate_distance_to_point
from openpilot.frogpilot.common.frogpilot_variables import params_memory

# Helper to log at module level
def _log_to_file(message):
  try:
    log_file = Path("/data/media/0/osm/traffic_calming_detections.log")
    log_file.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(log_file, 'a') as f:
      f.write(f"[{timestamp}] {message}\n")
  except Exception:
    pass

# Try to import osmium for OSM parsing
try:
  import osmium
  OSMIUM_AVAILABLE = True
  msg = "TrafficCalmingController: osmium available"
  print(msg)
  _log_to_file(msg)
except ImportError:
  OSMIUM_AVAILABLE = False
  msg = "TrafficCalmingController: osmium not available, feature will be limited"
  print(msg)
  _log_to_file(msg)


class TrafficCalmingHandler(osmium.SimpleHandler if OSMIUM_AVAILABLE else object):
  """OSM handler to extract traffic_calming features."""
  def __init__(self):
    if OSMIUM_AVAILABLE:
      super().__init__()
    self.features = []

  def node(self, n):
    """Process OSM nodes."""
    if 'traffic_calming' in n.tags:
      calming_type = n.tags['traffic_calming']
      if calming_type in ['bump', 'hump', 'table', 'cushion']:
        name = n.tags.get('name', '')
        self.features.append((n.location.lat, n.location.lon, calming_type, name))

  def way(self, w):
    """Process OSM ways."""
    if 'traffic_calming' in w.tags:
      calming_type = w.tags['traffic_calming']
      if calming_type in ['bump', 'hump', 'table', 'cushion']:
        # Get center point of way
        try:
          coords = [(n.lat, n.lon) for n in w.nodes]
          if coords:
            lat = sum(c[0] for c in coords) / len(coords)
            lon = sum(c[1] for c in coords) / len(coords)
            name = w.tags.get('name', '')
            self.features.append((lat, lon, calming_type, name))
        except osmium.InvalidLocationError:
          pass


class TrafficCalmingController:
  def __init__(self):
    self.cached_features = []  # [(lat, lon, type, name)]
    self.last_query_position = None
    self.cache_file = Path("/data/media/0/osm/traffic_calming_cache.json")
    self.log_file = Path("/data/media/0/osm/traffic_calming_detections.log")

    # Log init early (before _load_cache which also logs)
    msg = "TrafficCalmingController: __init__ called"
    print(msg)
    self._log_detection_early(msg)

    # Configuration
    self.query_radius = 500  # meters
    self.min_query_distance = 250  # meters before re-querying
    self.max_display_distance = 300  # meters
    self.bearing_tolerance = 60  # degrees (±60° = 120° cone ahead)

    # OSM data location
    self.osm_offline_path = Path("/data/media/0/osm/offline")

    # Track last detection to avoid duplicate logs
    self.last_detection = None

    # Load cached data if available
    self._load_cache()
    self._log_detection(f"TrafficCalmingController initialized with {len(self.cached_features)} cached features")

  def _load_cache(self):
    """Load previously cached traffic calming features."""
    try:
      if self.cache_file.exists():
        with open(self.cache_file, 'r') as f:
          data = json.load(f)
          self.cached_features = [tuple(item) for item in data.get('features', [])]
          msg = f"TrafficCalmingController: Loaded {len(self.cached_features)} features from cache"
          print(msg)
          self._log_detection_early(msg)
    except Exception as e:
      msg = f"TrafficCalmingController: Failed to load cache: {e}"
      print(msg)
      self._log_detection_early(msg)

  def _save_cache(self):
    """Save traffic calming features to cache."""
    try:
      self.cache_file.parent.mkdir(parents=True, exist_ok=True)
      with open(self.cache_file, 'w') as f:
        json.dump({'features': self.cached_features}, f)
    except Exception as e:
      msg = f"TrafficCalmingController: Failed to save cache: {e}"
      print(msg)
      self._log_detection(msg)

  def _log_detection_early(self, message):
    """Log detection to persistent file (early init version)."""
    try:
      log_file = Path("/data/media/0/osm/traffic_calming_detections.log")
      log_file.parent.mkdir(parents=True, exist_ok=True)
      timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      with open(log_file, 'a') as f:
        f.write(f"[{timestamp}] {message}\n")
    except Exception as e:
      print(f"TrafficCalmingController: Failed to write log: {e}")

  def _log_detection(self, message):
    """Log detection to persistent file."""
    try:
      self.log_file.parent.mkdir(parents=True, exist_ok=True)
      timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      with open(self.log_file, 'a') as f:
        f.write(f"[{timestamp}] {message}\n")
    except Exception as e:
      print(f"TrafficCalmingController: Failed to write log: {e}")

  def update(self, gps_position, v_ego, bearing):
    """
    Update traffic calming detection.

    Args:
      gps_position: dict with 'latitude' and 'longitude' in degrees
      v_ego: vehicle speed in m/s
      bearing: vehicle bearing in degrees (0-360, 0=North)
    """
    current_lat = gps_position["latitude"]
    current_lon = gps_position["longitude"]

    # Log first update to confirm code is running
    if not hasattr(self, '_first_update_logged'):
      self._log_detection(f"First update() call - lat={current_lat:.6f}, lon={current_lon:.6f}, bearing={bearing:.1f}")
      self._first_update_logged = True

    # Check if we need to query for new data
    should_query = False

    if self.last_query_position is None:
      should_query = True
    else:
      # Calculate distance from last query position
      distance_from_last_query = calculate_distance_to_point(
        self.last_query_position["latitude"] * CV.DEG_TO_RAD,
        self.last_query_position["longitude"] * CV.DEG_TO_RAD,
        current_lat * CV.DEG_TO_RAD,
        current_lon * CV.DEG_TO_RAD
      )

      # Query if we've moved far enough
      if distance_from_last_query >= self.min_query_distance:
        should_query = True

    # Fetch new data if needed
    if should_query and OSMIUM_AVAILABLE:
      self._log_detection(f"Querying OSM data at lat={current_lat:.6f}, lon={current_lon:.6f}")
      self._load_from_osm_tiles(current_lat, current_lon)
      self.last_query_position = {"latitude": current_lat, "longitude": current_lon}
      self._log_detection(f"After query: {len(self.cached_features)} features in cache")

    # Find nearest traffic calming feature ahead
    nearest_feature = self._find_nearest_ahead(current_lat, current_lon, bearing)

    # Log detection changes
    if nearest_feature:
      distance, feature_type, feature_name = nearest_feature
      detection_key = f"{feature_type}_{distance:.0f}"

      # Log if this is a new detection or distance changed significantly (>10m)
      if self.last_detection != detection_key:
        name_str = f" ({feature_name})" if feature_name else ""
        self._log_detection(f"Detected {feature_type}{name_str} at {distance:.1f}m ahead")
        self.last_detection = detection_key

      params_memory.put_float("TrafficCalmingDistance", distance)
      params_memory.put("TrafficCalmingType", feature_type)
    else:
      # Log when detection is cleared
      if self.last_detection is not None:
        self._log_detection("Traffic calming cleared")
        self.last_detection = None

      params_memory.put_float("TrafficCalmingDistance", 0.0)
      params_memory.put("TrafficCalmingType", "")

  def _load_from_osm_tiles(self, latitude, longitude):
    """
    Load traffic_calming features from local OSM tiles.
    """
    if not OSMIUM_AVAILABLE:
      msg = "TrafficCalmingController: osmium not available, cannot parse OSM tiles"
      print(msg)
      self._log_detection(msg)
      return

    if not self.osm_offline_path.exists():
      msg = f"TrafficCalmingController: OSM offline path does not exist: {self.osm_offline_path}"
      print(msg)
      self._log_detection(msg)
      return

    try:
      # Find relevant OSM tile files
      # Files are stored as coordinate bounding boxes: lat1_lon1_lat2_lon2
      # in subdirectories like: /data/media/0/osm/offline/54/10/54.000000_10.000000_54.250000_10.250000
      osm_files = [f for f in self.osm_offline_path.glob("*/*/*") if f.is_file()]

      if not osm_files:
        msg = "TrafficCalmingController: No OSM tiles found"
        print(msg)
        self._log_detection(msg)
        return

      self._log_detection(f"Found {len(osm_files)} OSM tile files")

      # Parse the OSM files (this is expensive, so we cache results)
      handler = TrafficCalmingHandler()

      # TODO: Implement proper tile selection based on GPS coordinates
      # For now, just parse the first few files
      for osm_file in osm_files[:3]:  # Limit to first 3 files for performance
        try:
          handler.apply_file(str(osm_file), locations=True)
          msg = f"Parsed {osm_file.name}, found {len(handler.features)} features"
          print(f"TrafficCalmingController: {msg}")
          self._log_detection(msg)
        except Exception as e:
          msg = f"Failed to parse {osm_file.name}: {e}"
          print(f"TrafficCalmingController: {msg}")
          self._log_detection(msg)

      # Update cached features
      self.cached_features = handler.features
      self._save_cache()
      self._log_detection(f"Cached {len(self.cached_features)} total features")

    except Exception as e:
      msg = f"Error loading OSM tiles: {e}"
      print(f"TrafficCalmingController: {msg}")
      self._log_detection(msg)

  def _find_nearest_ahead(self, current_lat, current_lon, bearing):
    """
    Find the nearest traffic calming feature ahead of the vehicle.

    Returns:
      tuple: (distance_m, type, name) or None if no feature found
    """
    nearest_distance = float('inf')
    nearest_feature = None

    for feature_lat, feature_lon, feature_type, feature_name in self.cached_features:
      # Calculate distance
      distance = calculate_distance_to_point(
        current_lat * CV.DEG_TO_RAD,
        current_lon * CV.DEG_TO_RAD,
        feature_lat * CV.DEG_TO_RAD,
        feature_lon * CV.DEG_TO_RAD
      )

      # Skip if too far
      if distance > self.max_display_distance:
        continue

      # Calculate bearing to feature
      bearing_to_feature = self._calculate_bearing(
        current_lat, current_lon, feature_lat, feature_lon
      )

      # Check if feature is ahead (within bearing tolerance)
      bearing_diff = abs(self._normalize_angle(bearing_to_feature - bearing))
      if bearing_diff > self.bearing_tolerance:
        continue

      # Update nearest if this is closer
      if distance < nearest_distance:
        nearest_distance = distance
        nearest_feature = (distance, feature_type, feature_name)

    return nearest_feature

  def _calculate_bearing(self, lat1, lon1, lat2, lon2):
    """
    Calculate bearing from point 1 to point 2 in degrees.

    Returns:
      float: bearing in degrees (0-360, 0=North)
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)

    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)

    # Normalize to 0-360
    return (bearing_deg + 360) % 360

  def _normalize_angle(self, angle):
    """
    Normalize angle to -180 to 180 range.
    """
    while angle > 180:
      angle -= 360
    while angle < -180:
      angle += 360
    return abs(angle)
