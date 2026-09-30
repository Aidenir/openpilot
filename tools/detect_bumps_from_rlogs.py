#!/usr/bin/env python3
"""Run the onroad speed bump detector over recorded drives.

Seeds mapd's suggested_speed_bumps.json from past rlogs, using exactly the
detector the car runs (frogpilot/system/speed_bump_detector.py). Detections
within 15 m are clustered; each route counts as one drive, so a location felt on
two routes is what the car would promote with the default settings.

  tools/detect_bumps_from_rlogs.py /data/media/0/realdata -o suggested_speed_bumps.json
  tools/detect_bumps_from_rlogs.py ~/bumpdata/realdata --mapped tiles.geojson --tracks -o out.geojson

--mapped takes a `mapd export-geojson` dump; detections within 10 m of its
traffic calming are reported as known-bump hits instead of suggestions (the
tuning ground truth). --tracks adds each route's GPS track as a `kind: "track"`
LineString for tools/map_viewer.html; mapd ignores non-point features.
To use the result on the device, copy it to /data/media/0/osm/suggested_speed_bumps.json
while mapd is not running (it rewrites the file from memory), or merge by hand.
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from openpilot.frogpilot.system.speed_bump_detector import DEFAULT_THRESHOLD, SpeedBumpDetector

MERGE_DISTANCE = 15.0  # m, as mapd's SuggestionMergeDistance
KNOWN_DISTANCE = 10.0  # m, as mapd's KnownBumpDistance


def distance(a, b):
  lat = math.radians((a[0] + b[0]) / 2)
  return math.hypot(math.radians(b[0] - a[0]) * 6371000, math.radians(b[1] - a[1]) * 6371000 * math.cos(lat))


def find_rlogs(paths):
  found = []
  for p in map(Path, paths):
    if p.is_file():
      found.append(p)
    else:
      found += [f for f in p.rglob("rlog*") if f.is_file()]
  return found


def route_and_segment(rlog):
  # <route>--<segment>/rlog.zst, as comma devices store them
  name = rlog.parent.name
  route, _, seg = name.rpartition("--")
  return (route, int(seg)) if seg.isdigit() else (name, 0)


def interp(t, ts, vs):
  if not ts:
    return None
  if t <= ts[0]:
    return vs[0] if ts[0] - t < 2 else None
  for i in range(1, len(ts)):
    if ts[i] >= t:
      f = (t - ts[i - 1]) / (ts[i] - ts[i - 1]) if ts[i] > ts[i - 1] else 0
      a, b = vs[i - 1], vs[i]
      return tuple(x + (y - x) * f for x, y in zip(a, b))
  return vs[-1] if t - ts[-1] < 2 else None


def detect_route(rlogs, threshold, az_floor):
  """Returns (events, gps track) for one route's segments, in order."""
  from openpilot.tools.lib.logreader import LogReader

  detector = SpeedBumpDetector(threshold=threshold, az_pp_floor=az_floor)
  events, gps_t, gps_v = [], [], []
  for rlog in rlogs:
    for m in LogReader(str(rlog)):
      w = m.which()
      if w == "carState":
        detector.set_speed(m.carState.vEgo)
      elif w == "accelerometer" and m.accelerometer.which() == "acceleration":
        detector.add_accel(m.accelerometer.timestamp * 1e-9, m.accelerometer.acceleration.v[0])
      elif w == "gyroscope" and m.gyroscope.which() == "gyroUncalibrated":
        e = detector.add_gyro(m.gyroscope.timestamp * 1e-9, m.gyroscope.gyroUncalibrated.v[1])
        if e is not None:
          events.append(e)
      elif w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if getattr(g, "hasFix", True) and g.latitude != 0:
          gps_t.append(m.logMonoTime * 1e-9)
          gps_v.append((g.latitude, g.longitude, g.unixTimestampMillis * 1e-3, g.bearingDeg))
  return events, gps_t, gps_v


def load_mapped(path):
  pts = []
  for f in json.load(open(path)).get("features", []):
    p = f.get("properties", {})
    if f["geometry"]["type"] == "Point" and (p.get("kind") == "trafficCalming" or p.get("source") in ("user", "detected")):
      lon, lat = f["geometry"]["coordinates"][:2]
      pts.append((lat, lon))
  return pts


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("paths", nargs="+", help="rlog files or folders of <route>--<segment>/rlog*")
  ap.add_argument("-o", "--output", default="-", help="GeoJSON to write (default stdout)")
  ap.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD, help="rad/s, as SpeedBumpDetectThreshold")
  ap.add_argument("--az-floor", type=float, default=0.0, help="minimum vertical accel peak-to-peak, m/s^2")
  ap.add_argument("--mapped", help="mapd export-geojson dump; detections near its bumps are known hits")
  ap.add_argument("--tracks", action="store_true", help="also write each route's GPS track as a kind=track LineString")
  args = ap.parse_args()

  routes = defaultdict(list)
  for rlog in find_rlogs(args.paths):
    route, seg = route_and_segment(rlog)
    routes[route].append((seg, rlog))
  mapped = load_mapped(args.mapped) if args.mapped else []

  suggestions, tracks = [], []
  known = unplaced = total = 0
  for route, segs in sorted(routes.items()):
    events, gps_t, gps_v = detect_route([r for _, r in sorted(segs)], args.threshold, args.az_floor)
    print(f"{route}: {len(segs)} segments, {len(events)} detections", file=sys.stderr)
    if args.tracks and gps_v:
      tracks.append({"type": "Feature", "geometry": {"type": "LineString", "coordinates": [[v[1], v[0]] for v in gps_v]},
                     "properties": {"kind": "track", "route": route}})
    for e in events:
      total += 1
      at = interp(e.t, gps_t, gps_v)
      if at is None:
        unplaced += 1
        continue
      pos = (at[0], at[1])
      if any(distance(pos, k) <= KNOWN_DISTANCE for k in mapped):
        known += 1
        continue
      seen = datetime.fromtimestamp(at[2], timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
      near = [s for s in suggestions if distance(pos, (s["lat"], s["lon"])) <= MERGE_DISTANCE]
      if near:
        s = min(near, key=lambda s: distance(pos, (s["lat"], s["lon"])))
        n = s["hits"]
        s["lat"], s["lon"] = (s["lat"] * n + pos[0]) / (n + 1), (s["lon"] * n + pos[1]) / (n + 1)
        s["pitches"].append(e.pitch)
        s["max_azpp"] = max(s["max_azpp"], e.az_pp)
        s["hits"] += 1
        s["last_seen"] = max(s["last_seen"], seen)
        if route not in s["drives"]:
          s["drives"].append(route)
      else:
        suggestions.append({"lat": pos[0], "lon": pos[1], "bearing": at[3], "hits": 1, "drives": [route], "pitches": [e.pitch],
                            "max_azpp": e.az_pp, "first_seen": seen, "last_seen": seen, "speed_kmh": e.v_ego * 3.6})

  features = [{
    "type": "Feature",
    "geometry": {"type": "Point", "coordinates": [round(s["lon"], 7), round(s["lat"], 7)]},
    "properties": {
      "traffic_calming": "bump", "source": "suggested", "status": "pending", "bearing": round(s["bearing"], 1),
      "created": s["first_seen"], "hits": s["hits"], "drive_count": len(s["drives"]), "drives": s["drives"],
      "first_seen": s["first_seen"], "last_seen": s["last_seen"],
      "max_pitch": round(max(s["pitches"]), 4), "mean_pitch": round(sum(s["pitches"]) / len(s["pitches"]), 4),
      "max_azpp": round(s["max_azpp"], 3), "speed_kmh": round(s["speed_kmh"], 1),
    },
  } for s in suggestions] + tracks

  print(f"{total} detections: {known} on known bumps, {unplaced} without GPS, "
        f"{len(suggestions)} suggested locations ({sum(len(s['drives']) >= 2 for s in suggestions)} on 2+ drives)", file=sys.stderr)
  out = json.dumps({"type": "FeatureCollection", "features": features}, indent=1)
  if args.output == "-":
    print(out)
  else:
    Path(args.output).write_text(out + "\n")


if __name__ == "__main__":
  main()
