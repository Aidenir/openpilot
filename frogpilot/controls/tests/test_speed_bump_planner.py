from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant

V_CRUISE = 50 / 3.6


def run(steps, speed_bump_decel=0.0, **plant_kwargs):
  plant = Plant(speed=V_CRUISE, **plant_kwargs)
  accels = []
  for _ in range(steps):
    plant.step(v_cruise=V_CRUISE, speed_bump_decel=speed_bump_decel, **({"v_lead": 0.0} if plant_kwargs.get("lead_relevancy") else {}))
    accels.append(plant.acceleration)
  return plant, accels


def test_no_request_holds_cruise():
  plant, accels = run(40)
  assert abs(plant.speed - V_CRUISE) < 0.1
  assert min(accels) > -0.1


def test_request_brakes_at_requested_rate():
  # A cruise-speed cap alone never gets past ~-1.2 m/s^2; the bump request must reach the planner output directly
  plant, accels = run(40, speed_bump_decel=2.5)
  assert min(accels) < -2.4
  assert min(accels) >= -2.5 - 1e-3
  assert plant.speed < V_CRUISE - 2.0


def test_request_never_exceeds_accel_min():
  _, accels = run(40, speed_bump_decel=10.0)
  assert min(accels) >= -3.5 - 1e-3


def test_harder_lead_braking_wins():
  # A stopped car 20 m ahead needs far more than 0.5 m/s^2; the bump request must not soften that
  _, accels = run(20, speed_bump_decel=0.5, lead_relevancy=True, distance_lead=20.0)
  assert min(accels) < -2.0


def test_brakes_come_off_gently_after_a_request():
  # The MPC's own plan can be far from where the request left the brakes; handing straight back made the car lurch
  from openpilot.selfdrive.controls.lib.longitudinal_planner import SPEED_BUMP_HANDOVER_JERK
  plant = Plant(speed=V_CRUISE)
  for _ in range(40):
    plant.step(v_cruise=V_CRUISE, speed_bump_decel=2.0)
  prev = plant.acceleration
  for _ in range(20):
    plant.step(v_cruise=V_CRUISE)
    assert plant.acceleration - prev <= SPEED_BUMP_HANDOVER_JERK * 0.05 + 1e-3
    prev = plant.acceleration


def test_no_brake_pulse_after_a_gas_override():
  # The driver pressing the gas mid-request turns longitudinal control off. Easing the brakes off from where the request left them
  # once it was back on braked the car at -2.25 m/s^2 after half a second of gas
  plant = Plant(speed=V_CRUISE)
  for _ in range(40):
    plant.step(v_cruise=V_CRUISE, speed_bump_decel=3.0)
  plant.enabled = False
  for _ in range(10):
    plant.step(v_cruise=V_CRUISE)
  plant.enabled = True
  plant.step(v_cruise=V_CRUISE)
  assert plant.acceleration > -1.0
