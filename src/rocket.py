'''
Created on Aug 8, 2020

@author: camrdale
'''

import sys
import math
import time
import krpc
import numpy 
from scipy import optimize


def unit_vector(vector):
  """Returns the unit vector of the vector. """
  return vector / numpy.linalg.norm(vector)

def angle_between(v1, v2):
  """Returns the angle in radians between vectors 'v1' and 'v2'"""
  v1_u = unit_vector(v1)
  v2_u = unit_vector(v2)
  return numpy.arccos(numpy.clip(numpy.dot(v1_u, v2_u), -1.0, 1.0))


class Controller:
  
  def __init__(self, conn, vessel=None):
    '@type conn: Client'
    self.conn = conn
    if vessel:
      self.vessel = vessel
    else:
      self.vessel = conn.space_center.active_vessel
    self.second_stage = None
    self.second_stage_event = None

  def message(self, content):
    print(content)
    self.conn.ui.message(content, 3.0)

  ############################################################################################    
  # Methods to calculate various orbital maneuver parameters.
  ############################################################################################    
    
  def burn_time(self, delta_v):
    # Calculate burn time (using rocket equation)
    F = self.vessel.available_thrust
    Isp = self.vessel.specific_impulse * 9.82
    flow_rate = F / Isp
    
    stage = self.vessel.control.current_stage
    mf = 0.0
    while mf == 0.0:
      stage -= 1
      resources = self.vessel.resources_in_decouple_stage(stage=stage, cumulative=False)
      mf = resources.amount('LiquidFuel')*resources.density('LiquidFuel') + resources.amount('Oxidizer')*resources.density('Oxidizer')
    
    m0 = self.vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    
    # Check that we have enough fuel for the current engine to do that burn.
    if (m0 - m1) < mf:
      return (m0 - m1) / flow_rate
    
    # Burn all of current stage's fuel and reduce needed delta_v by that
    burn_time = mf / flow_rate
    delta_v -= Isp * math.log(m0 / (m0-mf))
    
    # Simulate next stage separation and rocket activation
    m0 -= sum(p.mass for p in self.vessel.parts.in_decouple_stage(stage))

    # Find the next stage that activates an engine
    while not [True for p in self.vessel.parts.in_stage(stage) if p.engine is not None]:
      stage -= 1
    engines = [p.engine for p in self.vessel.parts.in_stage(stage) if p.engine is not None]

    # Calculate rocket equation for remaining burn from next engine
    F = sum(e.available_thrust for e in engines)
    Isp = sum(e.vacuum_specific_impulse for e in engines) * 9.82
    flow_rate = F / Isp
    m1 = m0 / math.exp(delta_v/Isp)
    
    # Add two seconds for the engine transition
    return burn_time + (m0 - m1) / flow_rate + 2.0
  
  def circularization_delta_v(self, r):
    # Calculate circularization burn using vis-viva equation. Burn will be at radius r,
    # transitioning from current elliptical orbit to circular orbit of radius r.
    mu = self.vessel.orbit.body.gravitational_parameter
    a1 = self.vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    return v2 - v1
  
  def transfer_delta_v(self, starting_r, target_r):
    # Calculate transfer burn using vis-viva equation. Burn will be from current orbit at
    # starting radius r, transitioning to elliptical orbit with radii starting_r and target_r.
    mu = self.vessel.orbit.body.gravitational_parameter
    a1 = self.vessel.orbit.semi_major_axis
    a2 = (starting_r + target_r)/2.
    v1 = math.sqrt(mu*((2./starting_r)-(1./a1)))
    v2 = math.sqrt(mu*((2./starting_r)-(1./a2)))
    return v2 - v1
  
  def inclination_delta_v(self, target_orbit, ut):
    '@type orbit: krpc.types.Orbit'
    # https://en.wikipedia.org/wiki/Orbital_inclination_change#Calculation
    e = self.vessel.orbit.eccentricity
    # Argument of periapsis for vessel in relation to target orbit
    w = -self.vessel.orbit.true_anomaly_at_an(target_orbit)
    f = self.vessel.orbit.true_anomaly_at_ut(ut)
    n = 2 * math.pi / self.vessel.orbit.period
    a = self.vessel.orbit.semi_major_axis
    delta_i = self.vessel.orbit.relative_inclination(target_orbit)
    return 2 * math.sin(delta_i / 2.0) * math.sqrt(1 - math.pow(e, 2)) * math.cos(w + f) * n * a / (1 + e * math.cos(f))
  
  def escape_delta_v(self, ut):
    # https://en.wikipedia.org/wiki/Escape_velocity
    mu = self.vessel.orbit.body.gravitational_parameter
    orbital_speed_at_ut = numpy.linalg.norm(
      numpy.array(self.vessel.orbit.position_at(ut + 1, self.vessel.orbit.body.orbital_reference_frame)) -
      numpy.array(self.vessel.orbit.position_at(ut, self.vessel.orbit.body.orbital_reference_frame)))
    return math.sqrt(mu * 2.0 / self.vessel.orbit.radius_at(ut)) - orbital_speed_at_ut

  def g_force(self):
    # Calculate the g force at the current altitude.
    body = self.vessel.orbit.body
    # https://en.wikipedia.org/wiki/Gravity_of_Earth#Altitude
    return body.surface_gravity * math.pow(
      body.equatorial_radius / (body.equatorial_radius + self.vessel.flight(body.reference_frame).mean_altitude), 2)
    
  def hover_throttle(self):
    hover_delta_v = self.g_force()
    # hover_delta_v needs to be delivered in one second, every second
    return min(self.burn_time(abs(hover_delta_v)), 1.0)

  def transfer_time(self, starting_radius, target_radius):
    # Calculate the time to transfer from the starting radius to the target.
    # https://en.wikipedia.org/wiki/Hohmann_transfer_orbit#Calculation
    mu = self.vessel.orbit.body.gravitational_parameter
    return math.pi * math.sqrt(math.pow(starting_radius + target_radius, 3) / (8.0 * mu))
  
  def ut_at_true_anomaly(self, orbit, true_anomaly):
    # Orbit.ut_at_true_anomaly returns ut for first orbit, we want the next ut after now.
    first_ut = orbit.ut_at_true_anomaly(true_anomaly)
    now = self.conn.space_center.ut
    print('  True anomaly %0.1f First UT %0.1f' % (true_anomaly*180.0/math.pi, first_ut - now))
    period = orbit.period
    if first_ut >= now:
      return first_ut
    return first_ut + period * math.ceil((now - first_ut) / period)
  
  def heading(self, target_orbit):
    # Heading to take off at to reach the target inclination.
    body = self.vessel.orbit.body
    longitude_ref_direction = body.longitude_at_position(target_orbit.reference_plane_direction(body.reference_frame), body.reference_frame)
    longitude_of_ascending_node = target_orbit.longitude_of_ascending_node * 180 / math.pi 
    vessel_longitude = body.longitude_at_position(self.vessel.position(body.reference_frame), body.reference_frame)

    longitude_of_target_an = (longitude_ref_direction + longitude_of_ascending_node) % 360.0
    longitude_diff = longitude_of_target_an - vessel_longitude
    
    if abs(longitude_diff) > 90.0:
      # DN, so add inclination to the East heading
      return 90.0 + target_orbit.inclination * 180.0 / math.pi
    else:
      # AN, so subtract the inclination from East (and normalize)
      return (90.0 - target_orbit.inclination * 180.0 / math.pi) % 360.0
    
  def target_rendezvous_time(self, target_orbit, transfer_start_time):
    # Calculate the time when the target reaches the rendezvous for a transfer.
    vessel_angle = (
      self.vessel.orbit.longitude_of_ascending_node + 
      self.vessel.orbit.argument_of_periapsis + 
      self.vessel.orbit.true_anomaly_at_ut(transfer_start_time))
    # Rendezvous occurs at 180 degrees from the starting angle.
    rendezvous_angle = vessel_angle + math.pi
    target_rendezvous_time = self.ut_at_true_anomaly(target_orbit, 
      rendezvous_angle - 
      target_orbit.argument_of_periapsis -
      target_orbit.longitude_of_ascending_node)
    print('    Vessel Angle %0.1f Rendezvous Angle %0.1f Target True Anomaly %0.1f' % 
          (vessel_angle * 180.0/math.pi, rendezvous_angle * 180.0/math.pi,
           (rendezvous_angle - target_orbit.argument_of_periapsis - target_orbit.longitude_of_ascending_node) * 180.0/math.pi))
    return target_rendezvous_time

  ############################################################################################    
  # Methods that setup a future event to occur when conditions are right.
  ############################################################################################    
    
  def setup_srb(self, stage):  
    srb_stage_resources = self.vessel.resources_in_decouple_stage(stage=stage, cumulative=False)
    srb_fuel = self.conn.get_call(srb_stage_resources.amount, 'SolidFuel')
  
    srb_expr = self.conn.krpc.Expression.less_than(
      self.conn.krpc.Expression.call(srb_fuel),
      self.conn.krpc.Expression.constant_float(0.1))
    
    srb_event = self.conn.krpc.add_event(srb_expr)
    def separate_srbs():
      # Separate SRBs when finished
      self.vessel.control.activate_next_stage()
      self.message('SRBs separated')
      srb_event.remove()
    srb_event.add_callback(separate_srbs)
    srb_event.start()
    
  def setup_lrb(self, stage):  
    lrb_stage_resources = self.vessel.resources_in_decouple_stage(stage=stage, cumulative=False)
    lrb_fuel = self.conn.get_call(lrb_stage_resources.amount, 'LiquidFuel')
  
    lrb_expr = self.conn.krpc.Expression.less_than(
      self.conn.krpc.Expression.call(lrb_fuel),
      self.conn.krpc.Expression.constant_float(0.1))
    
    lrb_event = self.conn.krpc.add_event(lrb_expr)
    def separate_lrbs():
      # Separate LRBs when finished
      self.vessel.control.activate_next_stage()
      self.message('LRBs separated')
      lrb_event.remove()
    lrb_event.add_callback(separate_lrbs)
    lrb_event.start()
    
  def activate_second_stage(self):
    if self.second_stage is not None and self.vessel.control.current_stage > self.second_stage:
      # Decouple the first stage
      self.vessel.control.activate_next_stage()
      self.message('First Stage separated')
      time.sleep(1)
      # Activate the second stage
      self.vessel.control.activate_next_stage()
      self.message('Second Stage activated')
    if self.second_stage_event is not None:
      self.second_stage_event.remove()
      self.second_stage_event = None
  
  def setup_second_stage(self, stage):  
    first_stage_resources = self.vessel.resources_in_decouple_stage(stage=stage, cumulative=False)
    first_stage_fuel = self.conn.get_call(first_stage_resources.amount, 'LiquidFuel')
    self.second_stage = stage
  
    second_stage_expr = self.conn.krpc.Expression.less_than(
        self.conn.krpc.Expression.call(first_stage_fuel),
        self.conn.krpc.Expression.constant_float(0.1))
    
    self.second_stage_event = self.conn.krpc.add_event(second_stage_expr)
    self.second_stage_event.add_callback(self.activate_second_stage)
    self.second_stage_event.start()
  
  def setup_gravity_turn(self, turn_end_altitude, turn_start=250.0, target_heading=90.0):  
  
    # Set up streams for telemetry
    altitude_stream = self.conn.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
  
    # Main ascent loop
    self.turn_angle = 0
    starting_altitude = altitude_stream()
    turn_start_altitude = turn_start + starting_altitude
    
    def apply_turn(altitude):
      # Gravity turn
      if altitude > turn_start_altitude and altitude < starting_altitude + turn_end_altitude:
        frac = math.sqrt((altitude - turn_start_altitude) /
                (starting_altitude + turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 89
        if abs(new_turn_angle - self.turn_angle) > 0.5:
          self.turn_angle = new_turn_angle
          self.vessel.auto_pilot.target_pitch_and_heading(90 - self.turn_angle, target_heading)
          
      if altitude >= starting_altitude + turn_end_altitude:
        altitude_stream.remove_callback(apply_turn)
          
    altitude_stream.add_callback(apply_turn)
    altitude_stream.start()

  ############################################################################################    
  # Methods that are used by other methods.
  ############################################################################################    

  def auto_pilot_wait(self, target_vessel=None):
    if target_vessel is None:
      target_vessel = self.vessel
    time.sleep(0.1)
    target_vessel.auto_pilot.wait()
    while abs(target_vessel.auto_pilot.error) > 10.0:
      print('Auto pilot wait terminated prematurely with error still: %0.3f' % (target_vessel.auto_pilot.error,))
      time.sleep(1)
      target_vessel.auto_pilot.wait()

  def perform_burn(self, burn_ut, distance_to_target=None, **kwargs):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')

    node = self.vessel.control.add_node(burn_ut, **kwargs)
    burn_time = self.burn_time(abs(node.delta_v))
    full_throttle = 1.0
    while burn_time < 2.0 and full_throttle > 0.001:
      burn_time *= 2.0
      full_throttle /= 2.0
    
    # Wait until burn
    start_burn_ut = burn_ut - (burn_time/2.)
    lead_time = 60
    if start_burn_ut - ut() > lead_time:
      self.message('Warping to burn')
      self.conn.space_center.warp_to(start_burn_ut - lead_time)
  
    # Orient ship
    self.message('Orienting ship for burn')
    self.vessel.auto_pilot.reference_frame = node.reference_frame
    self.vessel.auto_pilot.roll_threshold = 15.0
    self.vessel.auto_pilot.engage()
    # Point towards the node's target direction.
    self.vessel.auto_pilot.target_direction = (0, 1, 0)
    self.auto_pilot_wait()
  
    # Execute burn
    self.message('Ready to execute burn for %0.3f seconds at %0.5f throttle' % 
                 (burn_time, full_throttle))
    while ut() < start_burn_ut:
      pass
  
    self.message('Executing burn')
    self.vessel.control.throttle = full_throttle
    time.sleep(burn_time - 0.1)
   
    self.message('Fine tuning')
    self.vessel.control.throttle = full_throttle / 10.0

    remaining_burn = self.conn.add_stream(node.remaining_burn_vector, node.reference_frame)
    remaining_burn_time = remaining_burn()[1]
    last_remaining_burn_time = remaining_burn_time + 1.0
    
    remaining_distance_to_target = distance_to_target() if distance_to_target else None
    last_remaining_distance_to_target = remaining_distance_to_target + 1.0 if distance_to_target else None
    
    while remaining_burn_time > 0.1 and (distance_to_target is None or remaining_distance_to_target > 0.0):
      if last_remaining_burn_time < remaining_burn_time or (distance_to_target is not None and last_remaining_distance_to_target < remaining_distance_to_target):
        self.message('Aborting fine tuning')
        break

      if distance_to_target is not None and last_remaining_distance_to_target != remaining_distance_to_target:
        iterations_remaining = remaining_distance_to_target / (last_remaining_distance_to_target - remaining_distance_to_target)
        if iterations_remaining < 10.0 and self.vessel.control.throttle > 0.0001:
          self.vessel.control.throttle = self.vessel.control.throttle / 2.0

      time.sleep(0.1)
      last_remaining_burn_time = remaining_burn_time
      remaining_burn_time = remaining_burn()[1]
      last_remaining_distance_to_target = remaining_distance_to_target if distance_to_target else None
      remaining_distance_to_target = distance_to_target() if distance_to_target else None

    self.vessel.control.throttle = 0.0
    node.remove()
    self.vessel.auto_pilot.disengage()
    
  def try_rendezvous_at(self, target, rendezvous_time): 
    node = self.vessel.control.add_node(rendezvous_time)
    
    delta = 1.0
    num_iterations = 0
    while abs(delta) > 0.1 or num_iterations >= 100:
      num_iterations += 1
      closest_approach = min(node.orbit.list_closest_approaches(target.orbit, 2)[1])
      node.prograde += 0.01 * delta / abs(delta)
      next_closest_approach = min(node.orbit.list_closest_approaches(target.orbit, 2)[1])
      node.prograde -= 0.01 * delta / abs(delta)
      if next_closest_approach > closest_approach:
        delta = -delta / 2.0
      node.prograde += delta
      print('  Closest %0.1f Next %0.1f Prograde %0.3f Delta %0.3f' % (closest_approach, next_closest_approach, node.prograde, delta))

    delta_v = node.prograde
    node.remove()    
    return closest_approach, delta_v

  ############################################################################################    
  # Methods that launch a vessel into space.
  ############################################################################################    
  
  def wait_for_launch(self, target_angle=None):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    target = self.conn.space_center.target_vessel
    
    self.message('Calculating launch window for ' + target.name)

    frame = target.orbit.body.reference_frame

    if target_angle is None:
      target_angle = -20.0 * math.pi / 180
    starting_ut = ut()
    
    def transfer_angle(transfer_time):
      if transfer_time < starting_ut:
        transfer_time = starting_ut
      current_angle = angle_between(
        self.vessel.position(frame),
        target.orbit.position_at(transfer_time, frame))
      next_angle = angle_between(
        self.vessel.position(frame),
        target.orbit.position_at(transfer_time + 1, frame))
      if next_angle < current_angle:
        current_angle = -current_angle
      current_angle -= 2.0 * math.pi * (transfer_time - starting_ut) / self.vessel.orbit.body.rotational_period
      if current_angle < -math.pi:
        current_angle += 2 * math.pi
      print('  Target %0.3f Time %0.1f Current %0.3f' % (target_angle, transfer_time - ut(), current_angle))
      return abs(target_angle - current_angle)

    # Look for a time when the transfer angle is met
    transfer_time = optimize.newton(
      transfer_angle, starting_ut + target.orbit.period / 2.0, tol=0.0001, maxiter=100,
      x1=(starting_ut + target.orbit.period))
    
    if transfer_time - starting_ut > target.orbit.period * 1.1:
      print('  Launch window retry, %0.1f greater than period of %0.1f' % 
            (transfer_time - starting_ut, target.orbit.period))
      transfer_time = optimize.newton(
        transfer_angle, starting_ut, tol=0.0001, maxiter=100,
        x1=(transfer_time - target.orbit.period))
      
    self.message('Found launch window in %d seconds' % ((transfer_time - ut()),))
    
    # Wait until launch time
    self.message('Warping to launch time')
    lead_time = 5
    self.conn.space_center.warp_to(transfer_time - lead_time)
  
  def launch(self, throttle=1.0, activate_stage=True):  
    # Pre-launch setup
    self.vessel.control.sas = False
    self.vessel.control.rcs = False
    
    # Countdown...
    self.message('3...')
    time.sleep(1)
    self.message('2...')
    time.sleep(1)
    self.message('1...')
    time.sleep(1)
    self.message('Launch!')
  
    self.vessel.control.throttle = throttle
    if activate_stage:
      # Activate the first stage
      self.vessel.control.activate_next_stage()
    self.vessel.auto_pilot.engage()
    self.vessel.auto_pilot.target_pitch_and_heading(90, 90)
    self.vessel.auto_pilot.target_roll = 90
  
  def to_altitude(self, target_altitude):  
    self.message('Launching to target altitude of %s' % (target_altitude,))

    # Set up streams for telemetry
    apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
    
    while apoapsis() < target_altitude:
      # Decrease throttle when approaching target apoapsis
      if apoapsis() > target_altitude*0.99:
        self.message('Approaching target apoapsis')
        break
  
    # Disable engines when target apoapsis is reached
    self.vessel.control.throttle = 0.25
    while apoapsis() < target_altitude:
      pass
    self.message('Target apoapsis reached')
    self.vessel.control.throttle = 0.0
    # self.vessel.auto_pilot.disengage()
    
    # Retract the landing legs    
    self.vessel.control.gear = False
  
  def to_space(self):  
    # Set up streams for telemetry
    altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
    
    # Wait until out of atmosphere
    self.message('Coasting out of atmosphere')
    self.conn.space_center.physics_warp_factor = 3
    while altitude() < 70100:
      pass

    self.conn.space_center.physics_warp_factor = 0
    
    # Deploy any antennas
    self.vessel.control.antennas = True
    
  def drop_engine_at_peak(self):
    vertical_speed = self.conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'vertical_speed')
  
    # Wait until out of atmosphere
    self.message('Coasting to peak')
    while vertical_speed() > 0:
      pass
  
    # Drop the engine
    self.vessel.control.activate_next_stage()

  ############################################################################################    
  # Methods that adjust a vessel's orbit in space.
  ############################################################################################    
    
  def circularization_burn(self, periapsis=False):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    if not periapsis:
      target_radius = self.vessel.orbit.apoapsis
      time_to_target = self.conn.add_stream(getattr, self.vessel.orbit, 'time_to_apoapsis')
    else:
      target_radius = self.vessel.orbit.periapsis
      time_to_target = self.conn.add_stream(getattr, self.vessel.orbit, 'time_to_periapsis')

    # Plan circularization burn
    self.message('Planning circularization burn')
    delta_v = self.circularization_delta_v(target_radius)
    
    periapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'periapsis')
    apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis')
    def distance_to_target():
      return apoapsis() - periapsis()

    self.perform_burn(ut() + time_to_target(), distance_to_target, prograde=delta_v)
    
  def inclination_burn(self, target_orbit=None):
    if not target_orbit:
      target_orbit = self.conn.space_center.target_vessel.orbit
    
    if self.vessel.orbit.relative_inclination(target_orbit) < 0.1 * math.pi / 180:
      return
    
    # Plan inclination adjustment burn
    self.message('Calculating inclination adjustment burn')
    
    inclination_time = min(
      self.ut_at_true_anomaly(self.vessel.orbit, self.vessel.orbit.true_anomaly_at_an(target_orbit)),
      self.ut_at_true_anomaly(self.vessel.orbit, self.vessel.orbit.true_anomaly_at_dn(target_orbit)))
    
    delta_v = -self.inclination_delta_v(target_orbit, inclination_time)
    relative_inclination = self.conn.add_stream(self.vessel.orbit.relative_inclination, target_orbit)
    def distance_to_target():
      return abs(relative_inclination())

    self.perform_burn(inclination_time, distance_to_target=distance_to_target, normal=delta_v)
    
  def transfer_burn(self, target, target_distance):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    self.message('Calculating Hohmann transfer launch window for ' + target.name)

    starting_ut = ut()
    
    # Iteratively calculate the transfer start time as the target's radius at the
    # expected rendezvous depends on the time chosen and the time to rendezvous.
    def time_discrepancy(transfer_start_time):
      # Calculate the time discrepancy from when the vessel and target reach the rendezvous.
      starting_radius = self.vessel.orbit.radius_at(transfer_start_time)
      target_rendezvous_time = self.target_rendezvous_time(target.orbit, transfer_start_time)
      target_radius = target.orbit.radius_at(target_rendezvous_time)
      vessel_rendezvous_time = transfer_start_time + self.transfer_time(starting_radius, target_radius)
      print('  Start Time %0.1f Vessel Rendezvous %0.1f Target Rendezvous %0.1f Target Radius %0.1f' %
            (transfer_start_time - starting_ut, vessel_rendezvous_time - starting_ut,
             target_rendezvous_time - starting_ut, target_radius))
      return target_rendezvous_time - vessel_rendezvous_time

    # Look for a time when the vessel and body rendezvous.
    transfer_time = optimize.newton(
      time_discrepancy, ut() + self.vessel.orbit.period / 2.0,
      tol=0.1, maxiter=100,
      x1=(ut() + self.vessel.orbit.period))
    rendezvous_time = self.target_rendezvous_time(target.orbit, transfer_time)
    
    self.message('Found launch window in %d seconds' % ((transfer_time - ut()),))
    
    # Plan transfer burn
    self.message('Planning transfer burn')
    delta_v = self.transfer_delta_v(self.vessel.orbit.radius_at(transfer_time), target.orbit.radius_at(rendezvous_time))

    if type(target) == self.conn.space_center.CelestialBody:
      def distance_to_target():
        # Wait until there is an intercept with the target
        if self.vessel.orbit.next_orbit is None:
          return target.sphere_of_influence
        return self.vessel.orbit.next_orbit.periapsis - target.equatorial_radius - target_distance
    else:
      distance_at_closest_approach = self.conn.add_stream(self.vessel.orbit.distance_at_closest_approach, target.orbit)
      def distance_to_target():
        return distance_at_closest_approach() - target_distance

    self.perform_burn(transfer_time, distance_to_target=distance_to_target, prograde=delta_v)

  def warp_to_encounter(self):
    # Align ship to Sun so that panels are charging
    self.message('Aligning ship to expose solar panels to sun')
    sun = self.conn.space_center.bodies['Sun']
    current_period = 2.0 * math.pi * math.sqrt(math.pow(self.vessel.orbit.radius, 3) / self.vessel.orbit.body.gravitational_parameter)

    self.vessel.auto_pilot.reference_frame = sun.reference_frame
    self.vessel.auto_pilot.roll_threshold = 15.0
    self.vessel.auto_pilot.engage()
    self.vessel.auto_pilot.target_direction = (0, 1, 0)
    self.vessel.auto_pilot.target_roll = 0
    self.auto_pilot_wait()
    
    while sum(s.sun_exposure for s in self.vessel.parts.solar_panels) < 0.25:
      if self.vessel.auto_pilot.target_roll > 179.0:
        self.message('Failed to find sun exposure for solar panels, waiting a bit')
        self.conn.space_center.warp_to(self.conn.space_center.ut + current_period / 3.0)
        self.vessel.auto_pilot.target_roll = 0
      else:
        self.vessel.auto_pilot.target_roll += 90
      self.auto_pilot_wait()
    
    self.message('Warping to next orbital encounter')
    self.conn.space_center.warp_to(self.conn.space_center.ut + self.vessel.orbit.time_to_soi_change + 60.0)
    
  def rendezvous_burn(self):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    target = self.conn.space_center.target_vessel
    
    if self.vessel.orbit.distance_at_closest_approach(target.orbit) < 500:
      return
    
    # Plan rendeszvous burn
    self.message('Calculating rendezvous burn for ' + target.name)
    
    print('Trying time_of_closest_approach')
    rendezvous_time = self.vessel.orbit.time_of_closest_approach(target.orbit)
    closest_approach, rendezvous_deltav = self.try_rendezvous_at(target, rendezvous_time)
    print('For time_of_closest_approach: %0.1f' % closest_approach)
    
    print('Trying periapsis')
    rendezvous_time_try = ut() + self.vessel.orbit.time_to_periapsis
    closest_approach_try, rendezvous_deltav_try = self.try_rendezvous_at(target, rendezvous_time_try)
    print('For periapsis: %0.1f' % closest_approach_try)
    if closest_approach_try < closest_approach:
      closest_approach = closest_approach_try
      rendezvous_time = rendezvous_time_try
      rendezvous_deltav = rendezvous_deltav_try
    
    print('Trying apoapsis')
    rendezvous_time_try = ut() + self.vessel.orbit.time_to_apoapsis
    closest_approach_try, rendezvous_deltav_try = self.try_rendezvous_at(target, rendezvous_time_try)
    print('For apoapsis: %0.1f' % closest_approach_try)
    if closest_approach_try < closest_approach:
      closest_approach = closest_approach_try
      rendezvous_time = rendezvous_time_try
      rendezvous_deltav = rendezvous_deltav_try
    
    def distance_to_target():
      return min(self.vessel.orbit.list_closest_approaches(target.orbit, 2)[1])
    
    self.perform_burn(rendezvous_time, distance_to_target=distance_to_target, prograde=rendezvous_deltav)

  def rendezvous(self):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')

    target = self.conn.space_center.target_vessel

    closest_approach = min(zip(*self.vessel.orbit.list_closest_approaches(target.orbit, 2)), key=lambda x: x[1])
    
    if closest_approach[0] - ut() > 60.0:
      self.message('Warping to approach with ' + target.name)
      self.conn.space_center.warp_to(closest_approach[0] - 10.0)
      closest_approach = min(zip(*self.vessel.orbit.list_closest_approaches(target.orbit, 2)), key=lambda x: x[1])

    # Plan approach burn
    self.message('Planning approach burn to cancel relative velocity')
    velocity = self.vessel.velocity(target.orbital_reference_frame)
    delta_v = -numpy.linalg.norm(numpy.array(velocity))
    burn_time = self.burn_time(abs(delta_v))
    full_throttle = 1.0
    while burn_time < 2.0 and full_throttle > 0.001:
      burn_time *= 10.0
      full_throttle /= 10.0

    # Orient ship
    self.message('Orienting ship for approach burn')
    self.vessel.auto_pilot.reference_frame = target.orbital_reference_frame
    self.vessel.auto_pilot.engage()
    # Point in the opposite direction of the velocity
    self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity))
    self.auto_pilot_wait()
    
    # Execute burn
    self.message('Ready to execute burn')
    while closest_approach[0] - ut() - (burn_time/2.) > 0:
      pass
  
    self.message('Executing burn')
    self.vessel.control.throttle = full_throttle
    time.sleep(burn_time)
    self.vessel.control.throttle = 0.0
   
    position = self.conn.add_stream(self.vessel.position, target.orbital_reference_frame)
    distance_to_target = lambda : numpy.linalg.norm(numpy.array(position()))
    velocity = self.conn.add_stream(self.vessel.velocity, target.orbital_reference_frame)
    
    self.message('Starting fine approach')
    while True:
      self.message('Canceling relative velocity')
      delta_v = -numpy.linalg.norm(numpy.array(velocity()))
      burn_time = self.burn_time(abs(delta_v))
      full_throttle = 1.0
      while burn_time < 2.0 and full_throttle > 0.001:
        burn_time *= 10.0
        full_throttle /= 10.0
    
      # Point in the opposite direction of the velocity
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
      self.auto_pilot_wait()
  
      self.vessel.control.throttle = full_throttle
      time.sleep(burn_time)
      self.vessel.control.throttle = 0.0

      # No velocity relative to target, see if we are close enough.    
      if distance_to_target() < 20.0:
        break
      
      self.message('Moving towards the target')

      # Point towards the target
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(position()))
      self.auto_pilot_wait()
  
      # Approach at a rate that takes 60 seconds to close.
      delta_v = distance_to_target() / 60.0
      burn_time = self.burn_time(abs(delta_v))
      full_throttle = 1.0
      while burn_time < 1.0 and full_throttle > 0.001:
        burn_time *= 5.0
        full_throttle /= 5.0

      self.vessel.control.throttle = full_throttle
      time.sleep(burn_time)
      self.vessel.control.throttle = 0.0
      
      # Prepare for cancellation by pointing in the opposite direction of the velocity
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
      self.auto_pilot_wait()
  
      # Cancel the approach at 15m with at least 2 seconds to spare (plus any burn time)
      approach_buffer = 15.0 + abs(delta_v) * (2.0 + self.burn_time(abs(delta_v)))

      distance = distance_to_target()
      last_distance = distance + 1.0
      while distance > approach_buffer:
        if last_distance < distance:
          self.message('Aborting fine approach')
          break
        last_distance = distance
        distance = distance_to_target()

    self.message('Fine approach complete')
    self.vessel.auto_pilot.disengage()

  ############################################################################################    
  # Methods that interact with a non-Kerbin body.
  ############################################################################################    
  
  def cancel_surface_velocity(self, target_longitude=None, use_node=False):
    # Set up streams for telemetry
    body = self.vessel.orbit.body
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    if use_node:
      if not self.vessel.control.nodes:
        self.message('ERROR: no node found to stop at')
        sys.exit('ERROR: no node found to stop at')
        return
      landing_time = self.vessel.control.nodes[0].ut

    else:    
      if target_longitude is None:
        # Land on the sunny side of the currently orbited body.
        self.message('Finding sunny location to land')
        sun = self.conn.space_center.bodies['Sun']
        starting_ut = ut()
        target_longitude = body.longitude_at_position(sun.position(body.reference_frame), body.reference_frame)
       
      def longitude_discrepancy(landing_time):
        longitude = body.longitude_at_position(self.vessel.orbit.position_at(landing_time, body.reference_frame), body.reference_frame)
        longitude_diff = longitude - target_longitude
        longitude_diff = (longitude_diff + 180.0) % 360.0 - 180.0
        print('  Land Time %0.1f Vessel Long %0.1f Target Long %0.1f Diff %0.1f' %
              (landing_time - starting_ut, longitude, target_longitude, longitude_diff))
        return longitude_diff
   
      # Look for a time when the vessel is over the sunny longitude.
      landing_time = optimize.newton(
        longitude_discrepancy, ut() + self.vessel.orbit.period / 2.0,
        tol=0.1, maxiter=100,
        x1=(ut() + self.vessel.orbit.period))
     
    self.message('Found landing time in %d seconds' % ((landing_time - ut()),))
 
    if landing_time - ut() > 120.0:
      self.message('Warping to landing time on ' + body.name)
      self.conn.space_center.warp_to(landing_time - 60.0)
 
    # Plan landing burn
    self.message('Planning landing burn to cancel surface velocity')
    velocity = self.conn.add_stream(self.vessel.velocity, body.reference_frame)
    delta_v = -numpy.linalg.norm(numpy.array(velocity()))
    burn_time = self.burn_time(abs(delta_v))
    full_throttle = 1.0
    while burn_time < 2.0 and full_throttle > 0.001:
      burn_time *= 10.0
      full_throttle /= 10.0
 
    # Orient ship
    self.message('Orienting ship for landing burn')
    self.vessel.auto_pilot.reference_frame = body.reference_frame
    self.vessel.auto_pilot.engage()
    # Point in the opposite direction of the velocity
    self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
    self.auto_pilot_wait()
     
    # Execute burn
    self.message('Ready to execute burn')
    while landing_time - ut() - (burn_time/2.) > 10.0:
      pass
   
    self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
    time.sleep(10)
    self.message('Executing burn')
    start_burn_ut = ut()
    self.vessel.control.throttle = full_throttle
    while ut() < start_burn_ut + burn_time:
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
    self.vessel.control.throttle = self.vessel.control.throttle / 10.0
     
    self.message('Starting fine velocity cancellation')
    horizontal_speed = self.conn.add_stream(getattr, self.vessel.flight(body.reference_frame), 'horizontal_speed')
    while abs(horizontal_speed()) > 1.0:
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
    self.vessel.control.throttle = 0.0
  
    self.message('Fine velocity cancellation complete')

  def prepare_for_landing(self):  
    # If there's still a first stage, dump it here.    
    self.activate_second_stage()
    time.sleep(1)
  
    # Deploy the landing legs    
    self.vessel.control.gear = True
    time.sleep(2)

  def vertical_cancellation_time(self, vertical_speed):
    # Calculate the time it takes to cancel the vertical speed.
    burn_time = 0.0
    g = self.g_force()
    vertical_speed_increment = abs(vertical_speed)
    while vertical_speed_increment > 0.1:
      burn_time_increment = self.burn_time(vertical_speed_increment)
      burn_time += burn_time_increment
      # Calculate the additional velocity picked up during the burn.
      vertical_speed_increment = g * burn_time_increment
    return burn_time
    
  def suicide_burn(self):     
    body = self.vessel.orbit.body
    ref_frame = self.conn.space_center.ReferenceFrame.create_hybrid(
        position=body.reference_frame,
        rotation=self.vessel.surface_reference_frame)
    position = self.conn.add_stream(self.vessel.position, ref_frame)
    velocity = self.conn.add_stream(self.vessel.velocity, ref_frame)
    vertical_speed = self.conn.add_stream(getattr, self.vessel.flight(body.reference_frame), 'vertical_speed')
    surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(body.reference_frame), 'surface_altitude')

    self.message('Waiting for suicide burn altitude')
    self.vessel.auto_pilot.reference_frame = ref_frame
    self.vessel.auto_pilot.engage()
    self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
    # 
    while abs(2.0*surface_altitude()/vertical_speed()) - self.vertical_cancellation_time(vertical_speed()) > 0.5:
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))

    self.message('Starting suicide burn')
    self.vessel.control.throttle = 1.0
    while vertical_speed() < -1.0:
      if vertical_speed() > -5.0:
        self.vessel.control.throttle = 0.25
      self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))

    self.message('Hovering to the surface')
    self.vessel.control.throttle = self.hover_throttle() * 0.95
    while self.vessel.situation != self.conn.space_center.VesselSituation.landed:
      self.vessel.auto_pilot.target_direction = tuple(x for x in unit_vector(position()))
    self.vessel.control.throttle = 0.0

    self.vessel.auto_pilot.disengage()
    self.message('Welcome to ' + body.name + '!')

  ############################################################################################    
  # Methods for interacting with nearby ships.
  ############################################################################################    

  def cancel_relative_velocity(self):
    target_port = self.conn.space_center.target_docking_port
    target = target_port.part.vessel
    self.vessel.control.rcs = False

    velocity = self.conn.add_stream(self.vessel.velocity, target.orbital_reference_frame)
    
    if numpy.linalg.norm(numpy.array(velocity())) < 0.2:
      return
    
    self.message('Canceling relative velocity')
    delta_v = -numpy.linalg.norm(numpy.array(velocity()))
    burn_time = self.burn_time(abs(delta_v))
    full_throttle = 1.0
    while burn_time < 2.0 and full_throttle > 0.001:
      burn_time *= 10.0
      full_throttle /= 10.0
  
    # Point in the opposite direction of the velocity
    self.vessel.auto_pilot.target_direction = tuple(-x for x in unit_vector(velocity()))
    self.vessel.auto_pilot.engage()
    self.auto_pilot_wait()

    self.vessel.control.throttle = full_throttle
    time.sleep(burn_time)
    self.vessel.control.throttle = 0.0
    
  def dock(self):
    current_port = self.conn.space_center.active_vessel.parts.controlling.docking_port
    target_port = self.conn.space_center.target_docking_port
    target = target_port.part.vessel
    self.vessel.control.rcs = False
    self.vessel.control.sas = False
    target.control.rcs = False
    target.control.sas = False

    vessel_position = self.conn.add_stream(self.vessel.position, target.orbital_reference_frame)
    target_position = self.conn.add_stream(target.position, self.vessel.orbital_reference_frame)
    distance = self.conn.add_stream(target_port.position, current_port.reference_frame)
    velocity = self.conn.add_stream(current_port.part.velocity, target_port.reference_frame)

    self.message('Orienting target ship for docking maneuver')
    self.conn.space_center.active_vessel = target
    target.control.throttle = 0.0
    target.auto_pilot.reference_frame = target.orbital_reference_frame
    target.auto_pilot.target_direction = unit_vector(vessel_position())
    target.auto_pilot.engage()
    self.auto_pilot_wait(target_vessel=target)
    
    self.message('Orienting vessel for docking maneuver')
    self.conn.space_center.active_vessel = self.vessel
    self.vessel.auto_pilot.reference_frame = self.vessel.orbital_reference_frame
    self.vessel.auto_pilot.target_direction = unit_vector(target_position())
    self.vessel.auto_pilot.engage()
    self.auto_pilot_wait()
    
    self.message('Beginning docking approach')
    self.vessel.control.rcs = True
    while current_port.state == self.conn.space_center.DockingPortState.ready:
      target.auto_pilot.target_direction = unit_vector(vessel_position())
      self.vessel.auto_pilot.target_direction = unit_vector(target_position())
      
      current_distance = numpy.linalg.norm(numpy.array(distance()))

      if -(velocity()[1]) < current_distance / 10.0:
        self.vessel.control.forward = 0.5
      elif -(velocity()[1]) > 1.5 * current_distance / 10.0:
        self.vessel.control.forward = -0.5
      else:
        self.vessel.control.forward = 0.0
      
    self.message('Docking magnets engaged')

    target.auto_pilot.disengage()
    self.vessel.auto_pilot.disengage()
    self.vessel.control.rcs = False

    while current_port.state == self.conn.space_center.DockingPortState.docking:
      pass
    
    if current_port.state == self.conn.space_center.DockingPortState.docked:
      self.message('Docking complete')
    else:
      self.message('Docking failed!!!')

  ############################################################################################    
  # Methods that return a vessel to Kerbin.
  ############################################################################################    
    
  def return_burn(self):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    self.message('Calculating launch window for return home')

    starting_ut = ut()
    frame = self.vessel.orbit.body.orbital_reference_frame
    # Return launch when 90 deg from orbital direction of body being orbitted
    target_angle = math.pi/2.0
#     if self.vessel.orbit.inclination > 90.0:
#       target_angle = -target_angle

    def angle_discrepancy(return_time):
      position = unit_vector(self.vessel.orbit.position_at(return_time, frame))
      angle = math.pi/2.0 - math.atan2(position[1], position[0]) 
      angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
      print('  Time %0.1f, position "%s", angle %0.1f target %0.1f' % 
            (return_time - starting_ut, position, angle * 180.0 / math.pi, target_angle * 180.0 / math.pi))
      return (target_angle - angle + math.pi) % (2.0 * math.pi) - math.pi

    return_time = optimize.newton(
      angle_discrepancy, ut() + self.vessel.orbit.period / 2.0,
      tol=0.1, maxiter=100,
      x1=(ut() + self.vessel.orbit.period))
      
    self.message('Found launch window in %d seconds' % ((return_time - ut()),))

    # Plan rendeszvous burn
    self.message('Calculating return burn')
    
    target_periapsis = 35000.0
    
    # Start with a node to escape the current orbit
    escape_deltav = self.escape_delta_v(return_time)
    node = self.vessel.control.add_node(return_time, prograde=escape_deltav)

    if node.orbit.next_orbit is None:
      self.message('ERROR: failed to escape from current orbit for return')
      sys.exit('ERROR: failed to escape from current orbit for return')
      return
  
    target_periapsis_radius = target_periapsis + node.orbit.next_orbit.body.equatorial_radius
      
    def return_periapsis(delta_v):
      if delta_v < escape_deltav:
        delta_v = escape_deltav
      node.prograde = delta_v
      current_periapsis = node.orbit.next_orbit.periapsis
      print('  Target %0.1f Periapsis %0.1f Prograde %0.3f' % (target_periapsis_radius, current_periapsis, delta_v))
      return current_periapsis - target_periapsis_radius
      
    delta_v = optimize.newton(
      return_periapsis, escape_deltav + 100.0, tol=0.1, maxiter=100, x1=(escape_deltav + 200.0))
    node.remove()
  
    def distance_to_target():
      return self.vessel.orbit.next_orbit.periapsis - self.vessel.orbit.next_orbit.body.equatorial_radius - target_periapsis
      
    self.perform_burn(return_time, distance_to_target=distance_to_target, prograde=delta_v)
    
  def reentry_burn(self):
    target_altitude = 35000.0
    
    # Set up streams for telemetry
    periapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'periapsis_altitude')
  
    # Orient ship
    self.message('Orienting ship for reentry burn')
    self.vessel.auto_pilot.reference_frame = self.vessel.orbital_reference_frame

    # Point towards the retrograde
    self.vessel.auto_pilot.engage()
    self.vessel.auto_pilot.target_direction = (0, -1, 0)
    self.auto_pilot_wait()
    
    remaining_distance_to_target = periapsis() - target_altitude
    last_remaining_distance_to_target = remaining_distance_to_target + 1.0

    self.message('Executing burn')
    self.vessel.control.throttle = 1.0
    while periapsis() > target_altitude:
      if last_remaining_distance_to_target != remaining_distance_to_target:
        iterations_remaining = remaining_distance_to_target / (last_remaining_distance_to_target - remaining_distance_to_target)
        if iterations_remaining < 10.0 and self.vessel.control.throttle > 0.001:
          self.vessel.control.throttle = self.vessel.control.throttle / 2.0

      time.sleep(0.1)
      last_remaining_distance_to_target = remaining_distance_to_target
      remaining_distance_to_target = periapsis() - target_altitude

    self.message('Target periapsis reached')
    self.vessel.control.throttle = 0.0
    self.vessel.auto_pilot.disengage()
    
  def prepare_for_reentry(self):
    # Set up streams for telemetry
    ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    target_altitude = 70000.0
    starting_ut = ut()
    
    # Wait until into the atmosphere
    self.message('Warping into the atmosphere')
    
    def reentry_altitude(atmosphere_time):
      altitude = self.vessel.orbit.radius_at(atmosphere_time) - self.vessel.orbit.body.equatorial_radius
      print('  Target %0.1f Time %0.1f Altitude %0.1f' % (target_altitude, atmosphere_time - starting_ut, altitude))
      return altitude - target_altitude
    
    atmosphere_time = optimize.newton(
      reentry_altitude, max(self.vessel.orbit.time_to_periapsis + starting_ut - 600.0, starting_ut),
      tol=0.1, maxiter=100, x1=(self.vessel.orbit.time_to_periapsis + starting_ut))
      
    self.conn.space_center.warp_to(atmosphere_time)
    
    # Orient ship
    self.message('Orienting ship for reentry')
    self.vessel.auto_pilot.reference_frame = self.vessel.orbital_reference_frame

    # Point towards the retrograde
    self.vessel.auto_pilot.engage()
    self.vessel.auto_pilot.target_direction = (0, -1, 0)
    self.auto_pilot_wait()

    altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
    while altitude() > target_altitude:
      pass

    # Drop the engine
    self.message('Dropping engines')
    self.vessel.control.activate_next_stage()
    
    # Retract any antennas
    self.vessel.control.antennas = False

    self.conn.space_center.physics_warp_factor = 3

  def reentry_parachute(self):
    surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')

    # Wait until at safe height for parachute
    self.message('Falling back to Kerbin')
    while surface_altitude() > 1500:
      pass
    
    # Open the parachute
    self.message('Opening parachute')
    self.vessel.control.activate_next_stage()
    self.vessel.auto_pilot.disengage()

    while self.vessel.situation == self.conn.space_center.VesselSituation.flying:
      pass

    self.conn.space_center.physics_warp_factor = 0
