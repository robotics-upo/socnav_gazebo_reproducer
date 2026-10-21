import json
import math
import sys
import os
import statistics
from pathlib import Path
from xml.sax.saxutils import escape

HUMAN_SUPPORT_WINDOW = 0.25
HUMAN_CLUSTER_DISTANCE = 1.2
HUMAN_NEIGHBOR_STEPS = 5
GOAL_ANGLE_MESH_SEGMENTS = 96
CAMERA_OUTLIER_MIN_DISTANCE = 2.0
CAMERA_OUTLIER_GEOMETRY_FRACTION = 0.25
CAMERA_OUTLIER_MIN_SPEED = 3.0
CAMERA_ORIENTATION_STEPS = 180
CAMERA_HUMAN_FOOTPRINT = 0.7
CAMERA_WALL_HEIGHT = 2.5
CAMERA_DISTANCE_MARGIN = 1.1

def _compute_human_slot_count(sequence):
  supported = [
    [False] * len(step.get('people', []))
    for step in sequence
  ]

  for step_idx, step in enumerate(sequence):
    timestamp = step.get('timestamp', 0.0)
    for person_idx, person in enumerate(step.get('people', [])):
      for neighbor_idx in range(
          max(0, step_idx - HUMAN_NEIGHBOR_STEPS),
          min(len(sequence), step_idx + HUMAN_NEIGHBOR_STEPS + 1)):
        if neighbor_idx == step_idx:
          continue
        neighbor_step = sequence[neighbor_idx]
        if abs(neighbor_step.get('timestamp', 0.0) - timestamp) > HUMAN_SUPPORT_WINDOW:
          continue
        if any(
            math.hypot(
              person.get('x', 0.0) - neighbor.get('x', 0.0),
              person.get('y', 0.0) - neighbor.get('y', 0.0),
            ) <= HUMAN_CLUSTER_DISTANCE
            for neighbor in neighbor_step.get('people', [])):
          supported[step_idx][person_idx] = True
          break

  max_slots = 0
  for step_idx, step in enumerate(sequence):
    unclustered = [
      person
      for person_idx, person in enumerate(step.get('people', []))
      if supported[step_idx][person_idx]
    ]
    cluster_count = 0
    while unclustered:
      cluster = [unclustered.pop()]
      cluster_count += 1
      expanded = True
      while expanded:
        expanded = False
        remaining = []
        for person in unclustered:
          if any(
              math.hypot(
                person.get('x', 0.0) - member.get('x', 0.0),
                person.get('y', 0.0) - member.get('y', 0.0),
              ) <= HUMAN_CLUSTER_DISTANCE
              for member in cluster):
            cluster.append(person)
            expanded = True
          else:
            remaining.append(person)
        unclustered = remaining
    max_slots = max(max_slots, cluster_count)

  return max_slots

def _rectangle_corners(x, y, length, width, angle):
  half_length = length / 2.0
  half_width = width / 2.0
  cos_angle = math.cos(angle)
  sin_angle = math.sin(angle)
  return [
    (
      x + local_x * cos_angle - local_y * sin_angle,
      y + local_x * sin_angle + local_y * cos_angle,
    )
    for local_x, local_y in (
      (-half_length, -half_width),
      (-half_length, half_width),
      (half_length, -half_width),
      (half_length, half_width),
    )
  ]


def _convex_hull(points):
  points = sorted(set(points))
  if len(points) <= 1:
    return points

  def cross(origin, first, second):
    return (
      (first[0] - origin[0]) * (second[1] - origin[1])
      - (first[1] - origin[1]) * (second[0] - origin[0])
    )

  lower = []
  for point in points:
    while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
      lower.pop()
    lower.append(point)
  upper = []
  for point in reversed(points):
    while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
      upper.pop()
    upper.append(point)
  return lower[:-1] + upper[:-1]


def _static_camera_geometry(walls, sequence):
  geometry = []
  for wall in walls:
    if len(wall) != 4:
      continue
    x1, y1, x2, y2 = wall
    length = math.hypot(x2 - x1, y2 - y1)
    angle = math.atan2(y2 - y1, x2 - x1) if length else 0.0
    geometry.extend(_rectangle_corners(
      (x1 + x2) / 2.0,
      (y1 + y2) / 2.0,
      length,
      0.15,
      angle,
    ))

  objects = next(
    (step.get('objects', []) for step in sequence if step.get('objects', [])),
    [],
  )
  for obj in objects:
    shape = obj.get('shape', {})
    width = shape.get('width', 0.5)
    length = shape.get('length', width)
    geometry.extend(_rectangle_corners(
      obj.get('x', 0.0),
      obj.get('y', 0.0),
      length,
      width,
      obj.get('angle', 0.0),
    ))
  return geometry


def _robot_camera_geometry(robot):
  shape = robot.get('shape', {})
  width = shape.get('width', 0.5)
  length = shape.get('length', width)
  x = robot.get('x', 0.0)
  y = robot.get('y', 0.0)
  angle = robot.get('angle', 0.0)
  if shape.get('type', '').lower() == 'circle':
    radius = width / 2.0
    geometry = [
      (
        x + radius * math.cos(2.0 * math.pi * index / 32.0),
        y + radius * math.sin(2.0 * math.pi * index / 32.0),
      )
      for index in range(32)
    ]
  else:
    geometry = _rectangle_corners(x, y, length, width, angle)
  geometry.extend(_rectangle_corners(
    x + 0.42 * length * math.cos(angle),
    y + 0.42 * length * math.sin(angle),
    0.24 * length,
    0.64 * width,
    angle,
  ))
  return geometry


def _distance_to_bounds(point, bounds):
  min_x, max_x, min_y, max_y = bounds
  dx = max(min_x - point[0], 0.0, point[0] - max_x)
  dy = max(min_y - point[1], 0.0, point[1] - max_y)
  return math.hypot(dx, dy)


def _human_camera_outliers(sequence, static_geometry):
  if any(
    'timestamp' not in step
    or not isinstance(step['timestamp'], (int, float))
    or not math.isfinite(step['timestamp'])
    for step in sequence
  ):
    return set()

  tracks_by_id = {}
  for step_idx, step in enumerate(sequence):
    timestamp = step['timestamp']
    for person_idx, person in enumerate(step.get('people', [])):
      if 'id' not in person:
        continue
      tracks_by_id.setdefault(person['id'], []).append(
        (step_idx, person_idx, timestamp, person)
      )

  tracks = []
  for observations in tracks_by_id.values():
    segments = []
    for observation in observations:
      if (
        not segments
        or observation[2] - segments[-1][-1][2] > HUMAN_SUPPORT_WINDOW
        or observation[2] <= segments[-1][-1][2]
      ):
        segments.append([])
      segments[-1].append(observation)
    tracks.extend(segments)

  velocities = []
  for track in tracks:
    for previous, current in zip(track, track[1:]):
      delta_time = current[2] - previous[2]
      if 0.0 < delta_time <= HUMAN_SUPPORT_WINDOW:
        distance = math.hypot(
          current[3].get('x', 0.0) - previous[3].get('x', 0.0),
          current[3].get('y', 0.0) - previous[3].get('y', 0.0),
        )
        velocities.append(distance / delta_time)

  if velocities:
    ordered_speeds = sorted(velocities)
    # A minority of teleport-like jumps must not inflate the speed baseline.
    baseline_speeds = ordered_speeds[:max(1, (len(ordered_speeds) + 1) // 2)]
    median_speed = statistics.median(baseline_speeds)
    median_deviation = statistics.median(
      abs(speed - median_speed) for speed in baseline_speeds
    )
    speed_limit = max(
      CAMERA_OUTLIER_MIN_SPEED,
      median_speed + 6.0 * 1.4826 * median_deviation,
    )
  else:
    speed_limit = CAMERA_OUTLIER_MIN_SPEED

  base_geometry = list(static_geometry)
  people_geometry = {}
  for step in sequence:
    base_geometry.extend(_robot_camera_geometry(step.get('robot', {})))
    for person in step.get('people', []):
      if 'id' not in person:
        continue
      people_geometry.setdefault(person['id'], []).extend(_rectangle_corners(
        person.get('x', 0.0),
        person.get('y', 0.0),
        CAMERA_HUMAN_FOOTPRINT,
        CAMERA_HUMAN_FOOTPRINT,
        person.get('angle', 0.0),
      ))

  base_bounds = (
    min(point[0] for point in base_geometry),
    max(point[0] for point in base_geometry),
    min(point[1] for point in base_geometry),
    max(point[1] for point in base_geometry),
  ) if base_geometry else None
  people_bounds = {
    person_id: (
      min(point[0] for point in geometry),
      max(point[0] for point in geometry),
      min(point[1] for point in geometry),
      max(point[1] for point in geometry),
    )
    for person_id, geometry in people_geometry.items()
    if geometry
  }

  outliers = set()
  for track in tracks:
    person_id = track[0][3]['id']
    reference_bounds = list(base_bounds) if base_bounds else [
      math.inf, -math.inf, math.inf, -math.inf
    ]
    for other_id, other_bounds in people_bounds.items():
      if other_id == person_id:
        continue
      reference_bounds[0] = min(reference_bounds[0], other_bounds[0])
      reference_bounds[1] = max(reference_bounds[1], other_bounds[1])
      reference_bounds[2] = min(reference_bounds[2], other_bounds[2])
      reference_bounds[3] = max(reference_bounds[3], other_bounds[3])

    if not math.isfinite(reference_bounds[0]):
      continue
    geometry_diagonal = math.hypot(
      reference_bounds[1] - reference_bounds[0],
      reference_bounds[3] - reference_bounds[2],
    )
    distance_limit = max(
      CAMERA_OUTLIER_MIN_DISTANCE,
      CAMERA_OUTLIER_GEOMETRY_FRACTION * geometry_diagonal,
    )
    short_track = track[-1][2] - track[0][2] <= HUMAN_SUPPORT_WINDOW

    for track_idx, (step_idx, person_idx, _, person) in enumerate(track):
      position = (person.get('x', 0.0), person.get('y', 0.0))
      if _distance_to_bounds(position, reference_bounds) <= distance_limit:
        continue

      abrupt_isolated_jump = False
      if 0 < track_idx < len(track) - 1:
        previous = track[track_idx - 1]
        current = track[track_idx]
        following = track[track_idx + 1]
        before_dt = current[2] - previous[2]
        after_dt = following[2] - current[2]
        if (
          0.0 < before_dt <= HUMAN_SUPPORT_WINDOW
          and 0.0 < after_dt <= HUMAN_SUPPORT_WINDOW
        ):
          before_speed = math.hypot(
            current[3].get('x', 0.0) - previous[3].get('x', 0.0),
            current[3].get('y', 0.0) - previous[3].get('y', 0.0),
          ) / before_dt
          after_speed = math.hypot(
            following[3].get('x', 0.0) - current[3].get('x', 0.0),
            following[3].get('y', 0.0) - current[3].get('y', 0.0),
          ) / after_dt
          abrupt_isolated_jump = (
            before_speed > speed_limit and after_speed > speed_limit
          )

      if short_track or abrupt_isolated_jump:
        outliers.add((step_idx, person_idx))

  return outliers


def _compute_camera_pose(walls, sequence):
  static_geometry = _static_camera_geometry(walls, sequence)
  human_outliers = _human_camera_outliers(sequence, static_geometry)
  bounds = list(static_geometry)
  orientation_candidates = set()
  objects = next(
    (step.get('objects', []) for step in sequence if step.get('objects', [])),
    [],
  )
  for obj in objects:
    angle = obj.get('angle', 0.0)
    orientation_candidates.add(angle % math.pi)

  for wall in walls:
    if len(wall) == 4:
      orientation_candidates.add(
        math.atan2(wall[3] - wall[1], wall[2] - wall[0]) % math.pi
      )

  for step_idx, step in enumerate(sequence):
    robot = step.get('robot', {})
    bounds.extend(_robot_camera_geometry(robot))
    for person_idx, person in enumerate(step.get('people', [])):
      if (step_idx, person_idx) in human_outliers:
        continue
      angle = person.get('angle', 0.0)
      bounds.extend(_rectangle_corners(
        person.get('x', 0.0),
        person.get('y', 0.0),
        CAMERA_HUMAN_FOOTPRINT,
        CAMERA_HUMAN_FOOTPRINT,
        angle,
      ))

  if not bounds:
    bounds = [(-5.0, -5.0), (5.0, 5.0)]
  else:
    # Projection extrema depend only on the convex hull, not interior points.
    bounds = _convex_hull(bounds)

  horizontal_fov = math.radians(60.0)
  aspect_ratio = 16.0 / 9.0
  vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) / aspect_ratio)
  horizontal_scale = 2.0 * math.tan(horizontal_fov / 2.0)
  vertical_scale = 2.0 * math.tan(vertical_fov / 2.0)
  padding = max(
    0.15,
    0.01 * max(
      max(point[0] for point in bounds) - min(point[0] for point in bounds),
      max(point[1] for point in bounds) - min(point[1] for point in bounds),
    ),
  )

  def frame_at_angle(angle):
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    projected_x = [
      point[0] * cos_angle + point[1] * sin_angle
      for point in bounds
    ]
    projected_y = [
      -point[0] * sin_angle + point[1] * cos_angle
      for point in bounds
    ]
    min_x, max_x = min(projected_x), max(projected_x)
    min_y, max_y = min(projected_y), max(projected_y)
    span_x = max_x - min_x + 2.0 * padding
    span_y = max_y - min_y + 2.0 * padding
    # Wall tops are closer to the camera than the ground footprint, so add their height
    distance = CAMERA_DISTANCE_MARGIN * max(
      span_x / horizontal_scale,
      span_y / vertical_scale,
    ) + CAMERA_WALL_HEIGHT
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    world_x = center_x * cos_angle - center_y * sin_angle
    world_y = center_x * sin_angle + center_y * cos_angle
    return distance, world_x, world_y

  angle_step = math.pi / CAMERA_ORIENTATION_STEPS
  sampled_angles = [
    index * angle_step for index in range(CAMERA_ORIENTATION_STEPS)
  ]
  sampled_angles.extend(orientation_candidates)
  best_angle = min(
    sampled_angles,
    key=lambda angle: frame_at_angle(angle)[0],
  )
  refine_half_width = angle_step
  left = best_angle - refine_half_width
  right = best_angle + refine_half_width
  golden_ratio = (math.sqrt(5.0) - 1.0) / 2.0
  first = right - golden_ratio * (right - left)
  second = left + golden_ratio * (right - left)
  for _ in range(32):
    if frame_at_angle(first)[0] <= frame_at_angle(second)[0]:
      right = second
      second = first
      first = right - golden_ratio * (right - left)
    else:
      left = first
      first = second
      second = left + golden_ratio * (right - left)
  camera_angle = (left + right) / 2.0
  camera_height, camera_x, camera_y = frame_at_angle(camera_angle)
  camera_roll = (math.pi / 2.0 - camera_angle + math.pi / 2.0) % math.pi - math.pi / 2.0
  return camera_x, camera_y, camera_height, camera_roll


def _goal_angle_thresholds(sequence):
    thresholds = set()
    for step in sequence:
        goal = step.get('goal', {})
        if not isinstance(goal, dict):
            continue
        if not all(key in goal for key in ('x', 'y', 'angle')):
            continue
        threshold = abs(float(goal.get('angle_threshold', 0.1)))
        if not math.isfinite(threshold):
            raise ValueError("Goal angle thresholds must be finite numbers")
        thresholds.add(round(min(threshold, math.pi), 12))
    return sorted(thresholds)


def _write_goal_angle_mesh(path, angle_threshold):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    if angle_threshold >= math.pi:
        start_angle = 0.0
        end_angle = 2.0 * math.pi
    else:
        start_angle = -angle_threshold
        end_angle = angle_threshold

    sweep = end_angle - start_angle
    segment_count = max(
        4,
        math.ceil(GOAL_ANGLE_MESH_SEGMENTS * sweep / (2.0 * math.pi)),
    )
    points = [
        (
            math.cos(start_angle + sweep * index / segment_count),
            math.sin(start_angle + sweep * index / segment_count),
        )
        for index in range(segment_count + 1)
    ]

    with path.open('w', encoding='ascii') as mesh_file:
        mesh_file.write("solid goal_angle_sector\n")
        for index in range(segment_count):
            x1, y1 = points[index]
            x2, y2 = points[index + 1]
            mesh_file.write(
                "  facet normal 0 0 1\n"
                "    outer loop\n"
                "      vertex 0 0 0\n"
                f"      vertex {x1:.9g} {y1:.9g} 0\n"
                f"      vertex {x2:.9g} {y2:.9g} 0\n"
                "    endloop\n"
                "  endfacet\n"
            )
        mesh_file.write("endsolid goal_angle_sector\n")


def create_world(json_path, output_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    if isinstance(data, dict):
        walls = data.get('walls', [])
        sequence = data.get('sequence', [])
    else:
        walls = []
        sequence = data if isinstance(data, list) else []

    camera_x, camera_y, camera_z, camera_roll = _compute_camera_pose(walls, sequence)
    human_slot_count = _compute_human_slot_count(sequence)
    output_path = Path(output_path)
    mesh_dir = Path(__file__).resolve().parent / "meshes"
    goal_angle_meshes = []
    for mesh_index, angle_threshold in enumerate(_goal_angle_thresholds(sequence)):
        if angle_threshold <= 0.0:
            continue
        mesh_path = mesh_dir / f"{output_path.stem}_goal_angle_{mesh_index:02d}.stl"
        _write_goal_angle_mesh(mesh_path, angle_threshold)
        goal_angle_meshes.append((angle_threshold, mesh_path.resolve()))

    goal_angle_mesh_xml = "".join(
        f"""      <goal_angle_mesh>
        <angle_threshold>{angle_threshold:.12g}</angle_threshold>
        <uri>{escape(str(mesh_path))}</uri>
      </goal_angle_mesh>
"""
        for angle_threshold, mesh_path in goal_angle_meshes
    )

    world_template = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='default'>
    <gui>
      <camera name='user_camera'>
        <pose>{camera_x} {camera_y} {camera_z} {camera_roll} {math.pi / 2.0} 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <plugin name="socnav_plugin" filename="libSocNavPlugin.so">
      <robot_name>robot</robot_name>
      <trajectory_file>{json_path}</trajectory_file>
      <human_slots>{human_slot_count}</human_slots>
{goal_angle_mesh_xml}
    </plugin>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/kitchen.material</uri>
              <name>Kitchen/CounterTop_H</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

    if sequence:

        ################################################
        # Add robot model based on the first step in the sequence
        ################################################
        robot = sequence[0].get('robot', {})
        robot_shape = robot.get('shape', {})
        robot_type = robot_shape.get('type', '').lower()
        robot_x = robot.get('x', 0.0)
        robot_y = robot.get('y', 0.0)
        robot_angle = robot.get('angle', 0.0)
        robot_width = robot_shape.get('width', 0.5)
        robot_length = robot_shape.get('length', robot_width)
        robot_height = 1.0 #max(robot_width, robot_length)

        if robot_type == 'circle':
            robot_geometry = f"""            <cylinder>
              <radius>{robot_width / 2.0}</radius>
              <length>{robot_height}</length>
            </cylinder>"""
        elif robot_type in ('square', 'rectangle'):
            robot_geometry = f"""            <box>
              <size>{robot_length} {robot_width} {robot_height}</size>
            </box>"""
        else:
            robot_geometry = f"""            <box>
              <size>{robot_length} {robot_width} {robot_height}</size>
            </box>"""

        world_template += f"""    <model name='robot'>
      <pose>{robot_x} {robot_y} {robot_height / 2.0} 0 0 {robot_angle}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
{robot_geometry}
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
{robot_geometry}
          </geometry>
          <material>
            <ambient>0.1 0.4 0.8 1</ambient>
            <diffuse>0.1 0.4 0.8 1</diffuse>
          </material>
        </visual>
        <visual name='robot_top'>
          <pose>0 0 {robot_height * 0.58} 0 0 0</pose>
          <geometry><cylinder><radius>{min(robot_length, robot_width) * 0.36}</radius><length>{robot_height * 0.16}</length></cylinder></geometry>
          <material><ambient>0.04 0.08 0.12 1</ambient><diffuse>0.04 0.08 0.12 1</diffuse></material>
        </visual>
        <visual name='robot_front'>
          <pose>{robot_length * 0.42} 0 {robot_height * 0.44} 0 0 0</pose>
          <geometry><box><size>{robot_length * 0.24} {robot_width * 0.64} {robot_height * 0.24}</size></box></geometry>
          <material><ambient>0.9 0.12 0.08 1</ambient><diffuse>0.9 0.12 0.08 1</diffuse></material>
        </visual>
        <visual name='robot_sensor'>
          <pose>0 0 {robot_height * 0.78} 0 0 0</pose>
          <geometry><cylinder><radius>{min(robot_length, robot_width) * 0.07}</radius><length>{robot_height * 0.24}</length></cylinder></geometry>
          <material><ambient>0.1 0.9 0.9 1</ambient><diffuse>0.1 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

        ################################################
        # Add a fixed model pool sized by supported concurrent occupancy.
        ################################################
        skin_filenames = (
            "elegant_man.dae",
            "casual_man.dae",
            "elegant_woman.dae",
            "regular_man.dae",
        )
        for person_idx in range(human_slot_count):
            skin_filename = skin_filenames[person_idx % len(skin_filenames)]
            world_template += f"""    <actor name="person_slot_{person_idx}">
      <pose>0 0 -1000 0 0 0</pose>
      <skin>
        <filename>{skin_filename}</filename>
        <scale>1</scale>
      </skin>
      <animation name="walking">
        <filename>07_01-walk.bvh</filename>
        <scale>1.0</scale>
      </animation>
    </actor>
"""

        objects = []
        for step in sequence:
            if step.get('objects', []):
                objects = step['objects']
                break

        object_heights = {
            "shelf": 1.8,
            "table": 0.75,
            "chair": 0.9,
            "tv": 0.6,
        }
        object_colors = {
            "shelf": "0.55 0.36 0.2 1",
            "table": "0.65 0.45 0.27 1",
            "chair": "0.25 0.4 0.62 1",
            "tv": "0.08 0.08 0.08 1",
        }
        use_detailed_furniture_visuals = True
        furniture_visual_meshes = {
          "chair": (
            "model://aws_robomaker_residential_ChairA_01/meshes/aws_ChairA_01_visual.DAE",
            (0.437246338, 0.473526596, 0.913194336),
          ),
          "table": (
            "model://aws_robomaker_residential_KitchenTable_01/meshes/aws_KitchenTable_01_visual.DAE",
            (0.7, 1.8, 0.80135675),
          ),
        }
        for object_idx, obj in enumerate(objects):
            object_type = obj.get('type', 'object').lower()
            shape = obj.get('shape', {})
            object_width = shape.get('width', 0.5)
            object_length = shape.get('length', object_width)
            object_height = object_heights.get(object_type, 0.75)
            object_x = obj.get('x', 0.0)
            object_y = obj.get('y', 0.0)
            object_angle = obj.get('angle', 0.0)
            object_color = object_colors.get(object_type, "0.55 0.55 0.55 1")
            object_geometry = f"""            <box>
              <size>{object_length} {object_width} {object_height}</size>
            </box>"""
            visual_geometry = object_geometry
            object_material = f"""          <material>
            <ambient>{object_color}</ambient>
            <diffuse>{object_color}</diffuse>
          </material>"""
            visual_pose = ""
            if use_detailed_furniture_visuals and object_type in furniture_visual_meshes:
                mesh_uri, (mesh_length, mesh_width, mesh_height) = furniture_visual_meshes[object_type]
                visual_scale = (
                    object_length / mesh_length,
                    object_width / mesh_width,
                    object_height / mesh_height,
                )
                visual_geometry = f"""            <mesh>
              <uri>{mesh_uri}</uri>
              <scale>{visual_scale[0]} {visual_scale[1]} {visual_scale[2]}</scale>
            </mesh>"""
                object_material = ""
                visual_pose = f"          <pose>0 0 {-object_height / 2.0} 0 0 0</pose>"
            world_template += f"""    <model name="static_object_{object_idx}">
      <static>true</static>
      <pose>{object_x} {object_y} {object_height / 2.0} 0 0 {object_angle}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
{object_geometry}
          </geometry>
        </collision>
        <visual name="visual">
{visual_pose}
          <geometry>
{visual_geometry}
          </geometry>
{object_material}
        </visual>
      </link>
    </model>
"""

    world_template += """    <model name='simple_world_test'>
      <pose>0 0 0 0 -0 0</pose>
"""

    ################################################
    # Add walls to the world
    ################################################
    for idx, wall in enumerate(walls):
        if len(wall) == 4:
            x1, y1, x2, y2 = wall
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            theta = math.atan2(y2 - y1, x2 - x1)
            
            # Using 0.15 for width and 2.5 for height as in word_test.world
            width = 0.15
            height = 2.5
            
            wall_xml = f"""      <link name='Wall_{idx}'>
        <collision name='Wall_{idx}_Collision'>
          <geometry>
            <box>
              <size>{length} {width} {height}</size>
            </box>
          </geometry>
          <pose>0 0 {height/2} 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_{idx}_Visual'>
          <pose>0 0 {height/2} 0 -0 0</pose>
          <geometry>
            <box>
              <size>{length} {width} {height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>{cx} {cy} 0 0 -0 {theta}</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
"""
            world_template += wall_xml

    world_template += """      <static>1</static>
    </model>
  </world>
</sdf>
"""

    with open(output_path, 'w') as f:
        f.write(world_template)
    
    print(f"World successfully saved to {output_path}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 gazebo_world_builder.py <path_to_json> [output_path]")
        sys.exit(1)
        
    json_path = sys.argv[1]
    
    output_path = sys.argv[2] if len(sys.argv) > 2 else 'socnav.world'
    
    create_world(json_path, output_path)
