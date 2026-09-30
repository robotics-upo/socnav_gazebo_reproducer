import json
import math
import sys
import os

def _compute_camera_pose(walls, sequence):
  bounds = []
  scene_height = 0.0
  wall_axis_x = 0.0
  wall_axis_y = 0.0
  total_wall_length = 0.0
  longest_wall_length = 0.0
  longest_wall_angle = 0.0

  def include_rectangle(x, y, length, width, angle, height, is_circle=False):
    nonlocal scene_height
    if is_circle:
      half_x = half_y = max(length, width) / 2.0
    else:
      cos_angle = abs(math.cos(angle))
      sin_angle = abs(math.sin(angle))
      half_x = (cos_angle * length + sin_angle * width) / 2.0
      half_y = (sin_angle * length + cos_angle * width) / 2.0
    bounds.extend(((x - half_x, y - half_y), (x + half_x, y + half_y)))
    scene_height = max(scene_height, height)

  for wall in walls:
    if len(wall) == 4:
      x1, y1, x2, y2 = wall
      length = math.hypot(x2 - x1, y2 - y1)
      if length:
        angle = math.atan2(y2 - y1, x2 - x1)
        wall_axis_x += length * math.cos(2.0 * angle)
        wall_axis_y += length * math.sin(2.0 * angle)
        total_wall_length += length
        if length > longest_wall_length:
          longest_wall_length = length
          longest_wall_angle = angle
      if length:
        half_width_x = 0.075 * abs(y2 - y1) / length
        half_width_y = 0.075 * abs(x2 - x1) / length
      else:
        half_width_x = half_width_y = 0.075
      bounds.extend((
        (min(x1, x2) - half_width_x, min(y1, y2) - half_width_y),
        (max(x1, x2) + half_width_x, max(y1, y2) + half_width_y),
      ))
      scene_height = max(scene_height, 2.5)

  objects = next((step.get('objects', []) for step in sequence if step.get('objects', [])), [])
  object_heights = {"shelf": 1.8, "table": 0.75, "chair": 0.9, "tv": 0.6}
  for obj in objects:
    shape = obj.get('shape', {})
    object_type = obj.get('type', 'object').lower()
    width = shape.get('width', 0.5)
    length = shape.get('length', width)
    include_rectangle(
      obj.get('x', 0.0),
      obj.get('y', 0.0),
      length,
      width,
      obj.get('angle', 0.0),
      object_heights.get(object_type, 0.75),
    )

  for step in sequence:
    robot = step.get('robot', {})
    if robot:
      shape = robot.get('shape', {})
      width = shape.get('width', 0.5)
      length = shape.get('length', width)
      include_rectangle(
        robot.get('x', 0.0),
        robot.get('y', 0.0),
        length,
        width,
        robot.get('angle', 0.0),
        1.0,
        shape.get('type', '').lower() == 'circle',
      )
    for person in step.get('people', []):
      include_rectangle(
        person.get('x', 0.0),
        person.get('y', 0.0),
        0.7,
        0.7,
        person.get('angle', 0.0),
        2.5,
      )

  if not bounds:
    bounds = [(-5.0, -5.0), (5.0, 5.0)]

  min_x = min(point[0] for point in bounds)
  max_x = max(point[0] for point in bounds)
  min_y = min(point[1] for point in bounds)
  max_y = max(point[1] for point in bounds)
  padding = max(0.15, 0.01 * max(max_x - min_x, max_y - min_y))
  span_x = max_x - min_x + 2.0 * padding
  span_y = max_y - min_y + 2.0 * padding
  horizontal_fov = math.radians(60.0)
  aspect_ratio = 16.0 / 9.0
  vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) / aspect_ratio)
  camera_distance = max(
    span_x / (2.0 * math.tan(horizontal_fov / 2.0)),
    span_y / (2.0 * math.tan(vertical_fov / 2.0)),
  )
  camera_height = scene_height + max(1.0, camera_distance * 1.05)
  camera_roll = 0.0
  if total_wall_length:
    axis_strength = math.hypot(wall_axis_x, wall_axis_y)
    if axis_strength < 0.05 * total_wall_length:
      wall_axis_angle = longest_wall_angle
    else:
      wall_axis_angle = 0.5 * math.atan2(wall_axis_y, wall_axis_x)
    camera_roll = math.pi / 2.0 - wall_axis_angle
    camera_roll = (camera_roll + math.pi / 2.0) % math.pi - math.pi / 2.0
  return (min_x + max_x) / 2.0, (min_y + max_y) / 2.0, camera_height, camera_roll


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
        # Add people models based on the first step in the sequence
        ################################################
        people = []
        for step in sequence:
            if step.get('people', []):
                people = step['people']
                break

        skin_filenames = (
            "elegant_man.dae",
            "casual_man.dae",
            "elegant_woman.dae",
            "regular_man.dae",
        )
        for person_idx, p in enumerate(people):
            pid = p.get('id', 0)
            px = p.get('x', 0.0)
            py = p.get('y', 0.0)
            ptheta = p.get('angle', 0.0)
            skin_filename = skin_filenames[person_idx % len(skin_filenames)]
            world_template += f"""    <actor name="person_{pid}">
      <pose>{px} {py} 1.25 0 0 {ptheta}</pose>
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
