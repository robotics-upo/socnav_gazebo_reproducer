import json
import math
import sys
import os

def create_world(json_path, output_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    if isinstance(data, dict):
        walls = data.get('walls', [])
        sequence = data.get('sequence', [])
    else:
        walls = []
        sequence = data if isinstance(data, list) else []

    world_template = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='default'>
    <gui name="default_gui">
      <plugin name="AutoRRecordGuiPlugin" filename="libAutoRecordGuiPlugin.so"/>
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
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>9.05907 4.1081 10.8863 0 0.915644 -2.94299</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
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
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
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
        robot_height = max(robot_width, robot_length)

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
                
        for p in people:
            pid = p.get('id', 0)
            px = p.get('x', 0.0)
            py = p.get('y', 0.0)
            ptheta = p.get('angle', 0.0)
            world_template += f"""    <actor name="person_{pid}">
      <pose>{px} {py} 1.25 0 0 {ptheta}</pose>
      <skin>
        <filename>elegant_man.dae</filename>
        <scale>1</scale>
      </skin>
      <animation name="walking">
        <filename>07_01-walk.bvh</filename>
        <scale>1.0</scale>
      </animation>
    </actor>
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
