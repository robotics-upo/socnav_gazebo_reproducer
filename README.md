# socnav_gazebo_reproducer

A ROS 2 package containing a Gazebo plugin (`SocNavPlugin`) that reads trajectory JSON files from `SocNav3 dataset` and plays them back in Gazebo.

## Usage

The package has 3 tools:

### Gazebo world builder

This python script creates the Gazebo world file (.world) from the environment description found in the json file. It also add the human agents, the robot and the socnav plugin to control them. 

```
python3 gazebo_world_builder <path_to_json_file> [path_to_output_world_file]
```

### Generate worlds from survey CSVs

From the workspace root, preview the worlds selected by the survey CSV files:

```
python3 src/socnav_gazebo_reproducer/generate_survey_worlds.py --dry-run
```

Run the command without `--dry-run` to generate worlds in
`src/socnav_gazebo_reproducer/worlds`. Rows without a matching JSON, or with
multiple JSON files sharing the same numeric prefix, are reported and skipped
by default. Use `--first-match` to select the first matching JSON path in sorted
order for duplicate prefixes.

### socnav_plugin

Yhe world builder will add the socnav_plugin to the created world file and specify the `trajectory_file`:

```xml
<plugin name="socnav_plugin" filename="libSocNavPlugin.so">
  <robot_name>robot</robot_name>
  <trajectory_file>/home/hunav_gz_classic_ws/SocNav3_trajectories_labeled/src3/000001054_7d43a964.json</trajectory_file>
</plugin>
```

Then, to launch the world and reproduce the trajectory on Gazebo, you can use the launch file: 
```
ros2 launch socnav_gazebo_reproducer socnav_gazebo_world.launch.py 
```
Using the argument `world` to indicate the world file to run. 


### AutoRecord_guiplugin

There is a second plugin in charge of recording a video (.mp4) of the trajectory. But it is transparent to the user. It is launched along with the gzclient in the launch file. 
