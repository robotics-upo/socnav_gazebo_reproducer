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

The script reads CSV files from
`/home/hunav_gz_classic_ws/SocNav3_trajectories/index_trajectories_survey` by
default. Each CSV must contain a `file` column with video filenames such as
`000000001.mp4`. It looks for the matching `000000001.json` in the `src1`
through `src6` directories under
`/home/hunav_gz_classic_ws/SocNav3_trajectories`.

Run without `--dry-run` to generate worlds in
`src/socnav_gazebo_reproducer/worlds/survey_worlds`, named after their video
IDs (for example, `000000001.world`). Use `--csv-dir`, `--json-dir`, and
`--output-dir` to override the CSV directory, dataset directory, and output
directory. For example:

```
python3 src/socnav_gazebo_reproducer/generate_survey_worlds.py \
  --csv-dir /path/to/csvs \
  --json-dir /path/to/SocNav3_trajectories \
  --output-dir /path/to/survey_worlds
```

World generation also writes the goal angle-threshold sector meshes to
`src/socnav_gazebo_reproducer/meshes` and records their paths in each world.
The green sector uses the same radius as the red positional-tolerance circle.
Regenerate existing worlds after updating the builder so they use the sector
visuals. Thresholds at or above π are represented by a full disk.

IDs without a matching JSON, or with matching JSON files in multiple source
directories, are reported and skipped by default. Use `--first-match` to select
the first matching JSON path in sorted order for duplicate IDs.

### Launch a range of generated survey worlds

The script `run_survey_worlds.sh` launches a consecutive subset of the generated
world files found in `src/socnav_gazebo_reproducer/worlds/survey_worlds`.

From the workspace root, run:

```
./src/socnav_gazebo_reproducer/run_survey_worlds.sh
```

It will ask for:

- the index of the first world to launch
- the index of the last world to launch

The script validates the indices, sorts the available `.world` files, and then
launches each selected world through:

```
ros2 launch socnav_gazebo_reproducer socnav_gazebo_world.launch.py "world:=<path_to_world>"
```

This is useful to automatically replay a survey range without manually launching
one world at a time. The script sources the workspace setup file, checks that
`ros2` and `setsid` are available, and stops the current launch cleanly before
continuing to the next world.

### socnav_plugin

Yhe world builder will add the socnav_plugin to the created world file and specify the `trajectory_file`:

```xml
<plugin name="socnav_plugin" filename="libSocNavPlugin.so">
  <robot_name>robot</robot_name>
  <trajectory_file>/home/hunav_gz_classic_ws/SocNav3_trajectories/src3/000000001.json</trajectory_file>
</plugin>
```

Then, to launch the world and reproduce the trajectory on Gazebo, you can use the launch file: 
```
ros2 launch socnav_gazebo_reproducer socnav_gazebo_world.launch.py 
```
Using the argument `world` to indicate the world file to run. 


### AutoRecord_guiplugin

There is a second plugin in charge of recording a video (.mp4) of the trajectory. But it is transparent to the user. It is launched along with the gzclient in the launch file. 
