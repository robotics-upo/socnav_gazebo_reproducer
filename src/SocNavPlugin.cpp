#include <socnav_gazebo_reproducer/SocNavPlugin.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <cmath>
#include <filesystem>
#include <thread>
#include <chrono>
#include <csignal>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


namespace socnav_gazebo_reproducer
{

struct RobotState {
  double x, y, theta;
};
struct HumanState {
  int id;
  double x, y, theta;
  double speed = 0.0; // linear speed [m/s], computed from consecutive positions
};
struct GoalState {
  double x = 0.0;
  double y = 0.0;
  double angle = 0.0;
  double robot_footprint_radius = 0.25;
  double pos_threshold = 0.1;
  double angle_threshold = 0.1;
  bool valid = false;
};
struct SmoothedHumanState {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  bool initialized = false;
};
struct TrajectoryStep {
  double timestamp;
  RobotState robot;
  std::vector<HumanState> people;
  GoalState goal;
};

class SocNavPluginPrivate
{
public:
  gazebo::physics::WorldPtr world;
  gazebo::event::ConnectionPtr connection;
  gazebo_ros::Node::SharedPtr rosnode;
  
  std::vector<TrajectoryStep> trajectory_steps;
  double start_timestamp = 0.0;
  bool is_loaded = false;
  size_t total_steps = 0;

  /// Index of the trajectory step currently being applied.
  size_t current_step_idx = 0;

  /// Per-actor accumulated animation script time [s].
  std::map<int, double> actor_script_time;

  /// Filtered pose used to hide small, fast tracking fluctuations.
  std::map<int, SmoothedHumanState> smoothed_humans;

  /// Simulation time at the last OnUpdate call (for dt computation).
  double last_sim_time = -1.0;

  /// Reference walking speed: animation plays at 1x when agent moves at this speed.
  static constexpr double kRefWalkSpeed = 1.0; // m/s

  /// Time constant for the human pose low-pass filter.
  static constexpr double kHumanSmoothingTime = 0.12; // s

  std::string robot_name = "robot";
  gazebo::physics::ModelPtr goal_area_model;
  gazebo::physics::ModelPtr goal_tolerance_model;
  gazebo::physics::ModelPtr goal_direction_model;
  bool goal_models_requested = false;

  // Nodes and publishers for Gazebo Transport
  gazebo::transport::NodePtr gz_node;
  gazebo::transport::PublisherPtr record_pub;
  gazebo::transport::SubscriberPtr record_status_sub;
  bool is_recording = false;
  bool start_ack_received = false;
  bool recording_finished = false;
  bool stop_ack_received = false;
  bool shutdown_requested = false;
  double recording_signal_time = -1.0;
  double last_start_signal_time = -1.0;
  double stop_signal_time = -1.0;
  std::string json_path;
  
  void OnUpdate(const gazebo::common::UpdateInfo& _info);
  void OnRecordStatus(ConstGzStringPtr &_msg);
  void ApplyStep(const TrajectoryStep& step, double dt);
  void ApplyGoal(const GoalState& goal);
  void EnsureGoalModels();
  void SetupActorAnimations();
  
  inline double normalizeAngle(double a)
  {
    double value = a;
    while (value <= -M_PI)
      value += 2 * M_PI;
    while (value > M_PI)
      value -= 2 * M_PI;
    return value;
  }
};

SocNavPlugin::SocNavPlugin() : impl_(std::make_unique<SocNavPluginPrivate>())
{
}

SocNavPlugin::~SocNavPlugin()
{
}

void SocNavPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  impl_->world = _world;
  impl_->rosnode = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(impl_->rosnode->get_logger(), "Loading SocNavPlugin...");

  if (_sdf->HasElement("robot_name")) {
    impl_->robot_name = _sdf->Get<std::string>("robot_name");
  }

  impl_->json_path = "";
  if (_sdf->HasElement("trajectory_file")) {
    impl_->json_path = _sdf->Get<std::string>("trajectory_file");
  } else {
    RCLCPP_ERROR(impl_->rosnode->get_logger(), "No <trajectory_file> provided!");
    return;
  }

  std::ifstream f(impl_->json_path);
  if (!f.is_open()) {
    RCLCPP_ERROR(impl_->rosnode->get_logger(), "Could not open file: %s", impl_->json_path.c_str());
    return;
  }

  nlohmann::json data;
  try {
    f >> data;
  } catch (nlohmann::json::parse_error& e) {
    RCLCPP_ERROR(impl_->rosnode->get_logger(), "JSON parse error: %s", e.what());
    return;
  }

  nlohmann::json sequence_data;
  if (data.is_array()) {
    sequence_data = data;
  } else if (data.is_object() && data.contains("sequence")) {
    sequence_data = data["sequence"];
  }

  if (sequence_data.is_array()) {
    for (auto& item : sequence_data) {
      if (!item.contains("timestamp") || !item.contains("robot") || !item.contains("people")) continue;
      
      TrajectoryStep step;
      step.timestamp = item["timestamp"].get<double>();
      
      auto& robot_json = item["robot"];
      step.robot.x = robot_json["x"].get<double>();
      step.robot.y = robot_json["y"].get<double>();
      step.robot.theta = robot_json["angle"].get<double>();
      if (robot_json.contains("shape") && robot_json["shape"].is_object()) {
        const auto& shape_json = robot_json["shape"];
        const double width = shape_json.value("width", 0.5);
        const double length = shape_json.value("length", width);
        const std::string shape_type = shape_json.value("type", std::string{});
        step.goal.robot_footprint_radius = shape_type == "circle"
        ? std::max(width, length) / 2.0
        : std::hypot(length, width) / 2.0;
      }

      if (item.contains("goal") && item["goal"].is_object()) {
        const auto& goal_json = item["goal"];
        if (goal_json.contains("x") && goal_json.contains("y") &&
            goal_json.contains("angle")) {
          step.goal.x = goal_json["x"].get<double>();
          step.goal.y = goal_json["y"].get<double>();
          step.goal.angle = goal_json["angle"].get<double>();
          if (goal_json.contains("pos_threshold"))
            step.goal.pos_threshold = goal_json["pos_threshold"].get<double>();
          if (goal_json.contains("angle_threshold"))
            step.goal.angle_threshold = goal_json["angle_threshold"].get<double>();
          step.goal.valid = true;
        }
      }
      
      auto& people_json = item["people"];
      for (auto& person_json : people_json) {
        HumanState human;
        human.id = person_json["id"].get<int>();
        human.x = person_json["x"].get<double>();
        human.y = person_json["y"].get<double>();
        human.theta = person_json["angle"].get<double>();
        step.people.push_back(human);
      }
      
      impl_->trajectory_steps.push_back(step);
    }
  }

  if (impl_->trajectory_steps.empty()) {
    RCLCPP_ERROR(impl_->rosnode->get_logger(), "No trajectories found in file!");
    return;
  }

  // Compute per-person speeds from consecutive position differences.
  // Speed is stored in the *later* step (the one it applies to when moving).
  for (size_t si = 1; si < impl_->trajectory_steps.size(); ++si) {
    auto& cur  = impl_->trajectory_steps[si];
    auto& prev = impl_->trajectory_steps[si - 1];
    double dt  = cur.timestamp - prev.timestamp;
    if (dt <= 0.0) continue;

    // Build a fast lookup: id -> position in prev step
    std::map<int, std::pair<double,double>> prev_pos;
    for (const auto& p : prev.people) {
      prev_pos[p.id] = {p.x, p.y};
    }
    for (auto& p : cur.people) {
      auto it = prev_pos.find(p.id);
      if (it != prev_pos.end()) {
        double dx = p.x - it->second.first;
        double dy = p.y - it->second.second;
        p.speed = std::sqrt(dx*dx + dy*dy) / dt;
      }
    }
  }

  impl_->start_timestamp = impl_->trajectory_steps.front().timestamp;
  impl_->is_loaded = true;
  impl_->total_steps = impl_->trajectory_steps.size();
  RCLCPP_INFO(impl_->rosnode->get_logger(), "Loaded %zu trajectory steps.", impl_->trajectory_steps.size());

  impl_->connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SocNavPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  // Register custom trajectory (walking animation) for each actor once.
  impl_->SetupActorAnimations();


  // Initialize Gazebo Transport Node
  impl_->gz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  impl_->gz_node->Init(impl_->world->Name());

  // Publish to the record video topic
  impl_->record_pub = impl_->gz_node->Advertise<gazebo::msgs::GzString>("~/record_video");
  impl_->record_status_sub = impl_->gz_node->Subscribe(
    "~/record_video_status", &SocNavPluginPrivate::OnRecordStatus, impl_.get());

}

void SocNavPlugin::Reset()
{
  impl_->current_step_idx = 0;
  impl_->last_sim_time = -1.0;
  impl_->is_recording = false;
  impl_->start_ack_received = false;
  impl_->recording_finished = false;
  impl_->stop_ack_received = false;
  impl_->shutdown_requested = false;
  impl_->recording_signal_time = -1.0;
  impl_->last_start_signal_time = -1.0;
  impl_->stop_signal_time = -1.0;
  impl_->actor_script_time.clear();
  impl_->smoothed_humans.clear();
}

void SocNavPluginPrivate::OnRecordStatus(ConstGzStringPtr &_msg)
{
  RCLCPP_INFO(rosnode->get_logger(),
    "Recording GUI status received: %s", _msg->data().c_str());
  if (_msg->data() == "stopped")
    stop_ack_received = true;
  else if (_msg->data() == "started") {
    start_ack_received = true;
    is_recording = true;
    recording_signal_time = last_sim_time;
    RCLCPP_INFO(rosnode->get_logger(), "Recording GUI confirmed video start.");
  }
}

void SocNavPluginPrivate::SetupActorAnimations()
{
  // Collect all unique person IDs across all trajectory steps
  std::set<int> ids;
  for (const auto& step : trajectory_steps) {
    for (const auto& p : step.people) ids.insert(p.id);
  }

  for (int id : ids) {
    std::string model_name = "person_" + std::to_string(id);
    gazebo::physics::ModelPtr person_model = world->ModelByName(model_name);
    if (!person_model) continue;

    gazebo::physics::ActorPtr actor =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(person_model);
    if (!actor) continue;

    // Register the walking animation once; SetScriptTime will drive playback
    gazebo::physics::TrajectoryInfoPtr ti(new gazebo::physics::TrajectoryInfo());
    ti->type     = "walking"; // must match the animation name in the SDF
    ti->duration = 1.0;
    actor->SetCustomTrajectory(ti);

    // Initialise the script-time accumulator for this actor
    actor_script_time[id] = 0.0;

    RCLCPP_INFO(rosnode->get_logger(),
      "Registered walking animation for actor person_%d", id);
  }
}

void SocNavPluginPrivate::EnsureGoalModels()
{
  if (!goal_models_requested) {
    const std::string area_sdf_xml = R"(<sdf version='1.6'>
      <model name='socnav_goal_area'>
        <static>true</static>
        <link name='link'>
          <visual name='visual'>
            <geometry><cylinder><radius>1.0</radius><length>0.015</length></cylinder></geometry>
            <material>
              <ambient>0.1 0.8 0.1 1</ambient>
              <diffuse>0.1 0.8 0.1 1</diffuse>
              <transparency>0.55</transparency>
            </material>
          </visual>
        </link>
      </model>
    </sdf>)";
    const std::string tolerance_sdf_xml = R"(<sdf version='1.6'>
      <model name='socnav_goal_tolerance'>
        <static>true</static>
        <link name='link'>
          <visual name='visual'>
            <geometry><cylinder><radius>1.0</radius><length>0.015</length></cylinder></geometry>
            <material>
              <ambient>0.9 0.1 0.1 1</ambient>
              <diffuse>0.9 0.1 0.1 1</diffuse>
              <transparency>0.55</transparency>
            </material>
          </visual>
        </link>
      </model>
    </sdf>)";
    const std::string direction_sdf_xml = R"(<sdf version='1.6'>
      <model name='socnav_goal_direction'>
        <static>true</static>
        <link name='link'>
          <visual name='visual'>
            <pose>0.5 0 0.025 0 0 0</pose>
            <geometry><box><size>1.0 0.05 0.05</size></box></geometry>
            <material>
              <ambient>0.9 0.1 0.1 1</ambient>
              <diffuse>0.9 0.1 0.1 1</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>)";

    sdf::SDF area_sdf;
    sdf::SDF tolerance_sdf;
    sdf::SDF direction_sdf_data;
    area_sdf.SetFromString(area_sdf_xml);
    tolerance_sdf.SetFromString(tolerance_sdf_xml);
    direction_sdf_data.SetFromString(direction_sdf_xml);
    if (!area_sdf.Root() || !tolerance_sdf.Root() || !direction_sdf_data.Root()) {
      RCLCPP_ERROR(rosnode->get_logger(), "Could not parse goal marker SDF.");
      goal_models_requested = true;
      return;
    }
    world->InsertModelSDF(area_sdf);
    world->InsertModelSDF(tolerance_sdf);
    world->InsertModelSDF(direction_sdf_data);
    goal_models_requested = true;
  }

  if (!goal_area_model)
    goal_area_model = world->ModelByName("socnav_goal_area");
  if (!goal_tolerance_model)
    goal_tolerance_model = world->ModelByName("socnav_goal_tolerance");
  if (!goal_direction_model)
    goal_direction_model = world->ModelByName("socnav_goal_direction");
}

void SocNavPluginPrivate::ApplyGoal(const GoalState& goal)
{
  if (!goal.valid) return;
  EnsureGoalModels();

  constexpr double marker_thickness = 0.015;
  const double radius = std::max(0.05, goal.pos_threshold);
  const double footprint_radius = std::max(0.05, goal.robot_footprint_radius);
  const double outer_radius = std::max(footprint_radius + 0.1, radius + 0.05);
  if (goal_area_model) {
    goal_area_model->SetWorldPose(
      ignition::math::Pose3d(goal.x, goal.y, 0.01, 0, 0, 0));
    goal_area_model->SetScale(ignition::math::Vector3d(outer_radius, outer_radius, 1.0), true);
  }
  if (goal_tolerance_model) {
    goal_tolerance_model->SetWorldPose(
      ignition::math::Pose3d(goal.x, goal.y, marker_thickness + marker_thickness / 2.0, 0, 0, 0));
    goal_tolerance_model->SetScale(ignition::math::Vector3d(radius, radius, 1.0), true);
  }
  if (goal_direction_model) {
    goal_direction_model->SetWorldPose(
      ignition::math::Pose3d(goal.x, goal.y, 0.0, 0, 0, goal.angle));
  }
}

void SocNavPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  if (!is_loaded || trajectory_steps.empty()) return;

  double sim_time = _info.simTime.Double();

  // Start recording on the first frame if it hasn't started yet
  if (record_pub && !recording_finished && !start_ack_received &&
      sim_time >= 0.5 &&
      (last_start_signal_time < 0.0 ||
       sim_time - last_start_signal_time >= 0.5)) {

    // Get the path of the JSON provided in the SDF and change extension to .mp4
    std::filesystem::path json_p(json_path);
    std::filesystem::path video_p = json_p;
    video_p.replace_extension(".mp4"); // Change /path/file.json -> /path/file.mp4

    gazebo::msgs::GzString msg;
    msg.set_data(video_p.string()); // Send the video file path to the GUI plugin
    record_pub->Publish(msg);
    last_start_signal_time = sim_time;
    RCLCPP_INFO(rosnode->get_logger(), 
      "Recording video START signal sent on ~/record_video (t=%.2f s); waiting for GUI acknowledgment. Stored in: %s",
      sim_time, video_p.c_str());
  }

  // Compute dt since last tick (clamped to avoid a huge first-frame jump)
  double dt = (last_sim_time < 0.0) ? 0.0 : sim_time - last_sim_time;
  last_sim_time = sim_time;

  // Elapsed time relative to trajectory start
  double elapsed = sim_time;

  // Advance the step index while the *next* step's relative time has been reached.
  bool step_changed = false;
  while (current_step_idx + 1 < trajectory_steps.size()) {
    double next_rel_time = trajectory_steps[current_step_idx + 1].timestamp - start_timestamp;
    if (elapsed >= next_rel_time) {
      ++current_step_idx;
      step_changed = true;
    } else {
      break;
    }
  }

  if (step_changed && (current_step_idx + 1) % 20 == 0) {
    RCLCPP_INFO(rosnode->get_logger(),
      "Trajectory step %zu / %zu  (rel_t=%.3f s)",
      current_step_idx + 1, total_steps,
      trajectory_steps[current_step_idx].timestamp - start_timestamp);
  }

  // Always apply poses every physics tick (actors snap back without this).
  ApplyStep(trajectory_steps[current_step_idx], dt);
  ApplyGoal(trajectory_steps[current_step_idx].goal);


  // stop recording when the trajectory is completed
  if (current_step_idx >= trajectory_steps.size() - 1 && start_ack_received &&
      is_recording &&
      sim_time - recording_signal_time >= 0.5) {
    gazebo::msgs::GzString msg;
    msg.set_data("stop");
    record_pub->Publish(msg);
    is_recording = false;
    recording_finished = true;
    stop_signal_time = sim_time;
    RCLCPP_INFO(rosnode->get_logger(),
      "Recording video STOP signal sent on ~/record_video.");
  }

  // Give the GUI video encoder time to finalize the MP4 before shutting down.
  if (recording_finished && stop_signal_time >= 0.0 && stop_ack_received &&
      !shutdown_requested) {
    RCLCPP_INFO(rosnode->get_logger(),
      "Recording GUI confirmed MP4 finalization; scheduling Gazebo shutdown.");
    stop_signal_time = -1.0;
    shutdown_requested = true;

    // Do not shut down from the world-update callback. Gazebo waits for this
    // callback to return while shutting down. The SIGINT is sent only after
    // the GUI has acknowledged that SaveVideo() completed.
    std::thread([] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::raise(SIGINT);
    }).detach();
  }
}

void SocNavPluginPrivate::ApplyStep(const TrajectoryStep& step, double dt)
{
  // ---- Robot ----
  gazebo::physics::ModelPtr robot_model = world->ModelByName(robot_name);
  if (robot_model) {
    ignition::math::Pose3d pose;
    pose.Pos().X(step.robot.x);
    pose.Pos().Y(step.robot.y);
    pose.Pos().Z(0.0);
    pose.Rot() = ignition::math::Quaterniond(0, 0, step.robot.theta);
    robot_model->SetWorldPose(pose);
  }

  // ---- People ----
  for (const auto& person : step.people) {
    std::string model_name = "person_" + std::to_string(person.id);
    gazebo::physics::ModelPtr person_model = world->ModelByName(model_name);
    if (!person_model) continue;

    gazebo::physics::ActorPtr actor =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(person_model);

    auto &smoothed = smoothed_humans[person.id];
    double filtered_speed = 0.0;
    if (!smoothed.initialized || dt <= 0.0) {
      smoothed.x = person.x;
      smoothed.y = person.y;
      smoothed.theta = person.theta;
      smoothed.initialized = true;
    } else {
      const double alpha = 1.0 - std::exp(-dt / kHumanSmoothingTime);
      const double previous_x = smoothed.x;
      const double previous_y = smoothed.y;

      smoothed.x += alpha * (person.x - smoothed.x);
      smoothed.y += alpha * (person.y - smoothed.y);
      smoothed.theta = normalizeAngle(
        smoothed.theta + alpha * normalizeAngle(person.theta - smoothed.theta));

      filtered_speed = std::hypot(smoothed.x - previous_x, smoothed.y - previous_y) / dt;
    }

    ignition::math::Pose3d pose = person_model->WorldPose();
    pose.Pos().X(smoothed.x);
    pose.Pos().Y(smoothed.y);

    if (actor) {
      double yaw = normalizeAngle(smoothed.theta + M_PI_2);
      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, yaw);
      pose.Pos().Z(0.96); // keep actor feet on the ground
    } else {
      pose.Pos().Z(0.0);
      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, person.theta); //1.5707
    }

    person_model->SetWorldPose(pose);

    if (actor) {
      // Advance the animation script time proportionally to the agent's speed.
      // At kRefWalkSpeed the animation plays at 1× real speed.
      double speed_ratio = filtered_speed / kRefWalkSpeed;
      actor_script_time[person.id] += dt * speed_ratio;

      actor->SetScriptTime(actor_script_time[person.id]);
    }
  }
}

GZ_REGISTER_WORLD_PLUGIN(SocNavPlugin)

} // namespace socnav_gazebo_reproducer
