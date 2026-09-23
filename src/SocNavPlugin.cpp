#include <socnav_gazebo_reproducer/SocNavPlugin.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <cmath>

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
  
  void OnUpdate(const gazebo::common::UpdateInfo& _info);
  void ApplyStep(const TrajectoryStep& step, double dt);
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

  std::string json_path = "";
  if (_sdf->HasElement("trajectory_file")) {
    json_path = _sdf->Get<std::string>("trajectory_file");
  } else {
    RCLCPP_ERROR(impl_->rosnode->get_logger(), "No <trajectory_file> provided!");
    return;
  }

  std::ifstream f(json_path);
  if (!f.is_open()) {
    RCLCPP_ERROR(impl_->rosnode->get_logger(), "Could not open file: %s", json_path.c_str());
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
}

void SocNavPlugin::Reset()
{
  impl_->current_step_idx = 0;
  impl_->last_sim_time = -1.0;
  impl_->actor_script_time.clear();
  impl_->smoothed_humans.clear();
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

void SocNavPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
  if (!is_loaded || trajectory_steps.empty()) return;

  double sim_time = _info.simTime.Double();

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

  if (step_changed && (current_step_idx + 1) % 10 == 0) {
    RCLCPP_INFO(rosnode->get_logger(),
      "Trajectory step %zu / %zu  (rel_t=%.3f s)",
      current_step_idx + 1, total_steps,
      trajectory_steps[current_step_idx].timestamp - start_timestamp);
  }

  // Always apply poses every physics tick (actors snap back without this).
  ApplyStep(trajectory_steps[current_step_idx], dt);
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
