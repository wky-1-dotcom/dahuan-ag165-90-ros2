#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gazebo
{
class Ag165GripperMimicPlugin final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = std::make_shared<rclcpp::Node>(model_->GetName() + "_mimic");
    if (!sdf->HasElement("kp") || !sdf->HasElement("kd") || !sdf->HasElement("effort")) {
      RCLCPP_WARN(node_->get_logger(),
        "AG165 mimic is missing kp, kd or effort; using conservative defaults for missing values");
    }
    if (sdf->HasElement("kp")) kp_ = sdf->Get<double>("kp");
    if (sdf->HasElement("kd")) kd_ = sdf->Get<double>("kd");
    if (sdf->HasElement("effort")) effort_ = sdf->Get<double>("effort");
    enabled_ = !sdf->HasElement("enabled") || sdf->Get<bool>("enabled");

    const std::string active_name = sdf->HasElement("active_joint")
      ? sdf->Get<std::string>("active_joint") : "joint1_left";
    active_name_ = active_name;
    active_ = model_->GetJoint(active_name);
    if (active_) active_position_ = active_->Position(0);
    if (!active_) gzerr << "ag165_gripper_mimic_plugin: active joint not found: " << active_name << "\n";

    if (sdf->HasElement("follower")) {
      auto item = sdf->GetElement("follower");
      while (item) {
        const auto name = item->Get<std::string>("name");
        ++requested_followers_;
        auto joint = model_->GetJoint(name);
        if (joint && active_) {
          followers_.push_back(joint);
          follower_names_.push_back(name);
          const double multiplier = item->HasElement("multiplier")
            ? item->Get<double>("multiplier") : 1.0;
          const double offset = item->HasElement("offset")
            ? item->Get<double>("offset")
            : joint->Position(0) - multiplier * active_->Position(0);
          multipliers_.push_back(multiplier);
          offsets_.push_back(offset);
          follower_targets_.push_back(offset + multiplier * active_->Position(0));
          follower_positions_.push_back(joint->Position(0));
          follower_forces_.push_back(0.0);
        } else {
          gzerr << "ag165_gripper_mimic_plugin: follower joint not found: " << name << "\n";
        }
        item = item->GetNextElement("follower");
      }
    }
    const bool ready = active_ && requested_followers_ > 0 &&
      followers_.size() == requested_followers_;
    if (!ready) {
      enabled_ = false;
      RCLCPP_ERROR(node_->get_logger(),
        "AG165 mimic disabled: active joint or requested follower joint was not resolved");
    }

    RCLCPP_INFO(node_->get_logger(),
      "AG165 mimic loaded: active=%s found=%s, followers=%zu/%zu, enabled=%s, kp=%.3f kd=%.3f effort=%.3f",
      active_name_.c_str(), active_ ? "true" : "false", followers_.size(), requested_followers_,
      enabled_ ? "true" : "false", kp_, kd_, effort_);
    for (size_t i = 0; i < follower_names_.size(); ++i) {
      RCLCPP_INFO(node_->get_logger(), "AG165 mimic follower=%s multiplier=%.3f offset=%.6f",
        follower_names_[i].c_str(), multipliers_[i], offsets_[i]);
    }

    service_ = node_->create_service<std_srvs::srv::SetBool>(
      "/" + model_->GetName() + "/mimic_enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool ready = active_ && requested_followers_ > 0 &&
          followers_.size() == requested_followers_;
        enabled_ = request->data && ready;
        response->success = !request->data || ready;
        response->message = !request->data
          ? "mimic control disabled"
          : (ready ? "mimic control enabled"
                   : "mimic cannot enable: active or follower joint was not resolved; call mimic_status");
      });
    status_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/" + model_->GetName() + "/mimic_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->success = active_ && requested_followers_ > 0 &&
          followers_.size() == requested_followers_;
        std::ostringstream out;
        out << "active=" << active_name_ << " found=" << (active_ ? "true" : "false")
            << " enabled=" << (enabled_ ? "true" : "false")
            << " followers=" << followers_.size() << "/" << requested_followers_
            << " kp=" << kp_ << " kd=" << kd_ << " effort_limit=" << effort_;
        if (active_) out << " active_position=" << active_position_;
        for (size_t i = 0; i < followers_.size(); ++i) {
          out << "; " << follower_names_[i] << " position=" << follower_positions_[i]
              << " multiplier=" << multipliers_[i] << " offset=" << offsets_[i]
              << " target=" << follower_targets_[i] << " force=" << follower_forces_[i];
        }
        response->message = out.str();
      });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    ros_thread_ = std::thread([this]() { executor_->spin(); });
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&Ag165GripperMimicPlugin::OnUpdate, this));
  }

  void OnUpdate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return;
    const double active_position = active_->Position(0);
    if (!std::isfinite(active_position)) return;
    active_position_ = active_position;
    for (size_t i = 0; i < followers_.size(); ++i) {
      const double target = offsets_[i] + multipliers_[i] * active_position;
      follower_targets_[i] = target;
      const double position = followers_[i]->Position(0);
      follower_positions_[i] = position;
      const double velocity = followers_[i]->GetVelocity(0);
      if (!enabled_) {
        follower_forces_[i] = 0.0;
        continue;
      }
      if (!std::isfinite(target) || !std::isfinite(position) || !std::isfinite(velocity)) {
        followers_[i]->SetForce(0, 0.0);
        follower_forces_[i] = 0.0;
        continue;
      }
      const double raw = kp_ * (target - position) - kd_ * velocity;
      const double force = std::max(-effort_, std::min(effort_, raw));
      followers_[i]->SetForce(0, force);
      follower_forces_[i] = force;
    }
  }

  ~Ag165GripperMimicPlugin() override
  {
    if (executor_) executor_->cancel();
    if (ros_thread_.joinable()) ros_thread_.join();
  }

private:
  physics::ModelPtr model_;
  physics::JointPtr active_;
  std::vector<physics::JointPtr> followers_;
  std::string active_name_;
  std::vector<std::string> follower_names_;
  std::vector<double> multipliers_;
  std::vector<double> offsets_;
  std::vector<double> follower_targets_;
  std::vector<double> follower_positions_;
  std::vector<double> follower_forces_;
  double active_position_{0.0};
  size_t requested_followers_{0};
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
  event::ConnectionPtr update_connection_;
  std::thread ros_thread_;
  std::mutex mutex_;
  bool enabled_{true};
  double kp_{1.5};
  double kd_{0.3};
  double effort_{1.5};
};

GZ_REGISTER_MODEL_PLUGIN(Ag165GripperMimicPlugin)
}  // namespace gazebo
