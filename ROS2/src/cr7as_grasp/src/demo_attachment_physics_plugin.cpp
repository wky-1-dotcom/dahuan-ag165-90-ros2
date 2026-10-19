#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace gazebo
{
// Demo-only model plugin. The follower sets the object pose kinematically;
// this plugin changes Gazebo physics only on the world-update thread so ODE
// cannot fight the follower with gravity/contact impulses while attached.
class DemoAttachmentPhysicsPlugin final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    if (!model_) {
      gzerr << "demo_attachment_physics_plugin: model is null\n";
      return;
    }

    const std::string link_name = sdf->HasElement("link_name")
      ? sdf->Get<std::string>("link_name") : "red_box_link";
    service_name_ = sdf->HasElement("enable_service")
      ? sdf->Get<std::string>("enable_service")
      : "/gazebo_grasp/set_demo_attachment_physics";
    link_ = model_->GetLink(link_name);
    if (!link_) {
      gzerr << "demo_attachment_physics_plugin: link not found: " << link_name << "\n";
      return;
    }


    node_ = std::make_shared<rclcpp::Node>(model_->GetName() + "_demo_attachment_physics");
    service_ = node_->create_service<std_srvs::srv::SetBool>(
      service_name_,
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        // Gazebo links are not touched on this ROS executor thread. The world
        // update callback applies the requested state on the physics thread.
        requested_demo_hold_.store(request->data);
        physics_update_pending_.store(true);
        response->success = true;
        response->message = request->data
          ? "Demo attachment physics enable queued."
          : "Demo attachment physics restore queued.";
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    ros_thread_ = std::thread([this]() { executor_->spin(); });
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DemoAttachmentPhysicsPlugin::OnWorldUpdate, this));

    RCLCPP_INFO(
      node_->get_logger(),
      "Demo attachment physics plugin ready: link=%s, service=%s.",
      link_name.c_str(), service_name_.c_str());
  }

  ~DemoAttachmentPhysicsPlugin() override
  {
    if (executor_) executor_->cancel();
    if (ros_thread_.joinable()) ros_thread_.join();
  }

private:
  void OnWorldUpdate()
  {
    if (!physics_update_pending_.exchange(false) || !link_) {
      return;
    }

    const bool enable_kinematic_demo_hold = requested_demo_hold_.load();
    if (enable_kinematic_demo_hold) {
      link_->SetGravityMode(false);
      for (const auto & collision : link_->GetCollisions()) {
        if (collision) collision->SetCollideBits(0u);
      }
      RCLCPP_INFO(node_->get_logger(),
        "Demo attachment active: object gravity/collision disabled while follower owns pose.");
    } else {
      link_->SetGravityMode(true);
      for (const auto & collision : link_->GetCollisions()) {
        if (!collision) continue;
        // The red-box SDF uses Gazebo Classic default collide bits (all bits).`n        collision->SetCollideBits(0xFFFFu);`n
      }
      RCLCPP_INFO(node_->get_logger(),
        "Demo attachment released: object gravity/collision restored.");
    }
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  event::ConnectionPtr update_connection_;
  std::thread ros_thread_;
  std::unordered_map<std::string, unsigned int> original_collide_bits_;
  std::atomic_bool requested_demo_hold_{false};
  std::atomic_bool physics_update_pending_{false};
  std::string service_name_;
};

GZ_REGISTER_MODEL_PLUGIN(DemoAttachmentPhysicsPlugin)
}  // namespace gazebo