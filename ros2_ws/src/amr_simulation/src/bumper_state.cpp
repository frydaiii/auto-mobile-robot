// Publish contact snapshots, including clear states that Gazebo Contact omits.
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/transport/Node.hh>

namespace amr_simulation
{
class BumperState : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    model = gz::sim::Model(entity);
    const double rate = sdf->Get<double>("rate", 50.0).first;
    if (!std::isfinite(rate) || rate <= 0)
      throw std::invalid_argument("Bumper publish rate must be positive and finite");
    period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
    for (size_t i = 0; i < sides.size(); ++i)
      publishers[i] = transport.Advertise<gz::msgs::Contacts>("/sim/bumper/" + sides[i]);
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused)
      return;
    if (last && info.simTime < *last)
    {
      last.reset();
      for (auto &message : pending)
        message.Clear();
    }
    // Do not invent clear feedback when a required physics component is missing.
    // Re-resolve entities each step so reset/recreation does not leave stale IDs.
    for (size_t i = 0; i < sides.size(); ++i)
    {
      const auto link = model.LinkByName(ecm, "bumper_" + sides[i] + "_link");
      const auto collisions = ecm.ChildrenByComponents(link, gz::sim::components::Collision());
      if (collisions.size() != 1)
        return;
      const auto *contacts = ecm.Component<gz::sim::components::ContactSensorData>(collisions[0]);
      if (!contacts)
        return;
      // Latch brief impacts until the next snapshot, rather than sampling them away.
      if (contacts->Data().contact_size() > 0)
        pending[i].CopyFrom(contacts->Data());
    }
    if (last && info.simTime - *last < period)
      return;
    last = info.simTime;
    for (size_t i = 0; i < sides.size(); ++i)
    {
      *pending[i].mutable_header()->mutable_stamp() = gz::sim::convert<gz::msgs::Time>(info.simTime);
      publishers[i].Publish(pending[i]);
      pending[i].Clear();
    }
  }

private:
  gz::sim::Model model;
  gz::transport::Node transport;
  const std::array<std::string, 3> sides{{"left", "center", "right"}};
  std::array<gz::transport::Node::Publisher, 3> publishers;
  std::array<gz::msgs::Contacts, 3> pending;
  std::chrono::steady_clock::duration period{};
  std::optional<std::chrono::steady_clock::duration> last;
};
}  // namespace amr_simulation

GZ_ADD_PLUGIN(amr_simulation::BumperState, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
