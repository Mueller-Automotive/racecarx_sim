#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector4.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/transport/Node.hh>

#include <plugins/road_gen/DashedLine.hpp>

using namespace gz;
using namespace sim;
using namespace systems;

// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
    // Implement Configure callback, provided by ISystemConfigure
    // and called once at startup.
    virtual void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/) override
    {
        // Read property from SDF
        auto linkName = _sdf->Get<std::string>("plane2");

        // Create model object to access convenient functions
        auto model = Model(_entity);

        // Get link entity
        this->linkEntity = _entity;

        DashedLine line("sdf-build-example", 
                gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0), 
                gz::math::Vector4<float>(10.0, 0.0, 0.0, 1.0), 
                gz::math::Vector4<float>(0.0, 0.0, 0.01, 1.0),
                gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0));

        bool result;
        gz::msgs::EntityFactory req;
        gz::msgs::Boolean res;
        req.set_sdf(line.getSdf());
        req.set_allow_renaming(true);

        unsigned int timeout = 5000;

        bool executed = this->node.Request("/world/racecarx-depot/create", req, timeout, res, result);
        if (executed)
        {
            if (result)
            std::cout << "Entity was created : [" << res.data() << "]" << std::endl;
            else
            {
            std::cout << "Service call failed" << std::endl;
            return;
            }
        }
        else
            std::cerr << "Service call timed out" << std::endl;

    }

    // Implement PostUpdate callback, provided by ISystemPostUpdate
    // and called at every iteration, after physics is done
    virtual void PostUpdate(const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm) override
    {
        // Get link pose and print it
        //std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
    }

    // ID of link entity
    private: Entity linkEntity;
    private: transport::Node node;
};

// Register plugin
GZ_ADD_PLUGIN(MyPlugin,
                gz::sim::System,
                MyPlugin::ISystemConfigure,
                MyPlugin::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(MyPlugin, "gz::sim::systems::MyPlugin")