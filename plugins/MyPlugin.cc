#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/transport/Node.hh>

#include <plugins/road_gen/DashedLine.hpp>
#include <plugins/road_gen/StraightSegment.hpp>
#include <plugins/road_gen/LeftTurnSegment.hpp>
#include <plugins/road_gen/RightTurnSegment.hpp>
#include <plugins/road_gen/IntersectionSegment.hpp>
#include <plugins/road_gen/ParkingSegment.hpp>
#include <plugins/road_gen/Utils.hpp>

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

        std::vector<RoadSegment*> segments;
        segments.push_back(new StraightSegment(gz::math::Vector3<float>(0.0, 0.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 0.0)));

        segments.push_back(new IntersectionSegment(gz::math::Vector3<float>(10.0, 0.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 0.0)));

        segments.push_back(new StraightSegment(gz::math::Vector3<float>(19.0, 9.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 90.0)));

        segments.push_back(new StraightSegment(gz::math::Vector3<float>(15.0, -5.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 270.0)));

        segments.push_back(new LeftTurnSegment(gz::math::Vector3<float>(19.0, 19.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 90.0)));

        segments.push_back(new StraightSegment(gz::math::Vector3<float>(0.0, 24.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 0.0)));

        segments.push_back(new LeftTurnSegment(gz::math::Vector3<float>(0.0, 28.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 180.0)));

        segments.push_back(new StraightSegment(gz::math::Vector3<float>(-5.0, 9.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 90.0)));

        segments.push_back(new LeftTurnSegment(gz::math::Vector3<float>(-9.0, 9.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, -90.0)));


        segments.push_back(new ParkingSegment(gz::math::Vector3<float>(0.0, 0.0, 0.0),
                                        gz::math::Vector3<float>(0.0, 0.0, 0.0)));
        
        bool result;
        gz::msgs::EntityFactory req;
        gz::msgs::Boolean res;
        req.set_allow_renaming(true);

        unsigned int timeout = 5000;

        std::vector<std::string> models;

        for (int i = 0; i < segments.size(); i++)
        {
                std::vector<std::string> segmentModels = segments[i]->getModelsSdf();
                models.insert(models.end(), segmentModels.begin(), segmentModels.end());
        }
              

        for (int i = 0; i < models.size(); i++)
        {
            //req.set_sdf(lines[i]->getSdf());
            req.set_sdf(models[i]);

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
            {
                std::cerr << "Service call timed out" << std::endl;
            }
            
        }
    }

    // Implement PostUpdate callback, provided by ISystemPostUpdate
    // and called at every iteration, after physics is done
    virtual void PostUpdate(const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm) override
    {
        // Get link pose and print it
        //std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
    }

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