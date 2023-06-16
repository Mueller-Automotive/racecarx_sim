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
class RoadGen
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
    // This callback is called only once at simulation startup
    virtual void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/) override
    {

        // Spawn a bunch of sample segments in the world since
        // actual randomized world generation is not implemented yet
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

    // This callback is called every simulation step after physics calculations are done
    virtual void PostUpdate(const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm) override
    {
        // Do something!
        // Probably calculate ground truth data here
    }

    private: transport::Node node;
};

// Register the plugin
GZ_ADD_PLUGIN(RoadGen,
                gz::sim::System,
                RoadGen::ISystemConfigure,
                RoadGen::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(RoadGen, "gz::sim::systems::RoadGen")