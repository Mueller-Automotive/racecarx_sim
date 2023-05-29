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

        std::vector<gz::math::Vector4<float>> points = getBezierPoints(
            gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0), 
            gz::math::Vector4<float>(13.0, 13.0, 0.0, 1.0), 
            90,
            5);

        for (int i = 0; i < points.size(); i++)
        {
            std::cout << points[i] << std::endl;
        }

        DashedLine dashed_line("dashed-line-1", 
                gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0), 
                gz::math::Vector4<float>(-10.0, -10.0, 0.0, 1.0), 
                gz::math::Vector4<float>(0.0, 0.0, 0.01, 1.0),
                gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0),
                90);
        
        Line curved_line("curved-line-1", 
                gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0),
                gz::math::Vector4<float>(10.0, 10.0, 0.0, 1.0),
                gz::math::Vector4<float>(0.0, 3.0, 0.01, 1.0),
                gz::math::Vector4<float>(0.0, 0.0, 0.0, 1.0),
                90);

        std::vector<Line*> lines;
        lines.push_back(&dashed_line);
        lines.push_back(&curved_line);

        bool result;
        gz::msgs::EntityFactory req;
        gz::msgs::Boolean res;
        req.set_allow_renaming(true);

        unsigned int timeout = 5000;
        
        for (int i = 0; i < lines.size(); i++)
        {
            req.set_sdf(lines[i]->getSdf());
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
    }

    // Implement PostUpdate callback, provided by ISystemPostUpdate
    // and called at every iteration, after physics is done
    virtual void PostUpdate(const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm) override
    {
        // Get link pose and print it
        //std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
    }

    // Create a discrete polyline between two points at a given angle
    // by creating a quadratic Bezier curve
    private: std::vector<gz::math::Vector4<float>> getBezierPoints(gz::math::Vector4<float> p1, gz::math::Vector4<float> p2, float angle, int segments)
    {
        std::vector<gz::math::Vector4<float>> points;

        // We need to create a third control point to "curve" our bezier curve
        // The third control point is the apex of an isosceles triangle, where p1
        // is one base point and p2 is the other

        float b = p1.Distance(p2); // base
        float a = b / (2 * std::sin(degreesToRadians( (180-std::abs(angle))/2 ))); // equal sides

        std::cout << "a: " << a << std::endl;   
        std::cout << "b: " << b << std::endl;
        
        // Calculate the height of the triangle
        float h = std::sqrt(std::pow(a, 2) - (std::pow(b, 2) / 4));

        // Multiply it by the sign of the rotation angle (to make the line curve left or right)
        h *= angle / std::abs(angle);

        std::cout << "h: " << h << std::endl;

        gz::math::Vector4<float> midpoint = (p1 + p2) / 2;
        gz::math::Vector4<float> u = (p2 - midpoint).Normalized();
        gz::math::Vector4<float> controlPoint;
        controlPoint[0] = -u[1];
        controlPoint[1] = u[0];

        controlPoint = midpoint + controlPoint*h;
        // Control point is now ready

        std::cout << "u: " << u << std::endl;
        std::cout << "midpoint: " << midpoint << std::endl;
        std::cout << "control point: " << controlPoint << std::endl;

        for (int i = 0; i < segments + 1; i++)
        {
            float perc = (float)i / segments;
            float xa = getPt(p1[0], controlPoint[0], perc);
            float ya = getPt(p1[1], controlPoint[1], perc);
            float xb = getPt(controlPoint[0], p2[0], perc);
            float yb = getPt(controlPoint[1], p2[1], perc);
            
            float x = getPt(xa, xb, perc);
            float y = getPt(ya, yb, perc);

            std::cout << "X" << i << " :" << x << std::endl;
            std::cout << "Y" << i << " :" << x << std::endl;
            std::cout << "Perc: " << perc << std::endl;

            points.push_back(gz::math::Vector4<float>(x, y, p1[2], 1.0));
        }

        return points;
    }

    private: float getPt(float n1, float n2, float perc)
    {
        float diff = n2 - n1;

        return n1 + (diff * perc);
    }

    private: float degreesToRadians(float degrees) {
        return degrees * M_PI / 180.0;
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