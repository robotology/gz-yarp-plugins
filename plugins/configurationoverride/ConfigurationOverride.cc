#include <DeviceRegistry.hh>

#include <chrono>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

#include <thread>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

using namespace gz;
using namespace sim;
using namespace systems;

using yarp::os::Bottle;
using yarp::os::BufferedPort;

namespace gzyarp
{

class ConfigurationOverride : public System, public ISystemConfigure, public ISystemPostUpdate, public ISystemReset
{
public:
    ConfigurationOverride(): m_initialized(false)
    {
    }

    ~ConfigurationOverride()
    {
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        if (!m_initialized)
        {
            // We need to clone the sdf element to call non-const methods
            auto sdfClone = _sdf->Clone();

            gzyarp::PluginConfigureHelper configureHelper(_ecm);

            std::unordered_map<std::string, std::string> overridenParameters;

            if (!sdfClone->HasElement("yarpPluginConfigurationOverride"))
            {
                yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                            "missing yarpPluginConfigurationOverride element";
                return;
            }

            sdf::ElementPtr yarpPluginConfigurationOverrideElem = sdfClone->GetElement("yarpPluginConfigurationOverride");
            if (!yarpPluginConfigurationOverrideElem->HasAttribute("yarpDeviceName"))
            {
                yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                            "missing yarpDeviceName attribute of yarpPluginConfigurationOverride element";
                return;
            }

            std::string yarpDeviceName = yarpPluginConfigurationOverrideElem->GetAttribute("yarpDeviceName")->GetAsString();

            if (sdfClone->HasElement("initialConfiguration"))
            {
                overridenParameters["gzyarp-xml-element-initialConfiguration"] = sdfClone->Get<std::string>("initialConfiguration");
            }

            if (!DeviceRegistry::getHandler()->addConfigurationOverrideForYARPDevice(
                    _ecm,
                    scopedName(_entity, _ecm, "/"),
                    yarpDeviceName,
                    overridenParameters))
            {
                yError() << "Error in gzyarp::ConfigurationOverride::Configure: "
                            "addConfigurationOverrideForYARPDevice failed";
                return;
            }

            m_initialized = true;
            configureHelper.setConfigureIsSuccessful(true);
        }
    }

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
    }

    void Reset(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
    }

private:
    bool m_initialized;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::ConfigurationOverride,
              gz::sim::System,
              gzyarp::ConfigurationOverride::ISystemConfigure,
              gzyarp::ConfigurationOverride::ISystemPostUpdate,
              gzyarp::ConfigurationOverride::ISystemReset)
