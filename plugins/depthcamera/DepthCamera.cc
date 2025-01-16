#include <DepthCameraDriver.cpp>
#include <DepthCameraShared.hh>
#include <ConfigurationHelpers.hh>
#include <DeviceRegistry.hh>

#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/details/image.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class DepthCamera : public System,
               public ISystemConfigure,
               public ISystemPreUpdate,
               public ISystemPostUpdate

{
public:
    DepthCamera()
        : m_deviceRegistered(false)
    {
    }

    virtual ~DepthCamera()
    {
        if (m_deviceRegistered)
        {
            DeviceRegistry::getHandler()->removeDevice(*ecm, m_deviceId);
            m_deviceRegistered = false;
        }

        if (m_cameraDriver.isValid())
        {
            m_cameraDriver.close();
        }
        yarp::os::Network::fini();
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {
        yarp::os::Network::init();

        gzyarp::PluginConfigureHelper configureHelper(_ecm);

        ecm = &_ecm;

        ::yarp::dev::Drivers::factory().add(
            new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::DepthCameraDriver>("gazebo_camera",
                                                                                "grabber",
                                                                                "DepthCameraDriver"));
        ::yarp::os::Property driver_properties;

        if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
        {
            if (!driver_properties.check("sensorName"))
            {
                yError() << "gz-sim-yarp-camera-system : missing sensorName parameter";
                return;
            }
            if (!driver_properties.check("parentLinkName"))
            {
                yError() << "gz-sim-yarp-camera-system : missing parentLinkName parameter";
                return;
            }
            if (!driver_properties.check("yarpDeviceName"))
            {
                yError() << "gz-sim-yarp-camera-system : missing yarpDeviceName parameter";
                return;
            }
            yInfo() << "gz-sim-yarp-camera-system: configuration of sensor "
                    << driver_properties.find("sensorName").asString() << " loaded";
        } else
        {
            yError() << "gz-sim-yarp-camera-system : missing configuration";
            return;
        }

        std::string sensorName = driver_properties.find("sensorName").asString();
        std::string parentLinkName = driver_properties.find("parentLinkName").asString();

        auto model = Model(_entity);
        auto parentLink = model.LinkByName(_ecm, parentLinkName);
        this->sensor = _ecm.EntityByComponents(components::ParentEntity(parentLink),
                                               components::Name(sensorName),
                                               components::Sensor());
        auto sdfSensor = _ecm.ComponentData<components::DepthCamera>(sensor).value().Element();
        auto sdfImage = sdfSensor.get()->GetElement("camera").get()->GetElement("image").get();

        cameraData.init(sdfImage->Get<int>("width"), sdfImage->Get<int>("height"), sensorScopedName);

        driver_properties.put(YarpDepthCameraScopedName.c_str(), sensorScopedName.c_str());

        driver_properties.put("device", "gazebo_camera");
        driver_properties.put("sensor_name", sensorName);

        // Open the driver
        if (!m_cameraDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-camera-system Plugin failed: error in opening yarp driver";
            return;
        }

        IDepthCameraData* iDepthCameraData = nullptr;
        auto viewOk = m_cameraDriver.view(iDepthCameraData);

        if (!viewOk || !iDepthCameraData)
        {
            yError() << "gz-sim-yarp-camera-system Plugin failed: error in getting "
                        "IDepthCameraData interface";
            return;
        }
        iDepthCameraData->setDepthCameraData(&cameraData);

        m_cameraDriver.view(iFrameGrabberImage);
        if (iFrameGrabberImage == NULL)
        {
            yError() << "Unable to get the iFrameGrabberImage interface from the device";
            return;
        }

        auto yarpDeviceName = driver_properties.find("yarpDeviceName").asString();

        if (!DeviceRegistry::getHandler()
                 ->setDevice(_entity, _ecm, yarpDeviceName, &m_cameraDriver, m_deviceId))
        {
            yError() << "gz-sim-yarp-camera-system: failed setting scopedDeviceName(=" << m_deviceId
                     << ")";
            return;
        }
        this->m_deviceRegistered = true;
        this->cameraInitialized = false;
        configureHelper.setConfigureIsSuccessful(true);
        yInfo() << "gz-sim-yarp-camera-system: Registered YARP device with instance name:"
                << m_deviceId;
    }

    virtual void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
        if (!this->cameraInitialized
            && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
        {
            this->cameraInitialized = true;
            auto DepthCameraTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
            this->node.Subscribe(DepthCameraTopicName, &DepthCamera::DepthCameraCb, this);
        }
    }

    virtual void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        gz::msgs::Image cameraMsg;
        {
            std::lock_guard<std::mutex> lock(this->cameraMsgMutex);
            cameraMsg = this->cameraMsg;
        }

        if (this->cameraInitialized)
        {
            std::lock_guard<std::mutex> lock(cameraData.m_mutex);
            memcpy(cameraData.m_imageBuffer, cameraMsg.data().c_str(), cameraData.m_bufferSize);
            cameraData.simTime = _info.simTime.count() / 1e9;
        }
    }

    void DepthCameraCb(const gz::msgs::Image& _msg)
    {
        std::lock_guard<std::mutex> lock(this->cameraMsgMutex);
        cameraMsg = _msg;
    }

private:
    Entity sensor;
    yarp::dev::PolyDriver m_cameraDriver;
    std::string m_deviceId;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    DepthCameraData cameraData;
    bool cameraInitialized;
    gz::transport::Node node;
    gz::msgs::Image cameraMsg;
    std::mutex cameraMsgMutex;
    yarp::dev::IFrameGrabberImage* iFrameGrabberImage;
    EntityComponentManager* ecm;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::DepthCamera,
              gz::sim::System,
              gzyarp::DepthCamera::ISystemConfigure,
              gzyarp::DepthCamera::ISystemPreUpdate,
              gzyarp::DepthCamera::ISystemPostUpdate)
