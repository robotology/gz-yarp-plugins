#include <ConfigurationHelpers.hh>
#include <ImuDriver.cpp>

#include <gz/msgs/details/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace gzyarp
{

class Imu : public System,
            public ISystemConfigure,
            public ISystemPreUpdate,
            public ISystemPostUpdate

{
public:
    Imu()
        : m_deviceRegistered(false)
    {
    }

    virtual ~Imu()
    {
        if (m_deviceRegistered)
        {
            Handler::getHandler()->removeDevice(m_deviceScopedName);
            m_deviceRegistered = false;
        }

        if (m_imuDriver.isValid())
            m_imuDriver.close();
        ImuDataSingleton::getHandler()->removeSensor(sensorScopedName);
    }

    virtual void Configure(const Entity& _entity,
                           const std::shared_ptr<const sdf::Element>& _sdf,
                           EntityComponentManager& _ecm,
                           EventManager& /*_eventMgr*/) override
    {

        std::string netWrapper = "inertial";
        ::yarp::dev::Drivers::factory().add(
            new ::yarp::dev::DriverCreatorOf<::yarp::dev::gzyarp::ImuDriver>("gazebo_imu",
                                                                             netWrapper.c_str(),
                                                                             "IMUDriver"));

        ::yarp::os::Property driver_properties;

        if (ConfigurationHelpers::loadPluginConfiguration(_sdf, driver_properties))
        {
            if (!driver_properties.check("sensorName"))
            {
                yError() << "gz-sim-yarp-imu-system : missing sensorName parameter";
                return;
            }
            if (!driver_properties.check("parentLinkName"))
            {
                yError() << "gz-sim-yarp-imu-system : missing parentLinkName parameter";
                return;
            }
            yInfo() << "gz-sim-yarp-imu-system: configuration of sensor "
                    << driver_properties.find("sensorName").asString() << " loaded";
        } else
        {
            yError() << "gz-sim-yarp-imu-system : missing configuration";
            return;
        }

        std::string sensorName = driver_properties.find("sensorName").asString();
        std::string parentLinkName = driver_properties.find("parentLinkName").asString();
        auto model = Model(_entity);
        auto parentLink = model.LinkByName(_ecm, parentLinkName);
        this->sensor = _ecm.EntityByComponents(components::ParentEntity(parentLink),
                                               components::Name(sensorName),
                                               components::Sensor());

        sensorScopedName = scopedName(this->sensor, _ecm);
        this->imuData.sensorScopedName = sensorScopedName;

        driver_properties.put(YarpIMUScopedName.c_str(), sensorScopedName.c_str());
        if (!driver_properties.check("yarpDeviceName"))
        {
            yError() << "gz-sim-yarp-imu-system : missing yarpDeviceName parameter for device"
                     << sensorScopedName;
            return;
        }

        // Insert the pointer in the singleton handler for retriving it in the yarp driver
        ImuDataSingleton::getHandler()->setSensor(&(this->imuData));

        driver_properties.put("device", "gazebo_imu");
        driver_properties.put("sensor_name", sensorName);
        if (!m_imuDriver.open(driver_properties))
        {
            yError() << "gz-sim-yarp-imu-system Plugin failed: error in opening yarp driver";
            return;
        }

        m_deviceScopedName
            = sensorScopedName + "/" + driver_properties.find("yarpDeviceName").asString();

        if (!Handler::getHandler()->setDevice(m_deviceScopedName, &m_imuDriver))
        {
            yError() << "gz-sim-yarp-imu-system: failed setting scopedDeviceName(="
                     << m_deviceScopedName << ")";
            return;
        }
        m_deviceRegistered = true;
        yInfo() << "Registered YARP device with instance name:" << m_deviceScopedName;
    }
    virtual void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override
    {
        if (!this->imuInitialized
            && _ecm.ComponentData<components::SensorTopic>(sensor).has_value())
        {
            this->imuInitialized = true;
            auto imuTopicName = _ecm.ComponentData<components::SensorTopic>(sensor).value();
            this->node.Subscribe(imuTopicName, &Imu::imuCb, this);
        }
    }

    virtual void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override
    {
        gz::msgs::IMU imuMsg;
        {
            std::lock_guard<std::mutex> lock(this->imuMsgMutex);
            imuMsg = this->imuMsg;
        }
        std::lock_guard<std::mutex> lock(imuData.m_mutex);
        imuData.m_data[0] = (imuMsg.orientation().x() != 0) ? imuMsg.orientation().x() : 0;
        imuData.m_data[1] = (imuMsg.orientation().y() != 0) ? imuMsg.orientation().y() : 0;
        imuData.m_data[2] = (imuMsg.orientation().w() != 0) ? imuMsg.orientation().w() : 0;
        imuData.m_data[3]
            = (imuMsg.linear_acceleration().x() != 0) ? imuMsg.linear_acceleration().x() : 0;
        imuData.m_data[4]
            = (imuMsg.linear_acceleration().y() != 0) ? imuMsg.linear_acceleration().y() : 0;
        imuData.m_data[5]
            = (imuMsg.linear_acceleration().z() != 0) ? imuMsg.linear_acceleration().z() : 0;
        imuData.m_data[6] = (imuMsg.angular_velocity().x() != 0) ? imuMsg.angular_velocity().x()
                                                                 : 0;
        imuData.m_data[7] = (imuMsg.angular_velocity().y() != 0) ? imuMsg.angular_velocity().y()
                                                                 : 0;
        imuData.m_data[8] = (imuMsg.angular_velocity().z() != 0) ? imuMsg.angular_velocity().z()
                                                                 : 0;
        imuData.simTime = _info.simTime.count() / 1e9;
    }

    void imuCb(const gz::msgs::IMU& _msg)
    {
        std::lock_guard<std::mutex> lock(this->imuMsgMutex);
        imuMsg = _msg;
    }

private:
    Entity sensor;
    yarp::dev::PolyDriver m_imuDriver;
    std::string m_deviceScopedName;
    std::string sensorScopedName;
    bool m_deviceRegistered;
    ImuData imuData;
    bool imuInitialized;
    gz::transport::Node node;
    gz::msgs::IMU imuMsg;
    std::mutex imuMsgMutex;
    yarp::os::Network m_yarpNetwork;
};

} // namespace gzyarp

// Register plugin
GZ_ADD_PLUGIN(gzyarp::Imu,
              gz::sim::System,
              gzyarp::Imu::ISystemConfigure,
              gzyarp::Imu::ISystemPreUpdate,
              gzyarp::Imu::ISystemPostUpdate)
