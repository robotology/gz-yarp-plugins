#pragma once

#include <gz/common/Event.hh>
#include <gz/common/events/Types.hh>
#include <gz/sim/Entity.hh>

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include <gz/sim/EntityComponentManager.hh>

namespace gzyarp
{

/**
 * Class instantiated by all gz-sim-yarp-plugins in the plugin Configure.
 *
 * The class destructor calls DeviceRegistry::getHandler()->incrementNrOfGzSimYARPPluginsNotSuccessfullyLoaded(ecm),
 * unless the setConfigureIsSuccessful(true) method is called to signal that the configure of the plugin has been successful.
 * This is used to keep track of the number of gz-sim-yarp-plugins that have not been successfully loaded.
 */
class PluginConfigureHelper {
public:
    PluginConfigureHelper(const gz::sim::EntityComponentManager& ecm) : m_configureSuccessful(false), m_pecm(&ecm) {}
    ~PluginConfigureHelper();
    void setConfigureIsSuccessful(bool success);

private:
    bool m_configureSuccessful;
    const gz::sim::EntityComponentManager* m_pecm;
};

/**
 * Class that manages the YARP devices created by the gz-sim-yarp-plugins.
 *
 * The class is a singleton and is used to store the devices created by the gz-sim-yarp-plugins.
 */
class DeviceRegistry
{
public:
    // Return a pointer to the only global instance of the DeviceRegistry in the process
    static DeviceRegistry* getHandler();

    /**
     * Extract all the devices from the registry that correspond to a given gz server
     * and that belong to a model and all its descendants
     */
    bool getDevicesAsPolyDriverList(const gz::sim::EntityComponentManager& ecm,
                                    const std::string& modelScopedName,
                                    yarp::dev::PolyDriverList& list,
                                    std::vector<std::string>& deviceScopedNames) const;
    /**
     * Insert a device in the registry.
     *
     * This method MUST be called only by the gz plugin that created the YARP device
     */
    bool setDevice(const gz::sim::Entity& entity,
                   const gz::sim::EntityComponentManager& ecm,
                   const std::string& yarpDeviceName,
                   yarp::dev::PolyDriver* device2add,
                   std::string& generatedDeviceDatabaseKey);

    /**
     * Get a device whose device ID corresponds exactly to the specified deviceDatabaseKey.
     */
    bool getDevice(const gz::sim::EntityComponentManager& ecm,
                   const std::string& deviceDatabaseKey,
                   yarp::dev::PolyDriver*& driver) const;

    /**
     * Remove a device whose device ID corresponds exactly to the specified deviceDatabaseKey.
     */
    bool
    removeDevice(const gz::sim::EntityComponentManager& ecm, const std::string& deviceDatabaseKey);

    /**
     * Add a callback when a device is removed, i.e. removeDevice is called.
     */
    template<typename T>
    gz::common::ConnectionPtr connectDeviceRemoved(T _subscriber)
    {
        return m_deviceRemovedEvent.Connect(_subscriber);
    }

    /**
     * Get all the device ids (aka device database keys) of the devices in the registry for a given gz server.
     */
    std::vector<std::string> getDevicesKeys(const gz::sim::EntityComponentManager& ecm) const;

    /**
     * Get number of gz-sim-yarp-plugins not successfully loaded for a specific simulation server.
     */
    std::size_t getNrOfGzSimYARPPluginsNotSuccessfullyLoaded(const gz::sim::EntityComponentManager& ecm) const;

    /**
     * Get number of gz-sim-yarp-plugins not successfully loaded for all simulation in the process.
     */
    std::size_t getTotalNrOfGzSimYARPPluginsNotSuccessfullyLoaded() const;

    /**
     * Increment number of gz-sim-yarp-plugins not successfully loaded. This function is only meant to be called by
     * gz-sim-yarp-plugins gz-sim plugins.
     */
    void incrementNrOfGzSimYARPPluginsNotSuccessfullyLoaded(const gz::sim::EntityComponentManager& ecm);

    /**
     * Add a configuration override for a given yarp device.
     *
     */
    bool addConfigurationOverrideForYARPDevice(const gz::sim::EntityComponentManager& ecm,
                                               const std::string& modelScopedNameWhereConfigurationOverrideWasInserted,
                                               const std::string& yarpDeviceName,
                                               std::unordered_map<std::string, std::string> overridenParameters);
    /**
     * Get the configuration override for a given yarp device.
     *
     */
    bool getConfigurationOverrideForYARPDevice(const gz::sim::EntityComponentManager& ecm,
                                               const std::string& modelScopedNameWhereYARPDeviceWasInserted,
                                               const std::string& yarpDeviceName,
                                               std::unordered_map<std::string, std::string>& overridenParameters) const;

private:
    /**
     * Generate a unique device id for a given yarp device name and entity.
     *
     * The device id is a process-unique string that identifies a device in a given gz server.
     * Inside the process, a device is uniquely identified by the pair of the gz instance id and the device id.
     */
    static std::string generateDeviceId(const gz::sim::Entity& entity,
                                        const gz::sim::EntityComponentManager& ecm,
                                        const std::string& yarpDeviceName);

    /**
     * Generate a unique instance id for a given gz server.
     */
    static std::string getGzInstanceId(const gz::sim::EntityComponentManager& ecm);

    /**
     * Extract the yarpDeviceName from a device id, i.e. the third argument passed to generateDeviceId.
     */
    static std::string getYarpDeviceName(const std::string& deviceId);

    /**
     * Extract the scoped model name from a device id, i.e. the scoped name of the parent model of the plugin that created the device.
     */
    static std::string getModelScopedName(const std::string& deviceId);

    // Private constructor. The constructor is private to ensure that the class is a singleton.
    DeviceRegistry();

    // Static instance of the DeviceRegistry
    static DeviceRegistry* s_handle;
    static std::mutex& mutex();

    // Map that stores all the yarp devices created by the gz-sim-yarp-plugins
    // The key of the first map is the gz instance id, the key of the second map is the device id
    std::unordered_map<std::string, std::unordered_map<std::string, yarp::dev::PolyDriver*>>
        m_devicesMap;

    // Event for when a device is removed
    gz::common::EventT<void (std::string)> m_deviceRemovedEvent;

    // Number of gz-sim-yarp-plugins YARP devices not loaded correctly for a given ecm
    std::unordered_map<std::string, std::size_t> m_nrOfGzSimYARPPluginsNotSuccessfullyLoaded;


    // Element that stores the parameters that will be overriden for a given yarp device
    // A device will be affected by the override if:
    // - gzInstanceId match
    // - yarpDeviceName match
    // - the modelScopedNameWhereConfigurationOverrideWasInserted is a prefix of the model scoped name of the device`
    struct ConfigurationOverrideParameters {
        // Id that identifies the gz instance where the configuration override plugin was inserted
        std::string gzInstanceId;
        // Scoped name of the model where the condiguration override plugin was inserted,
        // used to understand if a given yarp device is affected by the override
        std::string modelScopedNameWhereConfigurationOverrideWasInserted;
        // yarpDeviceName of the yarp device that will have its parameters overriden
        std::string yarpDeviceName;
        // Map of the parameters that will be overriden
        // Some keys have a special meaning:
        // gzyarp-xml-element-initialConfiguration: override the content of a <initialConfiguration> tag
        std::unordered_map<std::string, std::string> overridenParameters;
    };

    // This is a list of parameters specified by the configuration override plugin,
    // they specify the parameters that will be overriden for a given yarp device
    std::vector<ConfigurationOverrideParameters> m_yarpDevicesOverridenParametersList;
};

} // namespace gzyarp
