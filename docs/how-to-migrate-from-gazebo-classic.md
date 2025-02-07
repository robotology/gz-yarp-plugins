# How to migrate from Gazebo Classic and `gazebo-yarp-plugins`

## Table of Contents

- [Introduction](#introduction)
- [Update of the robot model](#update-of-the-robot-model)
- [Update of plugins configurations](#update-of-plugins-configurations)
- [World set up](#world-set-up)
- [Environment variables](#environment-variables)
- [Examples of projects with Modern Gazebo support](#examples-of-projects-with-modern-gazebo-support)

## Introduction

This brief guide will help you migrate an existing project created initially for Gazebo Classic simulator and `gazebo-yarp-plugins` to Modern Gazebo (`gz-sim`) and `gz-sim-yarp-plugins`.

It is (usually) a quick and hassle-free process. In case you find errors or you need a feature missing from `gazebo-yarp-plugins` feel free to [open an issue](https://github.com/robotology/gz-sim-yarp-plugins/issues).

The migration process usually entails the following main stages:

1. Update of the robot model;
1. Update of the plugin configuration;
1. World set up;
1. Environment variables;

The next sections will address these steps in detail.

## Update of the robot model

Depending on the migration requirements and the model complexity, there are two ways to proceed with the migration: 

- create a new robot model from scratch;
- update the existing one used with Gazebo Classic.

Both approaches work fine, in the latter case you end up with a single model that could be used with both simulators at the expense of having some harmless error messages at the startup of the simulation since some of the plugins will not be found.

### gz-sim-yarp-plugins

In the table below are listed the `gz-sim-yarp-plugins` currently available: for each of them the `gazebo-yarp-plugins` `filename` attribute is present along with the new `name` and `filename` attributes that have to be used to make Modern Gazebo load them.

| Plugin          | Gazebo Classic `filename` attribute | Modern Gazebo `filename` attribute  | Modern Gazebo `name` attribute |
|-----------------|-------------------------------------|-------------------------------------|--------------------------------|
| Base state      | `libgazebo_yarp_basestate.so`       | `gz-sim-yarp-basestate-system`      | `gzyarp::BaseState`            |
| RGB Camera      | `libgazebo_yarp_camera.so`          | `gz-sim-yarp-camera-system`         | `gzyarp::Camera`               |
| Clock           | `libgazebo_yarp_clock.so`           | `gz-sim-yarp-clock-system`          | `gzyarp::Clock`                |
| Control board   | `libgazebo_yarp_lasersensor.so`     | `gz-sim-yarp-controlboard-system`   | `gzyarp::ControlBoard`         |
| Force-Torque    | `libgazebo_yarp_forcetorque.so`     | `gz-sim-yarp-forcetorque-system`    | `gzyarp::ForceTorque`          |
| IMU             | `libgazebo_yarp_imu.so`             | `gz-sim-yarp-imu-system`            | `gzyarp::Imu`                  |
| Laser           | `libgazebo_yarp_lasersensor.so`     | `gz-sim-yarp-laser-system`          | `gzyarp::Laser`                |
| Robot interface | `libgazebo_yarp_robotinterface.so`  | `gz-sim-yarp-robotinterface-system` | `gzyarp::RobotInterface`       |
| Configuration Override | `libgazebo_yarp_configurationoverride.so`  | `gz-sim-yarp-configurationoverride-system` | `gzyarp::ConfigurationOverride`       |


#### URDF

If your robot description is in a URDF file, then you should add the `gz-sim-yarp-plugins` you need under `<robot>` and inside a `<gazebo>` element, like in the following example:

```xml
<?xml version="1.0"?>
<robot name="wall-e">
    <!-- ... -->
    <gazebo>
        <plugin name="gzyarp::Imu" filename="gz-sim-yarp-imu-system">
            <!-- <yarpConfigurationFile> or <yarpConfigurationString> -->
        </plugin>
    </gazebo>
    <!-- ... -->
</robot>
```

From this example, just update the `name` and `filename` attributes of the `<plugin>` element with the ones found in the table above.

> [!NOTE]
> The `<sensor>` elements already present in the model do not need any modification.

#### SDF

If your robot description is in SDF format, then you should add the `gz-sim-yarp-plugins` you need under the `<model>` element, like in the following example:

```xml
<?xml version="1.0" ?>
<sdf version="1.11">
    <model name="wall-e">
        <!-- ... -->
        <plugin name="gzyarp::Imu" filename="gz-sim-yarp-imu-system">
            <!-- <yarpConfigurationFile> or <yarpConfigurationString> -->
        </plugin>
        <!-- ... -->
    </model>
</sdf>
```

### Modern Gazebo system plugins

One of the main differences of Modern Gazebo with respect to Gazebo Classic is its modularity; this means that several features that were included out-of-the-box in the Classic version now have to be explicitly added as system plugins.

For example, in order to be able to use a specific type of sensor with the `<sensor>` tag, a `gz-sim` plugin should be added. This can be done just once in the world SDF file containing the models needed for a simulation, or it can be also added to each model file (UDRF or SDF). The latter approach can be useful in all situations in which the robot is part of a library that will be used by different users that could not be aware they have to include such plugins in their world files, like for example [ergoCub](https://github.com/icub-tech-iit/ergocub-software).

> [!NOTE]
> Please note that the `gz-sim` plugins are different from `gz-sim-yarp-plugins` and both are needed to make the sensor information available from YARP devices.

Below are listed some examples related to sensors available with `gz-sim-yarp-plugins`, see the [official documentation](https://gazebosim.org/docs/harmonic/sensors) to learn more.

#### Force-Torque

```xml
<plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>
```

If the plugin is added to a URDF robot file, it should placed under `<robot>` and inside a `<gazebo>` element, just like the `gz-sim-yarp-plugins`, while for SDF files it can be placed under `<world>` or `<model>` elements.

#### Imu

```xml
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
```

#### Laser

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
```

#### Configuration Override

In `gazebo-yarp-plugins` with Gazebo Classic, the `libgazebo_yarp_configurationoverride.so` plugin can be used to override parameters in nested plugins, for example:

~~~xml
    <plugin name='configuration_override' filename='libgazebo_yarp_configurationoverride.so'>
      <yarpPluginConfigurationOverride plugin_name='controlboard_plugin'/>
      <initialConfiguration>0.785398</initialConfiguration>
    </plugin>
    <include>
      <uri>model://single_pendulum/model.sdf</uri>
    </include>
~~~

the equivalent syntax with `gz-sim-yarp-plugins` is:

~~~xml
    <!-- This is an example of how to override the initial configuration -->
    <plugin name='gzyarp::ConfigurationOverride' filename='gz-sim-yarp-configurationoverride-system'>
      <!-- The yarpPluginConfigurationOverride device is used to override the configuration of
           a given yarp device, whose name is specified via the yarpDeviceName attribute -->
      <yarpPluginConfigurationOverride yarpDeviceName='controlboard_plugin_device'/>

      <!-- If you want to override the initialConfiguration xml element, set it -->
      <initialConfiguration>0.785398</initialConfiguration>
    </plugin>

    <include>
      <uri>model://single_pendulum/model.sdf</uri>
    </include>
~~~

Beside the change of the `name` and `filename` attributes of the `plugin` element, the main difference is how we identify
the plugin of which we want to override the parameters: in `gazebo-yarp-plugins` we use the `name` element of the overriden plugin,
that may be unique even if you have multiple controlboards. In `gz-sim-yarp-plugins`, the `name` element of each controlboard plugin **must**
be `gzyarp::ControlBoard`, so it will not be unique among different controlboards. For this reason, to identify the plugin to override we
use instead the `yarpDeviceName`, that is unique for each plugin in the same model, and that needs already to be defined to expose each
plugins with Network Wrapper Servers instantiated by the `gzyarp::RobotInterface` plugin.

## Update of plugins configurations

`gz-sim-yarp-plugins` usually need a configuration, that can be specified in two ways:

- using the `<yarpConfigurationString>` element, which allows to specify all the parameters through a string directly;
- using `<yarpConfigurationFile>` element, specifying a path where the configuration file (usually a `.ini` file) is located;
- only for the robot interface plugin, with `<yarpRobotInterfaceConfigurationFile>` element, specifying the path of the [YARP XML configuration file](https://www.yarp.it/latest/group__yarp__robotinterface__xml__config__files.html).

See the [YARP documentation](https://www.yarp.it/latest/yarp_config_files.html) to learn the syntax needed for the configuration.

> [!NOTE]
> The configuration parameters needed by `gz-sim-yarp-plugins` are slightly different from the ones needed by `gazebo-yarp-plugins`.
> If you plan to use the same configuration file for both `gazebo-yarp-plugins` and `gz-sim-yarp-plugins`, you can just append the new parameters to the existing ones in the same file.

The parameters needed by each plugin can be found by looking at each [plugin](../plugins/) folder.

## World set up

World files are used to set up a simulation scenario containing all the required models and specific settings in Modern Gazebo. World files are written in [SDF](http://sdformat.org/) format and they replace the drag-and-drop way of placing models into the simulation from the _Insert_ tab of Gazebo Classic.

As already said, since Modern Gazebo has a modular architecture, the world file is also the place in which simulation settings are specified, such as the physics engine to use and the GUI components to show. See the [official documentation](https://gazebosim.org/docs/harmonic/sdf_worlds) to learn more, and feel free to check out the example worlds contained in the [tutorials](../tutorial/) folder.

## Environment variables

In order to find resources, Modern Gazebo uses some environment variables, that differ from the ones used by Gazebo Classic.
The most important variables are:

- `GZ_SIM_SYSTEM_PLUGIN_PATH` to find plugins;
- `GZ_SIM_RESOURCE_PATH` to find every other kind of resource, like worlds, models, meshes and everything that is specified through the `<include><uri>` elements.

Visit the [official documentation](https://gazebosim.org/api/sim/8/resources.html) to learn more.

These variables contain a colon-separated list of paths that Gazebo uses to find the resources it needs, hence in order to be able to run the simulation they have to be configured, ideally placing the `export` commands in the `.bashrc` or in conda activate scripts based on how you installed Modern Gazebo. If you followed the [README](../README.md), the `GZ_SIM_SYSTEM_PLUGIN_PATH` is already configured properly, while you should add to the `GZ_SIM_RESOURCE_PATH` variable the path(s) to your model(s).

## Examples of projects with Modern Gazebo support

In the following are listed the projects that already have support for Modern Gazebo along with the PR that introduced it.

| Project          | Repository                                        | PR introducing Modern Gazebo support                       |
|------------------|---------------------------------------------------|------------------------------------------------------------|
| ergocub-software | https://github.com/icub-tech-iit/ergocub-software | https://github.com/icub-tech-iit/ergocub-software/pull/230 |
