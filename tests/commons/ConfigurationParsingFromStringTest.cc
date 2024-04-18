#include <gtest/gtest.h>

#include <BaseStateDriver.cpp>
#include <CameraDriver.cpp>
#include <ControlBoardDriver.hh>
#include <DeviceRegistry.hh>
#include <ForceTorqueDriver.cpp>
#include <ImuDriver.cpp>
#include <LaserDriver.cpp>
#include <TestHelpers.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/TestFixture.hh>
#include <sdf/Element.hh>

#include <iostream>
#include <memory>
#include <string>

#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

TEST(ConfigurationParsingTest, LoadPluginsWithYarpConfigurationString)
{
    std::string modelSdfName = "model_configuration_string.sdf";
    std::string sdfPath = std::string("../../../tests/commons/") + modelSdfName;
    yarp::os::NetworkBase::setLocalMode(true);

    gz::sim::TestFixture testFixture{sdfPath};
    gz::common::Console::SetVerbosity(4);

    testFixture.OnConfigure([&](const gz::sim::Entity& _worldEntity,
                                const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                                gz::sim::EntityComponentManager& _ecm,
                                gz::sim::EventManager& /*_eventMgr*/) {
        // Test Camera
        std::cerr << "Testing Camera configuration" << std::endl;
        auto cameraDrivers
            = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::CameraDriver>(_ecm);
        EXPECT_EQ(cameraDrivers.size(), 1);
        EXPECT_TRUE(cameraDrivers[0] != nullptr);

        // Test ForceTorque
        std::cerr << "Testing FT configuration" << std::endl;
        auto ftDrivers
            = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::ForceTorqueDriver>(
                _ecm);
        EXPECT_EQ(ftDrivers.size(), 1);
        EXPECT_TRUE(ftDrivers[0] != nullptr);

        // Test Imu
        std::cerr << "Testing Imu configuration" << std::endl;
        auto imuDrivers
            = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::ImuDriver>(_ecm);
        EXPECT_EQ(imuDrivers.size(), 1);
        EXPECT_TRUE(imuDrivers[0] != nullptr);

        // Test Laser
        std::cerr << "Testing Laser configuration" << std::endl;
        auto laserDrivers
            = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::LaserDriver>(_ecm);
        EXPECT_EQ(laserDrivers.size(), 1);
        EXPECT_TRUE(laserDrivers[0] != nullptr);

        // Test basestate
        std::cerr << "Testing BaseState configuration" << std::endl;
        auto bsDrivers
            = gzyarp::testing::TestHelpers::getDevicesOfType<yarp::dev::gzyarp::BaseStateDriver>(
                _ecm);
        EXPECT_EQ(bsDrivers.size(), 1);
        EXPECT_TRUE(bsDrivers[0] != nullptr);

        // Controlboard test skipped since configuration too complex to pass as string
    });

    testFixture.Finalize();
}
