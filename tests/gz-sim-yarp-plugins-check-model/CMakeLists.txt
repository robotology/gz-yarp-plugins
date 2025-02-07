macro(gsyp_add_tutorial_model_check tutorialname worldrelpath)
    add_test(NAME gz-sim-yarp-plugins-check-model-tutorial-${tutorialname}
             COMMAND gz-sim-yarp-plugins-check-model --world-file ${PROJECT_SOURCE_DIR}/tutorial/${worldrelpath})
    set_tests_properties(gz-sim-yarp-plugins-check-model-tutorial-${tutorialname} PROPERTIES
        ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-device-registry>"
        ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${PROJECT_SOURCE_DIR}/tutorial")
endmacro()

# Check that the tutorial starts correctly
gsyp_add_tutorial_model_check(basestate basestate/model.sdf)
gsyp_add_tutorial_model_check(camera-model camera/model/model.sdf)
gsyp_add_tutorial_model_check(camera-model-horizontal-flip camera/model_horizontal_flip/model.sdf)
gsyp_add_tutorial_model_check(camera-model-vertical-flip camera/model_vertical_flip/model.sdf)
gsyp_add_tutorial_model_check(laser laser/model.sdf)
gsyp_add_tutorial_model_check(clock clock/model.sdf)
gsyp_add_tutorial_model_check(forcetorque-model-one-sensor forcetorque/model_one_sensor/model.sdf)
gsyp_add_tutorial_model_check(forcetorque-model-two-sensors forcetorque/model_two_sensors/model2sensors.sdf)
gsyp_add_tutorial_model_check(single-pendulum single_pendulum/single_pendulum_world.sdf)


if(GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN)
    gsyp_add_tutorial_model_check(coupled-pendulum coupled_pendulum/model.sdf)
endif()

# Then check that the command correctly exit a failure if we pass it a invalid model
add_test(NAME gz-sim-yarp-plugins-check-model-fails-as-expected-with-wrong-world-file
         COMMAND gz-sim-yarp-plugins-check-model --world-file  ./world_with_robotinterface_with_non_existing_filename.sdf
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_tests_properties(gz-sim-yarp-plugins-check-model-fails-as-expected-with-wrong-world-file PROPERTIES
    ENVIRONMENT  "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-device-registry>")
set_property(TEST gz-sim-yarp-plugins-check-model-fails-as-expected-with-wrong-world-file PROPERTY WILL_FAIL true)

# Check that the command fails if not argument is passed
add_test(NAME gz-sim-yarp-plugins-check-model-fails-as-expected-with-no-options
         COMMAND gz-sim-yarp-plugins-check-model)
set_property(TEST gz-sim-yarp-plugins-check-model-fails-as-expected-with-no-options PROPERTY WILL_FAIL true)

