# Run coupled pendulum tutorial in Gazebo

## Run model in Gazebo with YARP integration

- 1st terminal:

  ~~~bash
  yarp server
  ~~~

- 2nd terminal:
  - Update the `GZ_SIM_RESOURCE_PATH` environment variable to point to the `tutorial` folder:

    ~~~
    export GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH:<path-to-tutorial-folder>
    ~~~

  - Then, launch Gazebo:

    ~~~
    cd <path-to-tutorial-folder>/coupled_pendulum
    gz sim coupled_pendulum_world.sdf
    ~~~

  The Gazebo GUI will open and the coupled pendulum should be already spawned in the scene.

- 3rd terminal:

  ~~~bash
    cd tutorial/coupled_pendulum
    yarpmotorgui
  ~~~

  The [yarpmotorgui](https://www.yarp.it/latest/group__yarpmotorgui.html) interface will open and it should automatically prompt you to select the `/coupledPendulumGazebo/body` part.

  Click Ok.

Finally start the simulation in Gazebo (click on the Play button on the bottom left).
