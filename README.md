# test_ignition

Simple repo to test the C++ example found [here](https://robotology.github.io/gym-ignition/master/getting_started/scenario.html).

To run the code:

1. Follow the ScenarIO installation instructions found [here](https://github.com/robotology/gym-ignition/tree/master/scenario#installation).

2. Navigate to the `build` folder and run:

```
cmake ..
make
```

3. From the `build` folder, run the executable with:

```
./ExampleWithScenario
```

The Ignition Gazebo gui should appear and you should see the robot fall in the air for 30 s.