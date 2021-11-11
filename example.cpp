#include <scenario/gazebo/GazeboSimulator.h>
#include <scenario/gazebo/Joint.h>
#include <scenario/gazebo/Model.h>
#include <scenario/gazebo/World.h>

#include <chrono>
#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    // Create the simulator
    auto gazebo = scenario::gazebo::GazeboSimulator(
        /*stepSize=*/0.001, /*rtf=*/1.0, /*stepsPerRun=*/1);

    // Initialize the simulator
    gazebo.initialize();

    // Get the default world
    auto world = gazebo.getWorld();

    // Insert the ground plane
    const std::string groundPlaneSDF = "ground_plane.sdf";
    world->insertModel(groundPlaneSDF);

    // Select the physics engine
    world->setPhysicsEngine(scenario::gazebo::PhysicsEngine::Dart);

    // Open the GUI
    gazebo.gui();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    gazebo.run(/*paused=*/true);

    // Insert a robot
    const std::string robotURDF = "anymal.urdf"; // "pendulum.urdf";
    world->insertModel(/*modelFile=*/robotURDF);
    gazebo.run(/*paused=*/true);

    // Get the robot
    auto robot = world->getModel(/*modelName=*/"anymal");
    auto robotGazebo = std::static_pointer_cast<scenario::gazebo::Model>(robot);

    // Set the joint control mode
    robot->setJointControlMode(scenario::core::JointControlMode::Position);
    scenario::core::PID pidGains(40., 0., 1.);

    // Set the joint PID gains
    for(const auto& jointName : robot->jointNames()) {
        auto jointPtr = robot->getJoint(jointName);
        jointPtr->setPID(pidGains);
    }

    // Set the joint position targets
    std::vector<double> jointPositionTargets = std::vector<double>({0., 0.5, -0.8, 0., -0.5, 0.8, 0., 0.5, -0.8, 0., -0.5, 0.8});
    robot->setJointPositionTargets(jointPositionTargets);
    robot->setControllerPeriod(gazebo.stepSize());

    std::array<double, 3> position = {0., 0., 1.0};
    std::array<double, 4> quaternion = {1., 0., 0., 0.};
    std::array<double, 3> velocity = {0., 0., 0.};
    robotGazebo->resetBasePose(position, quaternion);
    robotGazebo->resetBaseWorldLinearVelocity(velocity);
    robotGazebo->resetBaseWorldAngularVelocity(velocity);

    // Simulate 30 seconds
    for (size_t i = 0; i < 30.0 / gazebo.stepSize(); ++i) {
        gazebo.run();
    }

    // Close the simulator
    std::this_thread::sleep_for(std::chrono::seconds(3));
    gazebo.close();

    return 0;
}