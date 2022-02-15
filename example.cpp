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

    std::array<double, 3> position = {0., 0., 0.6};
    std::array<double, 4> quaternion = {1., 0., 0., 0.};
    std::array<double, 3> velocity = {0., 0., 0.};
    scenario::core::Pose pose(position, quaternion);

    // Insert a robot
    const std::string robotURDF = "biped.urdf"; // "pendulum.urdf";
    world->insertModel(/*modelFile=*/robotURDF);
    gazebo.run(/*paused=*/true);

    // Get the robot
    auto robot = world->getModel(/*modelName=*/"biped");
    auto robotGazebo = std::static_pointer_cast<scenario::gazebo::Model>(robot);

    robot->enableContacts(true);
    robotGazebo->enableContacts(true);
    robotGazebo->enableSelfCollisions(true);

    // Set the joint control mode
    // robot->setJointControlMode(scenario::core::JointControlMode::Position);
    robot->setJointControlMode(scenario::core::JointControlMode::Force);
    scenario::core::PID pidGains(40., 0., 1.);

    // Set the joint PID gains
    // for(const auto& jointName : robot->jointNames()) {
    //     auto jointPtr = robot->getJoint(jointName);
    //     jointPtr->setPID(pidGains);
    // }

    // Set the joint position targets
    // std::vector<double> jointPositionTargets = std::vector<double>({0., 0.5, -0.8, 0., -0.5, 0.8, 0., 0.5, -0.8, 0., -0.5, 0.8});
    // robot->setJointPositionTargets(jointPositionTargets);
    std::vector<double> jointPositionTargets = std::vector<double>({-0.2, -0.57359877559, 1.308996939, 0.2, -0.57359877559, 1.308996939});
    robot->setControllerPeriod(gazebo.stepSize());

    std::vector<double> errCurr = std::vector<double>({0., 0., 0., 0., 0., 0.});

    robotGazebo->resetBasePose(position, quaternion);
    robotGazebo->resetBaseWorldLinearVelocity(velocity);
    robotGazebo->resetBaseWorldAngularVelocity(velocity);
    robotGazebo->resetJointPositions(jointPositionTargets, robot->jointNames());
    robotGazebo->resetJointVelocities(errCurr, robot->jointNames());

    // auto link_ptr = robot->getLink("LF_SHANK");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Simulate 30 seconds
    std::vector<double> errPrev = errCurr;
    std::vector<double> effort = errCurr;
    std::vector<double> errInt = errCurr;
    for (size_t i = 0; i < 45.0 / gazebo.stepSize(); ++i) {

        // if(int(i % 3000) == 0) {
        //     robotGazebo->resetBasePose(position, quaternion);
        //     robotGazebo->resetBaseWorldLinearVelocity(velocity);
        //     robotGazebo->resetBaseWorldAngularVelocity(velocity);
        //     robotGazebo->resetJointPositions(jointPositionTargets, robot->jointNames());
        //     robotGazebo->resetJointVelocities(errCurr, robot->jointNames());
        // }

        gazebo.run();
        // std::cout << i * gazebo.stepSize() << std::endl;
        
        if(i % 2 == 0) { // 500 Hz PD
            auto joint_positions = robot->jointPositions(robot->jointNames());
            auto joint_velocities = robot->jointVelocities(robot->jointNames());
            for(int j = 0; j < robot->nrOfJoints(); j++) {
                errCurr[j] = jointPositionTargets[j] - joint_positions[j];

                auto jointName = robot->jointNames()[j];

                double P = 80.0;
                double D = 2.0;
                double I = 0.;
                // if(jointName.find("knee") != std::string::npos) {
                //     P = 400.;
                //     D = 100.;
                //     I = 200.;
                // }

                effort[j] = P * errCurr[j] + I * errInt[j] - D * joint_velocities[j];

                errInt[j] += gazebo.stepSize() * errCurr[j];
                errPrev[j] = errCurr[j];

                std::cout << jointName << ": " << joint_positions[j] << " "; // effort[j] << " ";
            }
            std::cout << std::endl;
        }

        // std::cout << "diff: " << joint_positions[0] - joint_positions[3] << std::endl;

        robot->setJointGeneralizedForceTargets(effort);

        // auto contacts = link_ptr->contacts();
        // for(const auto& contact : contacts) {
        //     if((contact.bodyA.find("ground") != std::string::npos) || (contact.bodyB.find("ground") != std::string::npos)){
        //         std::cout << "LF_SHANK touched ground!" << std::endl;
        //     }
        // }
    }

    // Close the simulator
    std::this_thread::sleep_for(std::chrono::seconds(3));
    gazebo.close();
    return 0;  
}