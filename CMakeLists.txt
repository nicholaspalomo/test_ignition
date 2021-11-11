cmake_minimum_required(VERSION 3.16)
project(ExampleWithScenario VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Scenario COMPONENTS Gazebo REQUIRED)

add_executable(ExampleWithScenario example.cpp)

target_link_libraries(ExampleWithScenario PRIVATE
    ScenarioGazebo::ScenarioGazebo
    ScenarioGazebo::GazeboSimulator)