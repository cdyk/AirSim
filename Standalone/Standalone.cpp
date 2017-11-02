// Standalone.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <thread>
#include <conio.h>
#include <common/common_utils/Utils.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include "AirSimRunner.h"

std::atomic<bool> run = true;

void foo()
{
  AirSimRunner world;
  world.start();
  while (run) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    world.update(1.0f);
  }
  world.stop();
}



int main()
{
  common_utils::Utils::getSetMinLogLevel(true, 1);

  std::thread fooThread(foo);
  msr::airlib::MultirotorRpcLibClient client;

  client.confirmConnection();

  //std::cerr << "Capturing FPV image.\n";
  //vector<msr::airlib::VehicleCameraBase::ImageRequest> request;
  //request.emplace_back(0, msr::airlib::VehicleCameraBase::ImageType::Scene);
  //request.emplace_back(1, msr::airlib::VehicleCameraBase::ImageType::DepthPlanner);
  //const auto & response = client.simGetImages(request);
  //std::cerr << response.size() << " images received.\n";

  client.enableApiControl(true);

  std::cerr << "disarm\n";
  client.armDisarm(true);

  std::cerr << "takeoff\n";
  client.takeoff(5);

  std::cerr << "hover\n";
  client.hover();

  auto position = client.getPosition();
  float z = position.z(); // current position (NED coordinate system).  
  const float speed = 3.0f;
  const float size = 10.0f;
  const float duration = size / speed;
  auto driveTrain = msr::airlib::DrivetrainType::ForwardOnly;
  msr::airlib::YawMode yaw_mode(true, 0);
  std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
  client.moveByVelocityZ(speed, 0, z, duration, driveTrain, yaw_mode);
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));
  std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
  client.moveByVelocityZ(0, speed, z, duration, driveTrain, yaw_mode);
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));
  std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
  client.moveByVelocityZ(-speed, 0, z, duration, driveTrain, yaw_mode);
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));
  std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
  client.moveByVelocityZ(0, -speed, z, duration, driveTrain, yaw_mode);
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));

  client.hover();


  int pc = 0;
  int ticks = 0;
  while (_kbhit() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  run = false;
  fooThread.join();

  return 0;
}

