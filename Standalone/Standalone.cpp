// Standalone.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <thread>
#include <conio.h>
#include <fstream>
#include <common/common_utils/Utils.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include "AirSimRunner.h"

std::atomic<bool> run = true;

void foo()
{
  AirSimRunner world;
  world.start();

  auto prev = std::chrono::steady_clock::now();
  while (run) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto curr = std::chrono::steady_clock::now();
    std::chrono::duration<float> step = std::chrono::duration_cast<std::chrono::milliseconds>(curr - prev);
    world.update(step.count());
    prev = curr;
  }
  world.stop();
}



int main()
{
  common_utils::Utils::getSetMinLogLevel(true, 1);

  //std::thread fooThread(foo);

  msr::airlib::MultirotorRpcLibClient client;
  client.confirmConnection();

  std::cerr << "Capturing FPV image.\n";
  std::vector<msr::airlib::VehicleCameraBase::ImageRequest> request;
  request.emplace_back(0, msr::airlib::VehicleCameraBase::ImageType::Scene);
  request.emplace_back(0, msr::airlib::VehicleCameraBase::ImageType::DepthPlanner);
  const auto & response = client.simGetImages(request);
  for (size_t i = 0; i < response.size(); i++) {
    auto & r = response[i];
    if (r.pixels_as_float == false && r.compress) {
      if (r.image_data_uint8.empty()) continue;

      std::string path = "capture" + std::to_string(i) + ".png";

      std::ofstream o(path, std::ios::binary | std::ios::trunc);
      o.write(reinterpret_cast<const char*>(r.image_data_uint8.data()), r.image_data_uint8.size());
      o.close();

      std::cerr << "Wrote " << path << "\n";
    }
  }

  std::cerr << response.size() << " images received.\n";


  client.enableApiControl(true);
  //std::this_thread::sleep_for(std::chrono::duration<double>(5.0));
  //client.enableApiControl(false);
  //std::cerr << "no control\n";


#if 1




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
#endif

  int pc = 0;
  int ticks = 0;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  run = false;
  //fooThread.join();

  return 0;
}

