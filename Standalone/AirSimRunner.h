#pragma once
#include <string>
#include <memory>
#include <vector>
#include <physics/PhysicsWorld.hpp>
#include <physics/PhysicsEngineBase.hpp>
#include <controllers/VehicleConnectorBase.hpp>
#include <vehicles/multirotor/MultiRotorParams.hpp>

class AirSimRunner
{
  long long tickLength = 3000000LL; //3ms
  bool enableReport = false;

public:
  AirSimRunner();

  void start();

  void update(float dt);

  void stop();

  void readSettings();

protected:
  void createVehicles(std::vector<std::shared_ptr<msr::airlib::VehicleConnectorBase>>& v);

private:
  bool enableRpc = true;
  std::string apiServerAddress;
  std::string defaultVehicleConfig;

  std::unique_ptr<msr::airlib::PhysicsWorld> physicsWorld;
  std::unique_ptr<msr::airlib::PhysicsEngineBase> physicsEngine;
  std::vector<std::shared_ptr<msr::airlib::VehicleConnectorBase>> vehicles;

  std::vector <std::unique_ptr<msr::airlib::MultiRotorParams> > vehicleParams;
  std::shared_ptr<msr::airlib::VehicleConnectorBase> multiRotorConnector;
};