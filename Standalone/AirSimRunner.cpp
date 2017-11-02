#include <physics/PhysicsWorld.hpp>
#include <physics/FastPhysicsEngine.hpp>
#include <controllers/Settings.hpp>
#include <vehicles/multirotor/MultiRotorParamsFactory.hpp>

#include "AirSimRunner.h"
#include "MultiRotorConnector.h"

AirSimRunner::AirSimRunner()
{


}

void AirSimRunner::start()
{
  readSettings();

  msr::airlib::ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>()); // setup clock

  createVehicles(vehicles);

  std::vector<msr::airlib::UpdatableObject*> updateableObjects;
  for (auto v : vehicles) {
    updateableObjects.push_back(v.get());
  }
  physicsEngine.reset(new msr::airlib::FastPhysicsEngine());
  physicsWorld.reset(new msr::airlib::PhysicsWorld(physicsEngine.get(),
                                                   updateableObjects,
                                                   tickLength,
                                                   true,
                                                   false));

  if (multiRotorConnector) multiRotorConnector->startApiServer();

  physicsWorld->startAsyncUpdator();  // Not sure when this should be called.
}

void AirSimRunner::update(float dt)
{
  physicsWorld->lock();
  physicsWorld->enableStateReport(enableReport);
  physicsWorld->updateStateReport();
  for (auto& vehicle : vehicles) vehicle->updateRenderedState();
  physicsWorld->unlock();
  for (auto& vehicle : vehicles) vehicle->updateRendering(dt);
}

void AirSimRunner::stop()
{
  physicsWorld->stopAsyncUpdator(); // Not sure when this should be called.

  if (multiRotorConnector) multiRotorConnector->stopApiServer();

  physicsWorld.reset();
  physicsEngine.reset();
}

void AirSimRunner::readSettings()
{
  enableRpc = true;
  apiServerAddress = "";
  defaultVehicleConfig = "";

  auto & settings = msr::airlib::Settings::singleton();

  auto simmode_name = settings.getString("SimMode", "");
  if (simmode_name == "") simmode_name = "Multirotor";

  //usage_scenario = settings.getString("UsageScenario", "");
  defaultVehicleConfig = settings.getString("DefaultVehicleConfig", "");
  if (defaultVehicleConfig == "") {
    if (simmode_name == "Multirotor")
      defaultVehicleConfig = "SimpleFlight";
    else
      assert(false && "Unknown simmode_name");
  }

  enableRpc = settings.getBool("RpcEnabled", true);
  apiServerAddress = settings.getString("LocalHostIp", "");
}

void AirSimRunner::createVehicles(std::vector<std::shared_ptr<msr::airlib::VehicleConnectorBase>>& v)
{
  auto params = msr::airlib::MultiRotorParamsFactory::createConfig("SimpleFlight");
  vehicleParams.push_back(std::move(params));

  multiRotorConnector = std::make_shared<MultiRotorConnector>(vehicleParams.back().get(),
                                                                  enableRpc,
                                                                  apiServerAddress,
                                                                  vehicleParams.back()->getParams().api_server_port);
  v.push_back(multiRotorConnector);
}

