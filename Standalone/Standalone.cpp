// Standalone.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <thread>
#include <conio.h>

#include <common/common_utils/Utils.hpp>
#include <api/ControlServerBase.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibServer.hpp>
#include <vehicles/multirotor/api/DroneApi.hpp>
#include <vehicles/multirotor/MultiRotor.hpp>
#include <vehicles/multirotor/MultiRotorParamsFactory.hpp>
#include <controllers/VehicleConnectorBase.hpp>
#include <physics/PhysicsWorld.hpp>
#include <physics/FastPhysicsEngine.hpp>


namespace {


  class MyMultiRotorConnector : public msr::airlib::VehicleConnectorBase
  {
  public:
    MyMultiRotorConnector(msr::airlib::MultiRotorParams* vehicle_params,
                         bool enable_rpc, std::string api_server_address, uint16_t api_server_port)
      : enable_rpc(enable_rpc), api_server_address(api_server_address), api_server_port(api_server_port)
    {

      auto initialPose = msr::airlib::Pose::zero();
      //auto initialKinematicState = msr::airlib::Kinematics::State::zero();
      auto homePoint = msr::airlib::GeoPoint(47.7631699747, -122.0685655406, 9.93f); // woodinville
      vehicle.initialize(vehicle_params,
                         initialPose,
                         homePoint,
                         environment);
      assert(environment);
      controller = vehicle.getController();
    }

    virtual void updateRenderedState() override { }

    virtual void updateRendering(float dt) override
    {
#if 0
      std::cerr
        << "pos=["
        << vehicle.getPose().position[0] << ", "
        << vehicle.getPose().position[1] << ", "
        << vehicle.getPose().position[2] << "], out=["
        << vehicle.getRotorOutput(0).thrust << ", "
        << vehicle.getRotorOutput(1).thrust << ", "
        << vehicle.getRotorOutput(2).thrust << ", "
        << vehicle.getRotorOutput(3).thrust << "]\n";
#endif
    }

    virtual void startApiServer() override
    {
      if (enable_rpc) {
        controllerCanceable.reset(new msr::airlib::DroneApi(this));
        rpclibServer.reset(new msr::airlib::MultirotorRpcLibServer(controllerCanceable.get(), api_server_address, api_server_port));
        rpclibServer->start();
      }
    }

    virtual void stopApiServer() override
    {
      if (controllerCanceable) {
        controllerCanceable->cancelAllTasks();
        rpclibServer->stop();
        rpclibServer.reset();
        controllerCanceable.reset();
      }
    }

    virtual bool isApiServerStarted() override { return rpclibServer != nullptr; }

    virtual msr::airlib::VehicleControllerBase* getController() override { return controller; }

    virtual msr::airlib::VehicleCameraBase* getCamera(unsigned int index = 0) override { return nullptr; }  // return cameraConnector.

    virtual void setPose(const msr::airlib::Pose& pose, bool ignoreCollision) override { pending.pose = pose; pending.ignoreCollision = ignoreCollision; }

    virtual msr::airlib::Pose getPose() override { return vehicle.getPose(); }

    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id,
                                         bool is_name_regex = false) override { return true; }

    virtual int getSegmentationObjectID(const std::string& mesh_name) override { return 0; }

    virtual msr::airlib::UpdatableObject* getPhysicsBody() override { return vehicle.getPhysicsBody(); }

    virtual void reset() override { vehicle.reset(); }
    virtual void update() override { vehicle.update(); }
    virtual void reportState(msr::airlib::StateReporter& reporter) override { vehicle.reportState(reporter); }


  private:
    bool enable_rpc;
    std::string api_server_address;
    uint16_t api_server_port;
    std::unique_ptr<msr::airlib::DroneApi> controllerCanceable;
    std::unique_ptr<msr::airlib::ControlServerBase> rpclibServer;


    std::unique_ptr<msr::airlib::Environment> environment;
    msr::airlib::MultiRotor vehicle;
    msr::airlib::DroneControllerBase* controller;

    struct {
      msr::airlib::Pose pose;
      bool ignoreCollision;
    } pending;

  };

  class MySimModeWorld
  {
    long long tickLength = 3000000LL; //3ms
    bool enableReport = false;

  public:
    MySimModeWorld()
    {


    }

    void start()
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

      if (fpv_vehicle_connector) fpv_vehicle_connector->startApiServer();
   
      physicsWorld->startAsyncUpdator();  // Not sure when this should be called.
    }

    void update(float dt)
    {
      physicsWorld->lock();
      physicsWorld->enableStateReport(enableReport);
      physicsWorld->updateStateReport();
      for (auto& vehicle : vehicles) vehicle->updateRenderedState();
      physicsWorld->unlock();
      for (auto& vehicle : vehicles) vehicle->updateRendering(dt);
    }

    void stop()
    {
      physicsWorld->stopAsyncUpdator(); // Not sure when this should be called.

      if (fpv_vehicle_connector) fpv_vehicle_connector->stopApiServer();

      physicsWorld.reset();
      physicsEngine.reset();
    }

    void readSettings()
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

  protected:
    void createVehicles(std::vector<std::shared_ptr<msr::airlib::VehicleConnectorBase>>& v)
    {
      auto params = msr::airlib::MultiRotorParamsFactory::createConfig("SimpleFlight");
      vehicleParams.push_back(std::move(params));

      fpv_vehicle_connector = std::make_shared<MyMultiRotorConnector>(vehicleParams.back().get(),
                                                                      enableRpc,
                                                                      apiServerAddress,
                                                                      vehicleParams.back()->getParams().api_server_port);
      v.push_back(fpv_vehicle_connector);
    }

  private:
    bool enableRpc = true;
    std::string apiServerAddress;
    std::string defaultVehicleConfig;

    std::unique_ptr<msr::airlib::PhysicsWorld> physicsWorld;
    std::unique_ptr<msr::airlib::PhysicsEngineBase> physicsEngine;
    std::vector<std::shared_ptr<msr::airlib::VehicleConnectorBase>> vehicles;

    std::vector <std::unique_ptr<msr::airlib::MultiRotorParams> > vehicleParams;
    std::shared_ptr<msr::airlib::VehicleConnectorBase> fpv_vehicle_connector;
  };


}

std::atomic<bool> run = true;

void foo()
{
  MySimModeWorld world;
  world.start();
  while (run) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    world.update(0.5f);
  }
  world.stop();
}


enum What
{
  DISARM,
  TAKEOFF,
  HOVER,
  GET_IMAGE,
  SLEEP_FOREVER
};


struct  
{
  What cmd;
  int wait;
}
commands[] =
{
  {GET_IMAGE, 0},
  {DISARM, 0},
  {TAKEOFF, 5},
  {HOVER, 0},
  {SLEEP_FOREVER, 0}
};



int main()
{
  common_utils::Utils::getSetMinLogLevel(true, 1);

  std::thread fooThread(foo);
  msr::airlib::MultirotorRpcLibClient client;
  client.confirmConnection();
  client.enableApiControl(true);

  int pc = 0;
  int ticks = 0;
  while (_kbhit() == 0) {

    if (0 <= pc) {
      ticks -= 1;
      if (ticks < 0) {
        switch (commands[pc].cmd)
        {
        case DISARM:
          std::cerr << "disarm\n";
          client.armDisarm(true);
          break;
        case TAKEOFF:
          std::cerr << "takeoff\n";
          client.takeoff(2);
          std::cerr << "takeoff done\n";
          break;
        case HOVER:
          std::cerr << "hover\n";
          client.hover();
          break;
        case GET_IMAGE:
        {
          //std::cerr << "Capturing FPV image.\n";
          //vector<msr::airlib::VehicleCameraBase::ImageRequest> request;
          //request.emplace_back(0, msr::airlib::VehicleCameraBase::ImageType::Scene);
          //request.emplace_back(1, msr::airlib::VehicleCameraBase::ImageType::DepthPlanner);
          //const auto & response = client.simGetImages(request);
          //std::cerr << response.size() << " images received.\n";
          break;
        }
        case SLEEP_FOREVER:
          std::cerr << "Sleep forever.\n";
          pc = -2;
          break;
        }
        ticks = commands[pc].wait;
        pc++;
      }
    }

    auto gps = client.getGpsLocation();
    std::cerr
      << "lat=" << gps.latitude << ", "
      << "lon=" << gps.longitude << ", "
      << "alt=" << gps.altitude << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  fooThread.join();

  return 0;
}

