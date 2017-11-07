#define AIRSIM_NO_IMPL
#include "stdafx.h"
#include <conio.h>

#include "MultiRotorConnector.h"

#include <vehicles/multirotor/api/MultirotorRpcLibServer.hpp>
#include <vehicles/multirotor/MultiRotorParamsFactory.hpp>


MultiRotorConnector::MultiRotorConnector(msr::airlib::MultiRotorParams* vehicle_params,
                                             bool enable_rpc, std::string api_server_address, uint16_t api_server_port)
  : enable_rpc(enable_rpc), api_server_address(api_server_address), api_server_port(api_server_port)
{
  auto initialPose = msr::airlib::Pose::zero();
  auto homePoint = msr::airlib::GeoPoint(47.7631699747, -122.0685655406, 9.93f); // woodinville

  vehicle.initialize(vehicle_params,
                     initialPose,
                     homePoint,
                     environment);

  assert(environment);

  controller = vehicle.getController();

  cameraConnector = std::make_unique<VehicleCameraConnector>();
}

void MultiRotorConnector::updateRenderedState()
{
  auto curr = std::chrono::steady_clock::now();
  std::chrono::duration<float> step = std::chrono::duration_cast<std::chrono::milliseconds>(curr - rcDataPrev);
  rcDataPrev = curr;

  auto l = std::exp(-10.f*step.count());
  rcData.yaw *= l;
  rcData.pitch *= l;
  rcData.throttle *= l;

  if (std::abs(rcData.yaw) < 0.01f) rcData.yaw = 0.f;
  if (std::abs(rcData.pitch) < 0.01f) rcData.pitch = 0.f;
  if (std::abs(rcData.throttle) < 0.01f) rcData.throttle = 0.f;


  rcData.is_valid = true;
  while (_kbhit()) {
    auto k = getch();
    switch (k) {
    case 'w':
      rcData.yaw = -1.f;
      break;
    case 'a':
      rcData.pitch = -1.f;
      break;
    case 's':
      rcData.pitch = -1.f;
      break;
    case 'd':
      rcData.yaw = 1.f;
      break;
    case 'z':
      rcData.throttle = 1.f;
      break;
    case 'x':
      rcData.throttle = -1.f;
      break;
    }
  }
  controller->setRCData(rcData);
}

void MultiRotorConnector::updateRendering(float dt)
{
  timeToLog -= dt;
  if (timeToLog <= 0.f) {
    std::cerr
      << "pos=["
      << vehicle.getPose().position[0] << ", "
      << vehicle.getPose().position[1] << ", "
      << vehicle.getPose().position[2] << "], out=["
      << vehicle.getRotorOutput(0).thrust << ", "
      << vehicle.getRotorOutput(1).thrust << ", "
      << vehicle.getRotorOutput(2).thrust << ", "
      << vehicle.getRotorOutput(3).thrust << "], "
      << rcData.yaw << ", "
      << rcData.pitch << ", "
      << rcData.throttle << "\n";
      timeToLog = 0.5f;
  }
}

void MultiRotorConnector::startApiServer()
{
  if (enable_rpc) {
    controllerCanceable.reset(new msr::airlib::DroneApi(this));
    rpclibServer.reset(new msr::airlib::MultirotorRpcLibServer(controllerCanceable.get(), api_server_address, api_server_port));
    rpclibServer->start();
  }
}

void MultiRotorConnector::stopApiServer()
{
  if (controllerCanceable) {
    controllerCanceable->cancelAllTasks();
    rpclibServer->stop();
    rpclibServer.reset();
    controllerCanceable.reset();
  }
}

bool MultiRotorConnector::isApiServerStarted()
{
  return rpclibServer != nullptr;
}

msr::airlib::VehicleControllerBase* MultiRotorConnector::getController()
{
  return controller;
}

msr::airlib::VehicleCameraBase* MultiRotorConnector::getCamera(unsigned int index)
{
  return cameraConnector.get();
}

void MultiRotorConnector::setPose(const msr::airlib::Pose& pose, bool ignoreCollision)
{
  pending.pose = pose;
  pending.ignoreCollision = ignoreCollision;
}

msr::airlib::Pose MultiRotorConnector::getPose()
{
  return vehicle.getPose();
}

bool MultiRotorConnector::setSegmentationObjectID(const std::string& mesh_name,
                                                    int object_id,
                                                    bool is_name_regex)
{
  return true;
}

int MultiRotorConnector::getSegmentationObjectID(const std::string& mesh_name)
{
  return 0;
}

msr::airlib::UpdatableObject* MultiRotorConnector::getPhysicsBody()
{
  return vehicle.getPhysicsBody();
}

void MultiRotorConnector::reset()
{
  vehicle.reset();
}

void MultiRotorConnector::update()
{
  vehicle.update();
}

void MultiRotorConnector::reportState(msr::airlib::StateReporter& reporter)
{
  vehicle.reportState(reporter);
}
