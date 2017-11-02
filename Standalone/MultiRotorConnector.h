#pragma once
#include <api/ControlServerBase.hpp>
#include <vehicles/multirotor/api/DroneApi.hpp>
#include <controllers/VehicleConnectorBase.hpp>
#include <vehicles/multirotor/MultiRotor.hpp>

namespace msr {
  namespace airlib {
    class MultiRotorParams;
  }
}



class MultiRotorConnector : public msr::airlib::VehicleConnectorBase
{
public:
  MultiRotorConnector(msr::airlib::MultiRotorParams* vehicle_params,
                        bool enable_rpc, std::string api_server_address, uint16_t api_server_port);

  // Invoked from inside simulation lock.
  virtual void updateRenderedState() override;

  // Invoked from outside simulation lock.
  virtual void updateRendering(float dt) override;

  virtual void startApiServer() override;

  virtual void stopApiServer() override;

  virtual bool isApiServerStarted() override;

  virtual msr::airlib::VehicleControllerBase* getController() override;

  virtual msr::airlib::VehicleCameraBase* getCamera(unsigned int index = 0) override;

  virtual void setPose(const msr::airlib::Pose& pose, bool ignoreCollision) override;

  virtual msr::airlib::Pose getPose() override;

  virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id,
                                       bool is_name_regex = false) override;
  
  virtual int getSegmentationObjectID(const std::string& mesh_name) override;

  virtual msr::airlib::UpdatableObject* getPhysicsBody() override;

  virtual void reset() override;

  virtual void update() override;

  virtual void reportState(msr::airlib::StateReporter& reporter) override;


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