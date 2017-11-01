// Standalone.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>


#include <vehicles/multirotor/MultiRotor.hpp>
#include <vehicles/multirotor/MultiRotorParamsFactory.hpp>
#include <physics/PhysicsWorld.hpp>
#include <physics/FastPhysicsEngine.hpp>

namespace {


  class FooUpdateableObject : public msr::airlib::UpdatableObject
  {

    virtual void reset() override
    {
      msr::airlib::UpdatableObject::reset();
    }

    virtual void update() override
    {
      msr::airlib::UpdatableObject::update();
    }

    virtual void reportState(msr::airlib::StateReporter& reporter) override
    {
    }

    virtual msr::airlib::UpdatableObject* getPhysicsBody() override
    {
      return msr::airlib::UpdatableObject::getPhysicsBody();
    }

    virtual msr::airlib::ClockBase* clock() override
    {
      return msr::airlib::UpdatableObject::clock();
    }
  };

}


int main()
{

  long long tickLength = 3000000LL; //3ms

  // setup clock
  msr::airlib::ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>());


  auto params = msr::airlib::MultiRotorParamsFactory::createConfig("SimpleFlight");

  auto physicsEngine = std::make_unique<msr::airlib::FastPhysicsEngine>();

  std::vector<msr::airlib::UpdatableObject*> vehicles;
  vehicles.push_back(new FooUpdateableObject());


  auto physicsWorld = std::make_unique<msr::airlib::PhysicsWorld>(physicsEngine.get(),
                                                                  vehicles,
                                                                  tickLength);

  msr::airlib::GeoPoint testLocation(47.7631699747, -122.0685655406, 9.93f); // woodinville

  auto kinematics = msr::airlib::Kinematics::State::zero();
  msr::airlib::Environment environment(msr::airlib::Environment::State(kinematics.pose.position, testLocation));

  msr::airlib::MultiRotor vehicle;

  vehicle.initialize(params.get(), kinematics, &environment);


  auto * controller = vehicle.getController();

  char c;
  std::cerr << "moo";

  std::cin >> c;

  return 0;
}

