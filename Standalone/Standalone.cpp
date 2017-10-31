// Standalone.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>


#include <vehicles/multirotor/MultiRotor.hpp>
#include <vehicles/multirotor/MultiRotorParamsFactory.hpp>


int main()
{
  auto params = msr::airlib::MultiRotorParamsFactory::createConfig("SimpleFlight");


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

