#pragma once
#include <controllers/VehicleCameraBase.hpp>

class VehicleCameraConnector : public  msr::airlib::VehicleCameraBase
{
public:

  virtual ImageResponse getImage(ImageType image_type, bool pixels_as_float, bool compress) override;

   

};