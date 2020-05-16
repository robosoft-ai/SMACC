#pragma once

#include <sm_opencv_3/clients/opencv_perception_client/cl_opencv_perception_client.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_opencv_3
{
class OrPerception : public smacc::Orthogonal<OrPerception>
{
public:
  virtual void onInitialize() override
  {
    auto opencvPerceptionClient = this->createClient<cl_opencv_perception::ClOpenCVPerception>();
    opencvPerceptionClient->initialize();
  }
};
}  // namespace sm_opencv_3