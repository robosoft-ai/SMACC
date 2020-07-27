#pragma once

#include <sm_ridgeback_floor_coverage_static_1/clients/cl_string_publisher/cl_string_publisher.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_ridgeback_floor_coverage_static_1
{
class OrStringPublisher : public smacc::Orthogonal<OrStringPublisher>
{
public:
    virtual void onInitialize() override
    {
        this->createClient<ClStringPublisher>("/string_publisher_out");
    }
};
} // namespace sm_ridgeback_floor_coverage_static_1