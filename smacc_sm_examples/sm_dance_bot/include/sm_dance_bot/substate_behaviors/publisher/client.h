#include <smacc/interface_components/smacc_topic_publisher.h>
#include <std_msgs/String.h>

namespace smacc
{

class StringPublisherClient: public smacc::SmaccTopicPublisherClient<std_msgs::String>
{

};
}