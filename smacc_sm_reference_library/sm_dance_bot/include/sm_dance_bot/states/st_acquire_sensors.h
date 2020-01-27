#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StAcquireSensors : smacc::SmaccState<StAcquireSensors, MsDanceBotRunMode>
{
   // CUSTOM TRANSITION TAGS
   struct ON_KEYBOARD : PREEMPT{};
   struct ON_KEYBOARD2 : ABORT{};
   struct ON_SENSORS_AVAILABLE : SUCCESS{};

   struct SBehav1;
   //----------------
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvAllGo<SrAllEventsGo, SBehav1>, StEventCountDown, ON_SENSORS_AVAILABLE>,
   Transition<EvGlobalError, sc::deep_history<StAcquireSensors>>
   
   >reactions;

// STATE FUNCTIONS
   static void staticConfigure()
   {
      configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
      configure_orthogonal<OrStringPublisher, CbStringPublisher>("Hello World!");
      configure_orthogonal<OrTemperatureSensor, CbConditionTemperatureSensor>();
      configure_orthogonal<OrService3, CbService3>(Service3Command::SERVICE3_ON);
      configure_orthogonal<OrUpdatablePublisher, ros_publisher_client::CbDefaultPublishLoop>();

      // Create State Reactor
      auto srAllSensorsReady = static_createStateReactor<SrAllEventsGo>();
      srAllSensorsReady->addInputEvent<EvTopicMessage<CbLidarSensor, OrObstaclePerception>>();
      srAllSensorsReady->addInputEvent<EvTopicMessage<CbConditionTemperatureSensor, OrTemperatureSensor>>();

      srAllSensorsReady->setOutputEvent<EvAllGo<SrAllEventsGo, SBehav1>>();
   }
};
} // namespace sm_dance_bot