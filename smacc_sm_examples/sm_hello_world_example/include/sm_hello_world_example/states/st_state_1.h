using namespace smacc;

namespace hello_world_example
{
// ---- TAGS ----
struct EVENT_TAG{};
//----------------

struct StState1 : smacc::SmaccState<StState1, SmHelloWorld>
{
   using SmaccState::SmaccState;

   typedef smacc::transition<smacc::EvTopicMessage<Client1>, StState2, EVENT_TAG>
       reactions;

   static void onDefinition()
   {
     
   }

   void onInitialize()
   {
   }
};
}