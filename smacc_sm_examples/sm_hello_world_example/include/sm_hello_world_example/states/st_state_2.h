using namespace smacc;

namespace hello_world_example
{
// ---- TAGS ----
struct MY_EVENT{};
//----------------

struct StState2 : smacc::SmaccState<StState2, SmHelloWorld>
{
   using SmaccState::SmaccState;

   typedef smacc::transition<smacc::EvTopicMessage<Client2>, StState1, MY_EVENT>
       reactions;
       
   static void onDefinition()
   {
     
   }

   void onInitialize()
   {
   }
};
}