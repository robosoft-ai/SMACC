namespace hello_world_example
{
struct StState2 : smacc::SmaccState<StState2, SmHelloWorld>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<smacc::EvActionSucceded<Client2>, hello_world_example::SS1::Ss1>,
        smacc::transition<EvAll<LuAllEventsGo>, StState1>>
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1b>();
        static_configure<Orthogonal2, SbBehavior2b>();

        static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo>, EvTopicMessage<SbBehavior1b>, EvTopicMessage<Client1>>();
    }

    void onInitialize()
    {
    }
};
} // namespace hello_world_example