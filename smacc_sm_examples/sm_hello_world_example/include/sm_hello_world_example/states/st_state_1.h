namespace hello_world_example
{
struct StState1 : smacc::SmaccState<StState1, SmHelloWorld>
{
    using SmaccState::SmaccState;

    typedef mpl::list<smacc::transition<smacc::EvTopicMessage<Client1>, StState2>,
                      smacc::transition<smacc::EvTopicMessage<SbBehavior2>, StState3>>
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1>();
        static_configure<Orthogonal2, SbBehavior2>();
    }

    void onInitialize()
    {
    }
};
} // namespace hello_world_example