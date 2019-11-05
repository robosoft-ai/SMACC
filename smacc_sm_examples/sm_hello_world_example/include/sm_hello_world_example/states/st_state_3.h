namespace hello_world_example
{
struct StState3 : smacc::SmaccState<StState3, SmHelloWorld>
{
    using SmaccState::SmaccState;

    typedef smacc::transition<smacc::EvTopicMessage<SbBehavior2b>, StState2> reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1b>();
        static_configure<Orthogonal2, SbBehavior2b>();
    }

    void onInitialize()
    {
    }
};
} // namespace hello_world_example