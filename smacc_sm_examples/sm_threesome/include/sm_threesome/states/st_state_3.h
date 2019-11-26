namespace sm_threesome
{
struct StState3 : smacc::SmaccState<StState3, SmThreeSome>
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
} // namespace sm_threesome