namespace sm_three_some
{
struct StState3 : smacc::SmaccState<StState3, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef smacc::transition<smacc::EvTopicMessage<SbBehavior2b>, StState2> reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1b>();
        static_configure<Orthogonal2, SbBehavior2b>();
        static_configure<KeyboardOrthogonal, SbKeyboard>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some