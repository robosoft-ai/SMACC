namespace sm_three_some
{
struct StState3 : smacc::SmaccState<StState3, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
                    smacc::transition<smacc::EvTopicMessage<SbBehavior2b>, SS1::Ss1>,

                    // Keyboard events
                    smacc::transition<EvKeyPressP<SbKeyboard>, StState2>,
                    smacc::transition<EvKeyPressN<SbKeyboard>, SS1::Ss1>
                    > reactions;

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