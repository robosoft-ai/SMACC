namespace sm_three_some
{
struct StState1 : smacc::SmaccState<StState1, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
                    // Expected transition
                    smacc::transition<smacc::EvTopicMessage<Client1>, StState2>,

                    // Keyboard events
                    smacc::transition<EvKeyPressP<SbKeyboard>, SS1::Ss1>,
                    smacc::transition<EvKeyPressN<SbKeyboard>, StState2>
                      >
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1>();
        static_configure<Orthogonal2, SbBehavior2>();
        static_configure<KeyboardOrthogonal, SbKeyboard>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some