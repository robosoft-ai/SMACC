namespace sm_three_some
{
struct StState1 : smacc::SmaccState<StState1, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<smacc::transition<smacc::EvTopicMessage<Client1>, StState2>

                    //   // Keyboard events
                    //   smacc::transition<EvKeyPressP<SbKeyboard>, StRotateDegrees4>,
                    //   smacc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>,
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