namespace sm_three_some
{
struct StState3 : smacc::SmaccState<StState3, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<smacc::EvTopicMessage<CbBehavior2b, Orthogonal2>, SS1::Ss1>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbKeyboard, KeyboardOrthogonal>, StState2>,
        smacc::transition<EvKeyPressN<CbKeyboard, KeyboardOrthogonal>, SS1::Ss1>>
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, CbBehavior1b>();
        static_configure<Orthogonal2, CbBehavior2b>();
        static_configure<KeyboardOrthogonal, CbKeyboard>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some