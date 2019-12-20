namespace sm_three_some
{
struct StState3 : smacc::SmaccState<StState3, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<smacc::EvTopicMessage<CbBehavior2b, OrOrthogonal2>, SS1::Ss1>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbKeyboard, OrKeyboard>, StState2>,
        smacc::transition<EvKeyPressN<CbKeyboard, OrKeyboard>, SS1::Ss1>>
        reactions;

    static void onDefinition()
    {
        static_configure<OrOrthogonal1, CbBehavior1b>();
        static_configure<OrOrthogonal2, CbBehavior2b>();
        static_configure<OrKeyboard, CbKeyboard>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some