namespace sm_three_some
{
struct StState1 : smacc::SmaccState<StState1, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected transition
        smacc::transition<smacc::EvTopicMessage<ClClient1, OrOrthogonal1>, StState2>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbKeyboard, OrKeyboard>, SS1::Ss1>,
        smacc::transition<EvKeyPressN<CbKeyboard, OrKeyboard>, StState2>>
        reactions;

    static void onDefinition()
    {
        static_configure<OrOrthogonal1, CbBehavior1>();
        static_configure<OrOrthogonal2, CbBehavior2>();
        static_configure<OrKeyboard, CbKeyboard>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some