namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<EvAll<LuAllEventsGo>, StState3>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbKeyboard, OrKeyboard>, StState1>,
        smacc::transition<EvKeyPressN<CbKeyboard, OrKeyboard>, StState3>>
        reactions;

    static void onDefinition()
    {
        static_configure<OrOrthogonal1, CbBehavior1b>();
        static_configure<OrOrthogonal2, CbBehavior2b>();
        static_configure<OrKeyboard, CbKeyboard>();

        static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo>, mpl::list<EvTopicMessage<CbBehavior1b, OrOrthogonal1>, EvTopicMessage<ClClient1, OrOrthogonal1>>>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some