namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<EvAll<LuAllEventsGo>, StState3>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbKeyboard, KeyboardOrthogonal>, StState1>,
        smacc::transition<EvKeyPressN<CbKeyboard, KeyboardOrthogonal>, StState3>>
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, CbBehavior1b>();
        static_configure<Orthogonal2, CbBehavior2b>();
        static_configure<KeyboardOrthogonal, CbKeyboard>();

        static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo>, mpl::list<EvTopicMessage<CbBehavior1b, Orthogonal1>, EvTopicMessage<Client1, Orthogonal1>>>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some