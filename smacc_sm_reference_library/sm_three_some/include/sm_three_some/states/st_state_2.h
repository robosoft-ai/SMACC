namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<EvAll<LuAllEventsGo>, StState3>,

        // Keyboard events
        smacc::transition<EvKeyPressP<SbKeyboard, KeyboardOrthogonal>, StState1>,
        smacc::transition<EvKeyPressN<SbKeyboard, KeyboardOrthogonal>, StState3>>
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1b>();
        static_configure<Orthogonal2, SbBehavior2b>();
        static_configure<KeyboardOrthogonal, SbKeyboard>();

        static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo>, mpl::list<EvTopicMessage<SbBehavior1b, Orthogonal1>, EvTopicMessage<Client1, Orthogonal1>>>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some