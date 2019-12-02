namespace sm_three_some
{
struct StState2 : smacc::SmaccState<StState2, SmThreeSome>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
                    smacc::transition<EvAll<LuAllEventsGo>, StState3>,
                    
                    // Keyboard events
                    smacc::transition<EvKeyPressP<SbKeyboard>, StState1>,
                    smacc::transition<EvKeyPressN<SbKeyboard>, StState3>
        >
        reactions;

    static void onDefinition()
    {
        static_configure<Orthogonal1, SbBehavior1b>();
        static_configure<Orthogonal2, SbBehavior2b>();
        static_configure<KeyboardOrthogonal, SbKeyboard>();

        static_createLogicUnit<LuAllEventsGo, EvAll<LuAllEventsGo>, mpl::list<EvTopicMessage<SbBehavior1b>, EvTopicMessage<Client1>>>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some