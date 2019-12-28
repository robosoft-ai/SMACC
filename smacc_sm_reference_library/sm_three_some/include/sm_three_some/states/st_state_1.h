namespace sm_three_some
{
struct StState1 : smacc::SmaccState<StState1, MsThreeSomeRunMode>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected transition
        smacc::transition<smacc::EvTopicMessage<ClSubscriber, OrTimer>, StState2>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
        smacc::transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState2>>
        reactions;

    static void onDefinition()
    {
        static_configure<OrTimer, CbTimer>();
        static_configure<OrSubscriber, CbWatchdogSubscriberBehavior>();
        static_configure<OrUpdatablePublisher, CbDefaultPublishLoop>();
        static_configure<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some