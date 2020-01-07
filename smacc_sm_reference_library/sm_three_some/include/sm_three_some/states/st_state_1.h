namespace sm_three_some
{
struct StState1 : smacc::SmaccState<StState1, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected transition
        smacc::Transition<smacc::EvTopicMessage<ClSubscriber, OrTimer>, StState2>,

        // Keyboard events
        smacc::Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
        smacc::Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState2>,
        smacc::Transition<EvFail, MsRecover, smacc::ABORT>>
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