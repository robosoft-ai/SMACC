namespace sm_three_some
{
struct StState1 : smacc::SmaccState<StState1, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected transition
        Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrTimer>, StState2>,

        // Keyboard events
        Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>,
        Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState2>,
        Transition<EvFail, MsRecover, smacc::ABORT>>
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