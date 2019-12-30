namespace sm_three_some
{
struct StState3 : smacc::SmaccState<StState3, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::Ss1>,

        // Keyboard events
        smacc::transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState2>,
        smacc::transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>>
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