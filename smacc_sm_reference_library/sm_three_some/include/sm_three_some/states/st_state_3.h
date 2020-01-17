namespace sm_three_some
{
struct StState3 : smacc::SmaccState<StState3, MsRun>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SS1::Ss1>,

        // Keyboard events
        Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState2>,
        Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, SS1::Ss1>>
        reactions;

    static void onDefinition()
    {
        configure_orthogonal<OrTimer, CbTimer>();
        configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
        configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

    void onInitialize()
    {
    }
};
} // namespace sm_three_some