#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_reactor.h>
#include <smacc/smacc_updatable.h>
#include <typeinfo>
#include <boost/statechart/event.hpp>
#include <functional>

namespace smacc
{

namespace state_reactors
{
template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct EventA : sc::event<EventA<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct EventB : sc::event<EventB<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct EventC : sc::event<EventC<TSource, TObjectTag>>
{
};

enum class RandomGenerateReactorMode
{
    INPUT_EVENT_TRIGGERED,
    ONE_SHOT,
    ON_UPDATE
};

//-----------------------------------------------------------------------
class SrRandomGenerator : public StateReactor, public ISmaccUpdatable
{
public:
    SrRandomGenerator(RandomGenerateReactorMode mode, double evAMin = 1, double evAMax = 4, double evBMin = 5, double evBMax = 8, double evCMin = 9, double evCMax = 12);

    virtual void onEntry() override;

    template <typename TObjectTag>
    void declareObjectTag()
    {
        this->postEventA = [this]() { this->postEvent<EventA<SrRandomGenerator, TObjectTag>>(); };
        this->postEventB = [this]() { this->postEvent<EventB<SrRandomGenerator, TObjectTag>>(); };
        this->postEventC = [this]() { this->postEvent<EventC<SrRandomGenerator, TObjectTag>>(); };
    }

    void postRandomEvents();

    virtual void update() override;

    virtual void onEventNotified(const std::type_info *eventType) override;

    RandomGenerateReactorMode mode_;

private:
    std::function<void()> postEventA;
    std::function<void()> postEventB;
    std::function<void()> postEventC;

    double evAMin_;
    double evAMax_;
    double evBMin_;
    double evBMax_;
    double evCMin_;
    double evCMax_;

    double minValue;
    double maxValue;
};
} // namespace state_reactors
} // namespace smacc