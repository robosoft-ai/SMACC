#pragma once
#include <map>
#include <typeinfo>

class AllEventAggregator
{
    int event_types_count;
    
    public:
    void setTriggerEventTypesCount(int event_types_count)
    {
      this->event_types_count = event_types_count;
    }

    std::map<const char*,bool> events;

    template<typename T>
    bool notify()
    {
      auto* name = typeid(T).name();
      events[name] = true;

      return isTriggered();
    }

    template<typename T>
    bool notify(const T& ev)
    {
      return this->notify<T>();
    }

    bool isTriggered()
    {
      return events.size() >= event_types_count;
    }
};