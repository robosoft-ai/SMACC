/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include "smacc/smacc_state.h"

namespace smacc
{
    
template< class MostDerived,
          class Context,
          class InnerInitial = mpl::list<>,
          sc::history_mode historyMode = sc::has_no_history>
class SmaccState : public smacc::SmaccState<
  MostDerived, Context, InnerInitial, historyMode >
  {
typedef sc::simple_state< MostDerived, Context, InnerInitial, historyMode >
    base_type;

  public:
    using smacc::SmaccState::nh;

    using smacc::SmaccState::SmaccState;
  }

}