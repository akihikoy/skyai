//-------------------------------------------------------------------------------------------
/*! \file    lc_holder.h
    \brief   libskyai - low-level controller: holder type  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jan.14, 2010-

    Copyright (C) 2010  Akihiko Yamaguchi

    This file is part of SkyAI.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_lc_holder_h
#define skyai_lc_holder_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/action_space.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TLCHolderConfigurations
//===========================================================================================
{
public:

  TReal                Interval;    //!< time-interval during which an action is executed

  TLCHolderConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Interval    );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief low-level controller which directly outputs the input in conf_.Interval  */
template <typename t_action>
class MLCHolder
    : public MActionSpaceInterface <t_action, t_action>
//===========================================================================================
{
public:
  typedef MActionSpaceInterface
                  <t_action, t_action>   TParent;
  typedef typename TParent::TAction      TAction;
  typedef typename TParent::TCommand     TCommand;
  typedef MLCHolder<t_action>            TThis;
  SKYAI_MODULE_NAMES(MLCHolder)

  MLCHolder (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      ltime_         (-1.0l)
    {
    }

  virtual ~MLCHolder() {}

protected:

  TLCHolderConfigurations  conf_;

  TContinuousTime  ltime_;  //!< if positive, an action is active
  TCommand         current_command_;

  override void slot_initialize_exec (void) {}
  override void slot_reset_exec (void) {}
  override void slot_execute_action_exec (const TAction &a);
  override void slot_start_time_step_exec (const TContinuousTime &time_step);
  override void slot_finish_time_step_exec (const TContinuousTime &time_step);

  using TParent::signal_end_of_action;
  using TParent::signal_execute_command;

};  // end of MLCHolder
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLCHolder,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLCHolder,TRealVector)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_lc_holder_h
//-------------------------------------------------------------------------------------------
