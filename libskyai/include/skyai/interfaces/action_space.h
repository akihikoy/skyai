//-------------------------------------------------------------------------------------------
/*! \file    action_space.h
    \brief   libskyai - interface of action space modules
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.22, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

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
#ifndef skyai_action_space_h
#define skyai_action_space_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief interface of a standard action space which convert an input action to a sequence of control command
template <typename t_action, typename t_command>
class MActionSpaceInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MActionSpaceInterface    TThis;
  typedef t_action                 TAction;
  typedef t_command                TCommand;
  SKYAI_MODULE_NAMES(MActionSpaceInterface)

  MActionSpaceInterface (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_initialize         (*this),
      slot_reset              (*this),
      slot_execute_action     (*this),
      slot_start_time_step    (*this),
      slot_finish_time_step   (*this),
      signal_end_of_action    (*this),
      signal_execute_command  (*this)
    {
      add_slot_port   (slot_initialize         );
      add_slot_port   (slot_reset              );
      add_slot_port   (slot_execute_action     );
      add_slot_port   (slot_start_time_step    );
      add_slot_port   (slot_finish_time_step   );
      add_signal_port (signal_end_of_action    );
      add_signal_port (signal_execute_command  );
    }

  virtual ~MActionSpaceInterface() {}

protected:

  //!\brief if this slot catch a signal, this module is initialized
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  //!\brief if this slot catch a signal, this module is reset
  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  //!\brief if this slot catch a signal, this module starts to generate a sequence of control command according to `a'
  MAKE_SLOT_PORT(slot_execute_action, void, (const TAction &a), (a), TThis);

  //!\brief if this slot catch a signal, current control command is calculated and sent
  MAKE_SLOT_PORT(slot_start_time_step, void, (const TContinuousTime &time_step), (time_step), TThis);

  //!\brief if this slot catch a signal, current control command should be terminated
  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TContinuousTime &time_step), (time_step), TThis);


  //!\brief this signal is emitted when the current action is finished
  MAKE_SIGNAL_PORT(signal_end_of_action, void(void), TThis);

  //!\brief this signal is emitted to request to execute the control command
  MAKE_SIGNAL_PORT(signal_execute_command, void(const TCommand&), TThis);


  virtual void slot_initialize_exec (void) = 0;
  virtual void slot_reset_exec (void) = 0;
  virtual void slot_execute_action_exec (const TAction &a) = 0;
  virtual void slot_start_time_step_exec (const TContinuousTime &time_step) = 0;
  virtual void slot_finish_time_step_exec (const TContinuousTime &time_step) = 0;

};  // end of MActionSpaceInterface
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief interface of a discrete action space
template <typename t_command>
class MDiscreteActionSpaceInterface
    : public MActionSpaceInterface <TDiscreteAction, t_command>
//===========================================================================================
{
public:
  typedef MActionSpaceInterface
              <TDiscreteAction, t_command>   TParent;
  typedef MDiscreteActionSpaceInterface      TThis;
  SKYAI_MODULE_NAMES(MDiscreteActionSpaceInterface)

  MDiscreteActionSpaceInterface (const std::string &v_instance_name)
    : TParent              (v_instance_name),
      out_action_set_size  (*this)
    {
      add_out_port (out_action_set_size);
    }

  virtual ~MDiscreteActionSpaceInterface() {}

protected:

  MAKE_OUT_PORT(out_action_set_size, const TInt&, (void), (), TThis);

  virtual const TInt& out_action_set_size_get (void) const = 0;

};  // end of MDiscreteActionSpaceInterface
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_action_space_h
//-------------------------------------------------------------------------------------------
