//-------------------------------------------------------------------------------------------
/*! \file    lc_random.h
    \brief   libskyai - low-level controller: random command generator  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.09, 2013

    Copyright (C) 2013  Akihiko Yamaguchi

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
#ifndef skyai_lc_random_h
#define skyai_lc_random_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
template <typename t_command>
class TLCRandomConfigurations
//===========================================================================================
{
public:

  TReal                Interval;    //!< duration of each command

  t_command            Min;  //! the lower bound of a command
  t_command            Max;  //! the upper bound of a command

  TLCRandomConfigurations(var_space::TVariableMap &mmap)
    : Interval(0.0)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Interval    );
      ADD( Min         );
      ADD( Max         );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief low-level controller that generates a random command in every conf_.Interval  */
template <typename t_command>
class MLCRandom
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface      TParent;
  typedef t_command             TCommand;
  typedef MLCRandom<t_command>  TThis;
  SKYAI_MODULE_NAMES(MLCRandom)

  MLCRandom (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      ltime_         (-1.0l),
      slot_execute            (*this),
      slot_finish             (*this),
      slot_start_time_step    (*this),
      slot_finish_time_step   (*this),
      signal_start_of_command (*this),
      signal_end_of_command   (*this),
      signal_execute_command  (*this)
    {
      this->add_slot_port   (slot_execute            );
      this->add_slot_port   (slot_finish             );
      this->add_slot_port   (slot_start_time_step    );
      this->add_slot_port   (slot_finish_time_step   );
      this->add_signal_port (signal_start_of_command );
      this->add_signal_port (signal_end_of_command   );
      this->add_signal_port (signal_execute_command  );
    }

  virtual ~MLCRandom() {}

protected:

  TLCRandomConfigurations<TCommand>  conf_;

  TContinuousTime  ltime_;  //!< remaining time of the current command
  TCommand         current_command_;
  TBool            active_;

  //!\brief if this slot catch a signal, this module starts to generate a sequence of control commands
  MAKE_SLOT_PORT(slot_execute, void, (), (), TThis);

  /*!\brief if this slot catch a signal, the module is terminated */
  MAKE_SLOT_PORT(slot_finish, void, (void), (), TThis);

  //!\brief executed at each beginning of time-step
  MAKE_SLOT_PORT(slot_start_time_step, void, (const TContinuousTime &time_step), (time_step), TThis);

  //!\brief executed at each end of time-step
  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TContinuousTime &time_step), (time_step), TThis);

  //!\brief this signal is emitted when a new control command is started
  MAKE_SIGNAL_PORT(signal_start_of_command, void(void), TThis);

  //!\brief this signal is emitted when the current control command is finished
  MAKE_SIGNAL_PORT(signal_end_of_command, void(void), TThis);

  //!\brief this signal is emitted to request to execute the control command
  MAKE_SIGNAL_PORT(signal_execute_command, void(const TCommand&), TThis);

  virtual void slot_execute_exec (void);
  virtual void slot_finish_exec (void);
  virtual void slot_start_time_step_exec (const TContinuousTime &time_step);
  virtual void slot_finish_time_step_exec (const TContinuousTime &time_step);

};  // end of MLCRandom
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLCRandom,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MLCRandom,TRealVector)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_lc_random_h
//-------------------------------------------------------------------------------------------
