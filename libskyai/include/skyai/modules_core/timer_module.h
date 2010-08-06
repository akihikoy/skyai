//-------------------------------------------------------------------------------------------
/*! \file    timer_module.h
    \brief   libskyai - define the timer-modules that emits signals at each time step
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.30, 2009-

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
#ifndef skyai_timer_module_h
#define skyai_timer_module_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TMUserEmittedTimerConfigurations
//===========================================================================================
{
public:

  TContinuousTime           Cycle1;  //! user defined cycle. if negative, signal is not emitted. [0,dt) is error.
  TContinuousTime           Cycle2;  //! user defined cycle. if negative, signal is not emitted. [0,dt) is error.
  TContinuousTime           Cycle3;  //! user defined cycle. if negative, signal is not emitted. [0,dt) is error.

  TMUserEmittedTimerConfigurations (var_space::TVariableMap &mmap)
    :
      Cycle1  (-1.0l),
      Cycle2  (-1.0l),
      Cycle3  (-1.0l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Cycle1  );
      ADD( Cycle2  );
      ADD( Cycle3  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief User defined timer-module
    \note This timer is controlled by slot_start_step, slot_finish_step
    \note slot_start_step and slot_finish_step catch signals from an enviromnent module,
          such as a simulator module.
    \note This module can emit signal with three user-defined cycles (Cycle{1,2,3})
*/
class MUserEmittedTimer : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface  TParent;
  typedef MUserEmittedTimer TThis;
  SKYAI_MODULE_NAMES(MUserEmittedTimer)

  MUserEmittedTimer (const std::string &v_instance_name)
    : TParent       (v_instance_name),
      conf_         (TParent::param_box_config_map()),
      cont_time_    (INVALID_CONT_TIME),
      disc_time_    (INVALID_DISC_TIME),

      using_ud1_    (false),
      using_ud2_    (false),
      using_ud3_    (false),

      slot_reset               (*this),
      slot_start_step          (*this),
      slot_finish_step         (*this),
      signal_start_of_step     (*this),
      signal_end_of_step       (*this),
      signal_start_of_step_ud1 (*this),
      signal_end_of_step_ud1   (*this),
      signal_start_of_step_ud2 (*this),
      signal_end_of_step_ud2   (*this),
      signal_start_of_step_ud3 (*this),
      signal_end_of_step_ud3   (*this),
      out_cont_time            (*this),
      out_disc_time            (*this)
    {
      add_slot_port (slot_reset);
      add_slot_port (slot_start_step);
      add_slot_port (slot_finish_step);
      add_signal_port (signal_start_of_step );
      add_signal_port (signal_end_of_step   );
      add_signal_port (signal_start_of_step_ud1 );
      add_signal_port (signal_end_of_step_ud1   );
      add_signal_port (signal_start_of_step_ud2 );
      add_signal_port (signal_end_of_step_ud2   );
      add_signal_port (signal_start_of_step_ud3 );
      add_signal_port (signal_end_of_step_ud3   );
      add_out_port (out_cont_time);
      add_out_port (out_disc_time);
    }

  virtual ~MUserEmittedTimer() {}

protected:

  TMUserEmittedTimerConfigurations conf_;

  TContinuousTime cont_time_;
  TDiscreteTime   disc_time_;

  TBool           using_ud1_;
  TBool           using_ud2_;
  TBool           using_ud3_;
  TContinuousTime ud_cont_time1_;
  TContinuousTime ud_cont_time2_;
  TContinuousTime ud_cont_time3_;
  TContinuousTime ud_actual_time_step1_;
  TContinuousTime ud_actual_time_step2_;
  TContinuousTime ud_actual_time_step3_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT_SPECIFIC(slot_start_step, void, (const TContinuousTime &dt), (dt), TThis, SKYAI_CONNECTION_SIZE_MAX, SKYAI_DISABLED_FWD_PORT_NAME);

  MAKE_SLOT_PORT_SPECIFIC(slot_finish_step, void, (const TContinuousTime &dt), (dt), TThis, SKYAI_CONNECTION_SIZE_MAX, SKYAI_DISABLED_FWD_PORT_NAME);

  MAKE_SIGNAL_PORT(signal_start_of_step, void (const TContinuousTime &dt), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_step, void (const TContinuousTime &dt), TThis);

  MAKE_SIGNAL_PORT(signal_start_of_step_ud1, void (const TContinuousTime &dt), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_step_ud1, void (const TContinuousTime &dt), TThis);

  MAKE_SIGNAL_PORT(signal_start_of_step_ud2, void (const TContinuousTime &dt), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_step_ud2, void (const TContinuousTime &dt), TThis);

  MAKE_SIGNAL_PORT(signal_start_of_step_ud3, void (const TContinuousTime &dt), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_step_ud3, void (const TContinuousTime &dt), TThis);

  MAKE_OUT_PORT(out_cont_time, const TContinuousTime&, (void), (), TThis);

  MAKE_OUT_PORT(out_disc_time, const TDiscreteTime&, (void), (), TThis);


  virtual void slot_reset_exec (void)
    {
      cont_time_= 0.0l;
      disc_time_= 0;
      #define RESET_CYCLE(x_idx) \
          using_ud##x_idx##_= (conf_.Cycle##x_idx >= 0.0l);  \
          ud_cont_time##x_idx##_= INVALID_CONT_TIME;
      RESET_CYCLE(1)
      RESET_CYCLE(2)
      RESET_CYCLE(3)
      #undef RESET_CYCLE
    }

  virtual void slot_start_step_exec (const TContinuousTime &dt)
    {
      // LDEBUG(InstanceName()<<"->slot_start_step is activated at "<<cont_time_);
      if (ModuleMode()==TModuleInterface::mmDebug)
        {DebugStream()<<"TIMER(slot_start_step: "<<&slot_start_step<<"): "<<cont_time_<<"[s] ("<<disc_time_<<"[steps])"<<std::endl;}
      signal_start_of_step.ExecAll(dt);

      #define SIGNAL_START_OF_STEP_UD(x_idx)             \
        if (using_ud##x_idx##_)                          \
        {                                                \
          if (ud_cont_time##x_idx##_<0.0l)               \
          {                                              \
            ud_actual_time_step##x_idx##_= 0.0l;         \
            ud_cont_time##x_idx##_= conf_.Cycle##x_idx;  \
            signal_start_of_step_ud##x_idx.ExecAll(conf_.Cycle##x_idx);  \
          }                                              \
        }
      SIGNAL_START_OF_STEP_UD(1)
      SIGNAL_START_OF_STEP_UD(2)
      SIGNAL_START_OF_STEP_UD(3)
      #undef SIGNAL_START_OF_STEP_UD
    }
  virtual void slot_finish_step_exec (const TContinuousTime &dt)
    {
      cont_time_+= dt;
      disc_time_++;
      // LDEBUG(InstanceName()<<"->slot_finish_step is activated at "<<cont_time_);
      if (ModuleMode()==TModuleInterface::mmDebug)
        {DebugStream()<<"TIMER(slot_finish_step: "<<&slot_finish_step<<"): "<<cont_time_<<"[s] ("<<disc_time_<<"[steps])"<<std::endl;}
      signal_end_of_step.ExecAll(dt);

      #define SIGNAL_END_OF_STEP_UD(x_idx)              \
        if (using_ud##x_idx##_)                         \
        {                                               \
          ud_cont_time##x_idx##_-=dt;                   \
          ud_actual_time_step##x_idx##_+=dt;            \
          if (ud_cont_time##x_idx##_<CONT_TIME_TOL)     \
          {                                             \
            ud_cont_time##x_idx##_= INVALID_CONT_TIME;  \
            signal_end_of_step_ud##x_idx.ExecAll(ud_actual_time_step##x_idx##_);   \
          }                                             \
        }
      SIGNAL_END_OF_STEP_UD(1)
      SIGNAL_END_OF_STEP_UD(2)
      SIGNAL_END_OF_STEP_UD(3)
      #undef SIGNAL_END_OF_STEP_UD
    }

  virtual const TContinuousTime& out_cont_time_get (void) const
    {
      return cont_time_;
    }
  virtual const TDiscreteTime& out_disc_time_get (void) const
    {
      return disc_time_;
    }
};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_timer_module_h
//-------------------------------------------------------------------------------------------
