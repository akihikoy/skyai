//-------------------------------------------------------------------------------------------
/*! \file    univ_task.h
    \brief   libskyai - universal task modules (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.31, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#ifndef skyai_univ_task_h
#define skyai_univ_task_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TUniversalContTimeTaskConfigurations
//===========================================================================================
{
public:

  //! Decide if sensing the robot state at each event; default is true; setting false reduces the execution time
  TBool SensingAtEpisodeStart   ;
  TBool SensingAtEpisodeEnd     ;  //!< See SensingAtEpisodeStart
  TBool SensingAtActionStart    ;  //!< See SensingAtEpisodeStart
  TBool SensingAtActionEnd      ;  //!< See SensingAtEpisodeStart
  TBool SensingAtTimeStepStart  ;  //!< See SensingAtEpisodeStart
  TBool SensingAtTimeStepEnd    ;  //!< See SensingAtEpisodeStart

  /*! User-defined functions; before calling each function,
      memory.Reward is set to be 0 and memory
      memory.EndOfEps is set to be false.
      these functions should have an argument storing the task module's id and no return.

      \note If no ports are connected to neither signal_end_of_episode nor signal_reward,
         these functions are not executed. */
  TString FEpisodeStart   ;
  TString FEpisodeEnd     ;  //!< See FEpisodeStart
  TString FActionStart    ;  //!< See FEpisodeStart
  TString FActionEnd      ;  //!< See FEpisodeStart
  TString FTimeStepStart  ;  //!< See FEpisodeStart
  TString FTimeStepEnd    ;  //!< See FEpisodeStart

  //! constants used in user-defined functions
  TInt          CI1,CI2;
  TReal         CR1,CR2,CR3,CR4;
  TBool         CB1,CB2;
  TRealVector   CRV1,CRV2;


  TUniversalContTimeTaskConfigurations (var_space::TVariableMap &mmap) :
      SensingAtEpisodeStart   (true),
      SensingAtEpisodeEnd     (true),
      SensingAtActionStart    (true),
      SensingAtActionEnd      (true),
      SensingAtTimeStepStart  (true),
      SensingAtTimeStepEnd    (true),
      CI1   (0),
      CI2   (0),
      CR1   (0.0),
      CR2   (0.0),
      CR3   (0.0),
      CR4   (0.0),
      CB1   (false),
      CB2   (false)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( SensingAtEpisodeStart  );
      ADD( SensingAtEpisodeEnd    );
      ADD( SensingAtActionStart   );
      ADD( SensingAtActionEnd     );
      ADD( SensingAtTimeStepStart );
      ADD( SensingAtTimeStepEnd   );
      ADD( FEpisodeStart  );
      ADD( FEpisodeEnd    );
      ADD( FActionStart   );
      ADD( FActionEnd     );
      ADD( FTimeStepStart );
      ADD( FTimeStepEnd   );
      ADD( CI1 );
      ADD( CI2 );
      ADD( CR1 );
      ADD( CR2 );
      ADD( CR3 );
      ADD( CR4 );
      ADD( CB1 );
      ADD( CB2 );
      ADD( CRV1 );
      ADD( CRV2 );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TUniversalContTimeTaskMemory
//===========================================================================================
{
public:

  TReal        TimeStep;

  //! variable to store the reward; user-defined functions should assign to this variable
  TReal        Reward;
  //! variable to store the end-of-episode condition; user-defined functions should assign to this variable
  TBool        EndOfEps;

  //! temporary variables for user-defined functions
  TInt         TmpI1,TmpI2;
  TReal        TmpR1,TmpR2;
  TBool        TmpB1,TmpB2;
  TRealVector  TmpRV1,TmpRV2;

  TUniversalContTimeTaskMemory (var_space::TVariableMap &mmap)
    :
      TimeStep  (0.0l),
      Reward    (0.0l),
      EndOfEps  (false),
      TmpI1     (0),
      TmpI2     (0),
      TmpR1     (0.0l),
      TmpR2     (0.0l),
      TmpB1     (false),
      TmpB2     (false)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( TimeStep          );
      ADD( Reward            );
      ADD( EndOfEps          );
      ADD( TmpI1             );
      ADD( TmpI2             );
      ADD( TmpR1             );
      ADD( TmpR2             );
      ADD( TmpB1             );
      ADD( TmpB2             );
      ADD( TmpRV1            );
      ADD( TmpRV2            );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief universal continuous-time task module
class MUniversalContTimeTask
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface        TParent;
  typedef MUniversalContTimeTask  TThis;
  SKYAI_MODULE_NAMES(MUniversalContTimeTask)

  MUniversalContTimeTask (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      mem_           (TParent::param_box_memory_map()),
      slot_start_episode     (*this),
      slot_finish_episode    (*this),
      slot_start_of_action   (*this),
      slot_end_of_action     (*this),
      slot_start_time_step   (*this),
      slot_finish_time_step  (*this),
      signal_reward          (*this),
      signal_end_of_episode  (*this)
    {
      add_slot_port   (slot_start_episode     );
      add_slot_port   (slot_finish_episode    );
      add_slot_port   (slot_start_of_action   );
      add_slot_port   (slot_end_of_action     );
      add_slot_port   (slot_start_time_step   );
      add_slot_port   (slot_finish_time_step  );
      add_signal_port (signal_reward          );
      add_signal_port (signal_end_of_episode  );
    }

protected:

  TUniversalContTimeTaskConfigurations  conf_;
  TUniversalContTimeTaskMemory          mem_;

  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_finish_episode, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_start_of_action, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_end_of_action, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_start_time_step, void, (const TReal &dt), (dt), TThis);
  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TReal &dt), (dt), TThis);

  MAKE_SIGNAL_PORT(signal_reward, void (const TSingleReward &), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  //! Return true if this module is in use
  bool is_used(void)
    {
      return (signal_end_of_episode.ConnectionSize()>0) || (signal_reward.ConnectionSize()>0);
    }

  virtual void slot_start_episode_exec (void);
  virtual void slot_finish_episode_exec (void);

  virtual void slot_start_of_action_exec (void);
  virtual void slot_end_of_action_exec (void);

  virtual void slot_start_time_step_exec (const TReal &dt);
  virtual void slot_finish_time_step_exec (const TReal &dt);

  virtual void sense_common()
    {
    }

  virtual void sense_at_EpisodeStart  ()  {sense_common();}
  virtual void sense_at_EpisodeEnd    ()  {sense_common();}
  virtual void sense_at_ActionStart   ()  {sense_common();}
  virtual void sense_at_ActionEnd     ()  {sense_common();}
  virtual void sense_at_TimeStepStart ()  {sense_common();}
  virtual void sense_at_TimeStepEnd   ()  {sense_common();}

};  // end of MUniversalContTimeTask
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_univ_task_h
//-------------------------------------------------------------------------------------------
