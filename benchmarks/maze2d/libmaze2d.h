//-------------------------------------------------------------------------------------------
/*! \file    libmaze2d.h
    \brief   benchmarks - test libskyai on a simple toyproblem: navigation task in 2d-maze
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.04, 2010

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
#ifndef libmaze2d_h
#define libmaze2d_h
//-------------------------------------------------------------------------------------------
#include "detail/maze2d.cpp"
#include <skyai/skyai.h>
#include <skyai/interfaces/action_space.h>
#include <lora/small_classes.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TMazeConfigurations
//===========================================================================================
{
public:

  TRealVector                StartPos;
  TReal                      WindForce1, WindForce2;  //!< force of the wind
  int                        MapKind;  //!< kind of map
  TReal                      TimeStep;

  TReal                      StepCostFactor;
  TReal                      BadStatePenalty;

  TMazeConfigurations (var_space::TVariableMap &mmap) :
      WindForce1          (0.01l),
      WindForce2          (0.08l),
      MapKind             (0),
      TimeStep            (0.01l),
      StepCostFactor      (-25.0l),
      BadStatePenalty     (-0.5l)
    {
      StartPos= CVector2(-0.80, 0.10);

      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( StartPos               );
      ADD( WindForce1             );
      ADD( WindForce2             );
      ADD( MapKind                );
      ADD( TimeStep               );
      ADD( StepCostFactor         );
      ADD( BadStatePenalty        );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief environment module
class MMazeEnvironment
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MMazeEnvironment    TThis;
  SKYAI_MODULE_NAMES(MMazeEnvironment)

  MMazeEnvironment (const std::string &v_instance_name)
    : TParent                     (v_instance_name),
      conf_                       (TParent::param_box_config_map()),
      world_                      (),
      slot_initialize             (*this),
      slot_start_episode          (*this),
      slot_execute_command        (*this),
      slot_step_loop              (*this),
      signal_start_of_timestep    (*this),
      signal_end_of_timestep      (*this),
      signal_system_reward        (*this),
      signal_end_of_episode       (*this),
      out_position                (*this),
      out_situation               (*this)
    {
      add_slot_port   (slot_initialize           );
      add_slot_port   (slot_start_episode        );
      add_slot_port   (slot_execute_command      );
      add_slot_port   (slot_step_loop            );
      add_signal_port (signal_start_of_timestep  );
      add_signal_port (signal_end_of_timestep    );
      add_signal_port (signal_system_reward      );
      add_signal_port (signal_end_of_episode     );
      add_out_port    (out_position              );
      add_out_port    (out_situation             );
    }

  void StepLoop (void)
    {
      slot_step_loop.Exec();
    }

protected:

  TMazeConfigurations conf_;

  TRealWorld          world_;
  TRealVector         current_command_;
  bool                in_bad_state_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  //! u: displacement of the robot
  MAKE_SLOT_PORT(slot_execute_command, void, (const TRealVector &u), (u), TThis);

  MAKE_SLOT_PORT(slot_step_loop, void, (void), (), TThis);


  MAKE_SIGNAL_PORT(signal_start_of_timestep, void (const TContinuousTime &), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_timestep, void (const TContinuousTime &), TThis);

  MAKE_SIGNAL_PORT(signal_system_reward, void (const TSingleReward&), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);


  MAKE_OUT_PORT(out_position, const TContinuousState&, (void), (), TThis);
  MAKE_OUT_PORT(out_situation, const TInt&, (void), (), TThis);


  virtual void slot_initialize_exec (void)
    {
      // current_command_= CVector2(0.0, 0.0);
      in_bad_state_= false;
      world_.MAP_KIND     = conf_.MapKind;
      world_.WIND_FORCE1  = conf_.WindForce1;
      world_.WIND_FORCE2  = conf_.WindForce2;
      world_.Init();
    }

  virtual void slot_start_episode_exec (void)
    {
      in_bad_state_= false;
      world_.Reset (conf_.StartPos);
    }

  virtual void slot_execute_command_exec (const TRealVector &u)
    {
      current_command_= u;
    }

  virtual void slot_step_loop_exec (void)
    {
      signal_start_of_timestep.ExecAll(conf_.TimeStep);

      /*dbg*/if (current_command_.length()==0)  {LERROR("slot_execute_command is not called!!!");}

      ////////////////////////
      TReal step_cost=  world_.Step (conf_.TimeStep, current_command_);
      if (ModuleMode()==TModuleInterface::mmDebug)
        {DebugStream()<<"MAZE(slot_step_loop: "<<&slot_step_loop<<"):  world_.Step is executed (cost="<<step_cost<<")"<<std::endl;}
      ////////////////////////

      TSingleReward reward(0.0l);
      reward+= conf_.StepCostFactor*step_cost;
      if (!in_bad_state_ && world_.IsBadState())
      {
        in_bad_state_= true;
        reward+= conf_.BadStatePenalty;
        signal_end_of_episode.ExecAll();
      }

      signal_system_reward.ExecAll(reward);

      signal_end_of_timestep.ExecAll(conf_.TimeStep);
    }


  virtual const TContinuousState& out_position_get () const
    {
      return world_.State();
    }

  mutable TInt situation_;
  virtual const TInt& out_situation_get () const
    {
      TContinuousState st(world_.State());
      if (st(0)<0.0)  situation_=0;
      if (st(0)>0.0)  situation_=1;
      return situation_;
    }


};  // end of MMazeEnvironment
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TRadialActionSpaceConfigurations
//===========================================================================================
{
public:

  TInt                  NumOfDirs;
  TReal                 Interval;
  TReal                 Speed;

  TRadialActionSpaceConfigurations(var_space::TVariableMap &mmap)
    :
      NumOfDirs             (16),
      Interval              (0.1l),
      Speed                 (0.03l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( NumOfDirs       );
      ADD( Interval        );
      ADD( Speed           );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief radial action space (discrete set)
class MRadialActionSpace
    : public MDiscreteActionSpaceInterface <TRealVector>
//===========================================================================================
{
public:
  typedef MDiscreteActionSpaceInterface <TRealVector>  TParent;
  typedef MRadialActionSpace                           TThis;
  SKYAI_MODULE_NAMES(MRadialActionSpace)

  MRadialActionSpace (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      COMMAND_SIZE_  (2),
      ltime_         (-1.0l)
    {
    }

  virtual ~MRadialActionSpace() {}

protected:

  TRadialActionSpaceConfigurations  conf_;

  const TInt       COMMAND_SIZE_;
  TReal            ltime_;  //!< if positive, an action is active
  TRealVector      current_command_;

  override void slot_initialize_exec (void)
    {
      ltime_= -1.0l;
      current_command_.resize(COMMAND_SIZE_,0.0);
    }
  override void slot_reset_exec (void)
    {
    }
  override void slot_execute_action_exec (const TAction &a)
    {
      TReal dir= static_cast<TReal>(a)/static_cast<TReal>(conf_.NumOfDirs)*REAL_2PI;
      current_command_(0)= -conf_.Speed*real_sin(dir);
      current_command_(1)= conf_.Speed*real_cos(dir);
      ltime_= conf_.Interval;
    }
  override void slot_start_time_step_exec (const TContinuousTime &time_step)
    {
      if (ltime_<=0.0l)  return;
      signal_execute_command.ExecAll(current_command_);
    }
  override void slot_finish_time_step_exec (const TContinuousTime &time_step)
    {
      if (ltime_<=0.0l)  return;
      ltime_-= time_step;
      if (ltime_<=0.0l)
      {
        signal_end_of_action.ExecAll();
      }
    }

  override const TInt& out_action_set_size_get (void) const  {return conf_.NumOfDirs;}

};  // end of MRadialActionSpace
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief radial action space
    \todo conbine with MRadialActionSpace */
class MRadialActionSpace2
    : public MActionSpaceInterface <TContinuousAction, TRealVector>
//===========================================================================================
{
public:
  typedef MActionSpaceInterface <
                  TContinuousAction,
                  TRealVector>          TParent;
  typedef MRadialActionSpace2           TThis;
  SKYAI_MODULE_NAMES(MRadialActionSpace2)

  MRadialActionSpace2 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      ACTION_SIZE_   (3),
      COMMAND_SIZE_  (2),
      ltime_         (-1.0l)
    {
    }

  virtual ~MRadialActionSpace2() {}

protected:

  const TInt       ACTION_SIZE_;
  const TInt       COMMAND_SIZE_;
  TReal            ltime_;  //!< if positive, an action is active
  TRealVector      current_command_;

  override void slot_initialize_exec (void)
    {
      ltime_= -1.0l;
      current_command_.resize(COMMAND_SIZE_,0.0);
    }
  override void slot_reset_exec (void)
    {
    }
  override void slot_execute_action_exec (const TAction &a)
    {
      if (a.length()!=ACTION_SIZE_)
        {LERROR("invalid action input: "<<a.transpose()); lexit(df);}
      const double &dir= a(0);
      const double &speed= a(1);
      current_command_(0)= -speed*real_sin(dir);
      current_command_(1)= speed*real_cos(dir);
      ltime_= a(2);
    }
  override void slot_start_time_step_exec (const TContinuousTime &time_step)
    {
      if (ltime_<=0.0l)  return;
      signal_execute_command.ExecAll(current_command_);
    }
  override void slot_finish_time_step_exec (const TContinuousTime &time_step)
    {
      if (ltime_<=0.0l)  return;
      ltime_-= time_step;
      if (ltime_<=0.0l)
      {
        signal_end_of_action.ExecAll();
      }
    }

};  // end of MRadialActionSpace2
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TConstActionSpaceConfigurations
//===========================================================================================
{
public:

  TRealVector           CommandGain;

  TConstActionSpaceConfigurations(var_space::TVariableMap &mmap)
    {
      CommandGain= OctGen1<TRealVector> (2, 0.03, 0.03);

      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( CommandGain      );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief const action space */
class MConstActionSpace
    : public MActionSpaceInterface <TContinuousAction, TRealVector>
//===========================================================================================
{
public:
  typedef MActionSpaceInterface <
                  TContinuousAction,
                  TRealVector>          TParent;
  typedef MConstActionSpace             TThis;
  SKYAI_MODULE_NAMES(MConstActionSpace)

  MConstActionSpace (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      ACTION_SIZE_   (3),
      COMMAND_SIZE_  (2),
      ltime_         (-1.0l)
    {
    }

  virtual ~MConstActionSpace() {}

protected:

  TConstActionSpaceConfigurations conf_;

  const TInt       ACTION_SIZE_;
  const TInt       COMMAND_SIZE_;
  TReal            ltime_;  //!< if positive, an action is active
  TRealVector      current_command_;

  override void slot_initialize_exec (void)
    {
      ltime_= -1.0l;
      current_command_.resize(COMMAND_SIZE_,0.0);
    }
  override void slot_reset_exec (void)
    {
    }
  override void slot_execute_action_exec (const TAction &a)
    {
      if (a.length()!=ACTION_SIZE_)
        {LERROR("invalid action input: "<<a.transpose()); lexit(df);}
      current_command_(0)= a(0) * conf_.CommandGain(0);
      current_command_(1)= a(1) * conf_.CommandGain(1);
      ltime_= a(2);
      // ltime_= 0.1l;
    }
  override void slot_start_time_step_exec (const TContinuousTime &time_step)
    {
      if (ltime_<=0.0l)  return;
      signal_execute_command.ExecAll(current_command_);
    }
  override void slot_finish_time_step_exec (const TContinuousTime &time_step)
    {
      if (ltime_<=0.0l)  return;
      ltime_-= time_step;
      if (ltime_<=0.0l)
      {
        signal_end_of_action.ExecAll();
      }
    }

};  // end of MConstActionSpace
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // libmaze2d_h
//-------------------------------------------------------------------------------------------
