//-------------------------------------------------------------------------------------------
/*! \file    libhumanoid01.h
    \brief   benchmarks - motion learning task of a simulation humanoid robot
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.04, 2010-
    \date    Jun.07, 2012

    Copyright (C) 2009, 2010, 2012  Akihiko Yamaguchi

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
#ifndef libhumanoid01_h
#define libhumanoid01_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <lora/variable_space_impl.h>
#include <lora/ctrl_tools.h>
#include <lora/ode.h>
#include <lora/ode_ds.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class THumanoidEnvironmentConfigurations
//===========================================================================================
{
public:

  TReal                      TimeStep;
  TReal                      FPS;
  TRealVector                ViewPoint;

  bool                       UseInitPose;
  TRealVector                InitBodyPosRotQ;
  TRealVector                InitJointAngles;

  TRealVector                PDGainKp, PDGainKd;  //!< gain parameters for lowlevel PD-controller
  TRealVector                TorqueMax;

  TRealVector                CommandMax, CommandMin;  //!< upper and lower bounds for command

  // parameters for TSimulationCondition simulationcnd
  int                        MaxContactNum;  //!< maximum number of contact points per body
  dSurfaceParameters         Surface;       //!< parameters of contact face
  double                     BodyContactLPFParamF;
  double                     BodyContactLPFParamQ;
  bool                       ForceInitFeetContactWithGround;  //!< true: force to initialize _bodies_contact_with_ground[FEET]=1
  bool                       UsingQuickStep;  //!< use quickStep to step the world. this mode is fast, but sometimes lose the accuracy
  int                        QuickStepIterationNum;  //!< number of iteration used in quickStep

  TReal                      MazeScale  ;
  TInt                       MazeMapKind;


  THumanoidEnvironmentConfigurations (var_space::TVariableMap &mmap);

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( TimeStep                   );
      ADD( FPS                        );
      ADD( ViewPoint                  );
      ADD( UseInitPose                );
      ADD( InitBodyPosRotQ            );
      ADD( InitJointAngles            );

      ADD( PDGainKp                       );
      ADD( PDGainKd                       );
      ADD( TorqueMax                      );

      ADD( CommandMax                     );
      ADD( CommandMin                     );

      // parameters for TSimulationCondition simulationcnd
      ADD( MaxContactNum                              );
      ADD( Surface                                    );
      ADD( BodyContactLPFParamF                       );
      ADD( BodyContactLPFParamQ                       );
      ADD( ForceInitFeetContactWithGround             );
      ADD( UsingQuickStep                             );
      ADD( QuickStepIterationNum                      );

      ADD( MazeScale                                  );
      ADD( MazeMapKind                                );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief environment module
    \todo <b>FIXME: separate this module into robot controller and environment </b> */
class MHumanoidEnvironment
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface      TParent;
  typedef MHumanoidEnvironment  TThis;
  SKYAI_MODULE_NAMES(MHumanoidEnvironment)

  MHumanoidEnvironment (const std::string &v_instance_name);

  void StepLoop (int ds_pause=0);

  void ODEDS_Start();

  void ODEDS_Stop()
    {
      executing_= false;
    }

  void ODEDS_KeyEvent (int cmd);

  bool Executing() const {return executing_;}
  bool ConsoleMode() const {return console_mode_;}
  void SetConsoleMode(bool m)  {console_mode_= m;}

protected:

  THumanoidEnvironmentConfigurations conf_;

  TPDController       PDC_;
  TRealVector         current_command_;
  TRealVector         tq_input_;
//   mutable TContinuousState  tmp_state_;
  mutable TContinuousState  tmp_joint_state_;
  mutable TContinuousState  tmp_base_pose_, tmp_base_vel_, tmp_base_euler_, tmp_joint_angle_, tmp_joint_vel_;
  mutable TRealMatrix       tmp_base_rot_;
  mutable TContinuousState  tmp_base_atan1202_;
  mutable TBoolVector       tmp_contact_with_ground_, tmp_contact_with_object_;

  bool   executing_;
  bool   console_mode_;
  int    repaint_time_;
  TReal  old_fps_;


  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  //! u: desired joint angle (using the controller's constraint mode)
  MAKE_SLOT_PORT(slot_execute_command_des_q, void, (const TRealVector &u), (u), TThis);

  //! u: desired displacement of the joint angle (using the controller's constraint mode)
  MAKE_SLOT_PORT(slot_execute_command_des_qd, void, (const TRealVector &u), (u), TThis);

  MAKE_SLOT_PORT(slot_step_loop, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_finish_loop, void, (void), (), TThis);


  MAKE_SIGNAL_PORT(signal_start_of_timestep, void (const TContinuousTime &), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_timestep, void (const TContinuousTime &), TThis);

  MAKE_SIGNAL_PORT(signal_system_reward, void (const TSingleReward&), TThis);

  //!\brief output base link pose (position and rotation) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_pose, const TContinuousState&, (void), (), TThis);
  //!\brief output base link velocities (of position and rotation) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_vel, const TContinuousState&, (void), (), TThis);

  //!\brief output base rotation matrix[3x3]
  MAKE_OUT_PORT(out_base_rot, const TRealMatrix&, (void), (), TThis);
  //!\brief output base Euler angle \warning maybe GetPitch is incorrect
  MAKE_OUT_PORT(out_base_euler, const TContinuousState&, (void), (), TThis);
  //!\brief output base atan(R12,R02)
  MAKE_OUT_PORT(out_base_atan1202, const TContinuousState&, (void), (), TThis);

  //!\brief output joint angles according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_angle, const TContinuousState&, (void), (), TThis);
  //!\brief output joint angular velocities according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_vel, const TContinuousState&, (void), (), TThis);

  MAKE_OUT_PORT(out_contact_with_ground, const TBoolVector&, (void), (), TThis);
  MAKE_OUT_PORT(out_contact_with_object, const TBoolVector&, (void), (), TThis);

  void set_global_config (void);

  virtual void slot_initialize_exec (void);
  virtual void slot_start_episode_exec (void);

  virtual void slot_execute_command_des_q_exec (const TRealVector &u)
    {
      current_command_= u;
    }
  virtual void slot_execute_command_des_qd_exec (const TRealVector &u);

  virtual void slot_step_loop_exec (void);

  virtual void slot_finish_loop_exec (void)
    {
      executing_= false;
    }

  virtual const TContinuousState& out_base_pose_get() const;
  virtual const TContinuousState& out_base_vel_get() const;
  virtual const TRealMatrix& out_base_rot_get() const;
  virtual const TContinuousState& out_base_euler_get() const;
  virtual const TContinuousState& out_base_atan1202_get() const;
  virtual const TContinuousState& out_joint_angle_get() const;
  virtual const TContinuousState& out_joint_vel_get() const;
  virtual const TBoolVector& out_contact_with_ground_get() const;
  virtual const TBoolVector& out_contact_with_object_get() const;

};  // end of MHumanoidEnvironment
//-------------------------------------------------------------------------------------------


//===========================================================================================
class THumanoidUnivTaskConfigurations
//===========================================================================================
{
public:

  //! decide if sensing the robot state at each event; default is true, false is faster
  TBool SensingAtEpisodeStart   ;
  TBool SensingAtEpisodeEnd     ;
  TBool SensingAtActionStart    ;
  TBool SensingAtActionEnd      ;
  TBool SensingAtTimeStepStart  ;
  TBool SensingAtTimeStepEnd    ;

  /*! user-defined functions; before calling each function,
      memory.Reward is set to be 0 and memory
      memory.EndOfEps is set to be false.
      these functions should have no argument and no return. */
  TString FEpisodeStart   ;
  TString FEpisodeEnd     ;
  TString FActionStart    ;
  TString FActionEnd      ;
  TString FTimeStepStart  ;
  TString FTimeStepEnd    ;

  //! constants used in user-defined functions
  TInt          CI1,CI2;
  TReal         CR1,CR2,CR3,CR4;
  TBool         CB1,CB2;
  TRealVector   CRV1,CRV2;


  THumanoidUnivTaskConfigurations (var_space::TVariableMap &mmap) :
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
class THumanoidUnivTaskMemory
//===========================================================================================
{
public:

  /*! the state of the robot is stored into following variables.
      these variables are assumed to be used in user-defined functions,
      and they are not saved into a file */
  TRealVector  BasePose;
  TRealVector  BaseVel;
  TRealMatrix  BaseRot;
  TBoolVector  ContactWithGround;
  TBoolVector  ContactWithObject;

  //! variable to store the reward; user-defined functions should assign to this variable
  TReal        Reward;
  //! variable to store the end-of-episode condition; user-defined functions should assign to this variable
  TBool        EndOfEps;

  //! temporary variables for user-defined functions
  TInt         TmpI1,TmpI2;
  TReal        TmpR1,TmpR2;
  TBool        TmpB1,TmpB2;
  TRealVector  TmpRV1,TmpRV2;

  THumanoidUnivTaskMemory (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      // ADD( TestC );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief universal task module for humanoid
class MHumanoidUnivTask
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MHumanoidUnivTask    TThis;
  SKYAI_MODULE_NAMES(MHumanoidUnivTask)

  MHumanoidUnivTask (const std::string &v_instance_name)
    : TParent        (v_instance_name),   // mandatory
      conf_          (TParent::param_box_config_map()),
      mem_           (TParent::param_box_memory_map()),
      slot_start_episode     (*this),
      slot_finish_episode    (*this),
      slot_start_action      (*this),
      slot_finish_action     (*this),
      slot_start_timestep    (*this),
      slot_finish_timestep   (*this),
      signal_reward          (*this),
      signal_end_of_episode  (*this),
      in_base_pose           (*this),
      in_base_vel            (*this),
      in_base_rot            (*this),
      in_contact_with_ground (*this),
      in_contact_with_object (*this)
    {
      add_slot_port   (slot_start_episode     );
      add_slot_port   (slot_finish_episode    );
      add_slot_port   (slot_start_action      );
      add_slot_port   (slot_finish_action     );
      add_slot_port   (slot_start_timestep    );
      add_slot_port   (slot_finish_timestep   );
      add_signal_port (signal_reward          );
      add_signal_port (signal_end_of_episode  );
      add_in_port     (in_base_pose           );
      add_in_port     (in_base_vel            );
      add_in_port     (in_base_rot            );
      add_in_port     (in_contact_with_ground );
      add_in_port     (in_contact_with_object );
    }

protected:

  THumanoidUnivTaskConfigurations  conf_;
  THumanoidUnivTaskMemory          mem_;

  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_finish_episode, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_start_action, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_finish_action, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_start_timestep, void, (const TReal &dt), (dt), TThis);
  MAKE_SLOT_PORT(slot_finish_timestep, void, (const TReal &dt), (dt), TThis);

  MAKE_SIGNAL_PORT(signal_reward, void (const TSingleReward &), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  MAKE_IN_PORT(in_base_pose          , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_vel           , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_rot           , const TRealMatrix& (void), TThis);
  MAKE_IN_PORT(in_contact_with_ground, const TBoolVector& (void), TThis);
  MAKE_IN_PORT(in_contact_with_object, const TBoolVector& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(base_pose          , const TRealVector&, (void), ())
  GET_FROM_IN_PORT(base_vel           , const TRealVector&, (void), ())
  GET_FROM_IN_PORT(base_rot           , const TRealMatrix&, (void), ())
  GET_FROM_IN_PORT(contact_with_ground, const TBoolVector&, (void), ())
  GET_FROM_IN_PORT(contact_with_object, const TBoolVector&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual void slot_start_episode_exec (void);
  virtual void slot_finish_episode_exec (void);

  virtual void slot_start_action_exec (void);
  virtual void slot_finish_action_exec (void);

  virtual void slot_start_timestep_exec (const TReal &dt);
  virtual void slot_finish_timestep_exec (const TReal &dt);

  void sense_from_inports()
    {
      mem_.BasePose          = get_base_pose();
      mem_.BaseVel           = get_base_vel();
      mem_.BaseRot           = get_base_rot();
      mem_.ContactWithGround = get_contact_with_ground();
      mem_.ContactWithObject = get_contact_with_object();
    }

};  // end of MHumanoidUnivTask
//-------------------------------------------------------------------------------------------



//===========================================================================================
// supplementary functions to work with ODE
//===========================================================================================

extern MHumanoidEnvironment *ptr_environment;

void ODEDS_Start();
void ODEDS_Stop();
void ODEDS_KeyEvent (int cmd);
void ODEDS_StepLoop (int ds_pause);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // libhumanoid01_h
//-------------------------------------------------------------------------------------------
