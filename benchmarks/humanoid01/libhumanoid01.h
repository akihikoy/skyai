//-------------------------------------------------------------------------------------------
/*! \file    libhumanoid01.h
    \brief   benchmarks - motion learning task of a simulation humanoid robot
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.04, 2010-
    \date    Jun.07, 2012
    \date    Aug.31, 2012

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
#include <skyai/modules_core/univ_task.h>
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

  //! forwarding the step cost given from the environment module
  TBool   ForwardStepCost;

  THumanoidUnivTaskConfigurations (var_space::TVariableMap &mmap)
    :
      ForwardStepCost  (true)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( ForwardStepCost );
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
      these variables are assumed to be used in user-defined functions, */
  TRealVector  BasePose;
  TRealVector  BaseVel;
  TRealMatrix  BaseRot;
  std::list<bool>  ContactWithGround;
  std::list<bool>  ContactWithObject;

  THumanoidUnivTaskMemory (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( BasePose          );
      ADD( BaseVel           );
      ADD( BaseRot           );
      ADD( ContactWithGround );
      ADD( ContactWithObject );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief universal task module for humanoid environment
class MHumanoidUnivTask
    : public MUniversalContTimeTask
//===========================================================================================
{
public:
  typedef MUniversalContTimeTask  TParent;
  typedef MHumanoidUnivTask       TThis;
  SKYAI_MODULE_NAMES(MHumanoidUnivTask)

  MHumanoidUnivTask (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      hconf_         (TParent::param_box_config_map()),
      hmem_          (TParent::param_box_memory_map()),
      slot_step_cost         (*this),
      in_base_pose           (*this),
      in_base_vel            (*this),
      in_base_rot            (*this),
      in_contact_with_ground (*this),
      in_contact_with_object (*this)
    {
      add_slot_port   (slot_step_cost         );
      add_in_port     (in_base_pose           );
      add_in_port     (in_base_vel            );
      add_in_port     (in_base_rot            );
      add_in_port     (in_contact_with_ground );
      add_in_port     (in_contact_with_object );
    }

protected:

  THumanoidUnivTaskConfigurations  hconf_;
  THumanoidUnivTaskMemory          hmem_;

  MAKE_SLOT_PORT(slot_step_cost, void, (const TSingleReward &c), (c), TThis);

  MAKE_IN_PORT(in_base_pose          , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_vel           , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_rot           , const TRealMatrix& (void), TThis);
  MAKE_IN_PORT(in_contact_with_ground, const TBoolVector& (void), TThis);
  MAKE_IN_PORT(in_contact_with_object, const TBoolVector& (void), TThis);

  virtual void slot_step_cost_exec (const TSingleReward &c)
    {
      if(hconf_.ForwardStepCost)  signal_reward.ExecAll(c);
    }

  template <typename t_container_dest, typename t_container_src>
  static void copy_container(t_container_dest &dest, const t_container_src &src)
    {
      dest.resize(src.size());
      std::copy(src.begin(),src.end(),dest.begin());
    }

  override void sense_common()
    {
      if (in_base_pose          .ConnectionSize()!=0)  hmem_.BasePose          = in_base_pose          .GetFirst();
      if (in_base_vel           .ConnectionSize()!=0)  hmem_.BaseVel           = in_base_vel           .GetFirst();
      if (in_base_rot           .ConnectionSize()!=0)  hmem_.BaseRot           = in_base_rot           .GetFirst();
      if (in_contact_with_ground.ConnectionSize()!=0)  copy_container(hmem_.ContactWithGround, in_contact_with_ground.GetFirst());
      if (in_contact_with_object.ConnectionSize()!=0)  copy_container(hmem_.ContactWithObject, in_contact_with_object.GetFirst());
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
