//-------------------------------------------------------------------------------------------
/*! \file    libdynsim.h
    \brief   skyai - certain application (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.28, 2013

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
#ifndef app_libdynsim_h
#define app_libdynsim_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <skyai/modules_core/univ_task.h>
#include <lora/variable_space_impl.h>
#include <lora/ctrl_tools.h>
#include <lora/robot_model.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TDynamicsSimulatorConfigurations
//===========================================================================================
{
public:

  std::list<TString>         ModelFiles;
  TBool                      EnableDefaultKeyEvent;

  TDynamicsSimulatorConfigurations (var_space::TVariableMap &mmap)
    :
      EnableDefaultKeyEvent(true)
    {
      Register(mmap);
    }

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( ModelFiles                 );
      ADD( EnableDefaultKeyEvent      );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Dynamics simulator environment module */
class MDynamicsSimulator
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MDynamicsSimulator  TThis;
  SKYAI_MODULE_NAMES(MDynamicsSimulator)

  MDynamicsSimulator (const std::string &v_instance_name)
      :
        TParent                     (v_instance_name),
        conf_                       (TParent::param_box_config_map()),
        slot_initialize             (*this),
        slot_start_episode          (*this),
        slot_finish_loop            (*this),
        signal_start_of_timestep    (*this),
        signal_end_of_timestep      (*this),
        signal_key_event            (*this),
        out_world                   (*this)
    {
      add_slot_port   (slot_initialize            );
      add_slot_port   (slot_start_episode         );
      add_slot_port   (slot_finish_loop           );
      add_signal_port (signal_start_of_timestep   );
      add_signal_port (signal_end_of_timestep     );
      add_signal_port (signal_key_event           );
      add_out_port    (out_world                  );
    }

  void Start()
    {
      world_.Start();
    }

  void Stop()
    {
      executing_= false;
      world_.Stop();
    }

  void StepDrawing()
    {
      world_.StepDrawing();
    }

  bool Step()
    {
      if (!executing_)  {world_.Stop(); return true;}
      return world_.Step();
    }

  void KeyEvent (int cmd);

  bool Executing() const {return executing_;}
  bool ConsoleMode() const {return world_.ConsoleMode();}
  void SetConsoleMode(bool m)  {world_.SetConsoleMode(m);}

protected:

  TDynamicsSimulatorConfigurations conf_;

  mutable xode::TWorld world_;

  bool   executing_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_finish_loop, void, (void), (), TThis);


  MAKE_SIGNAL_PORT(signal_start_of_timestep, void (const TContinuousTime &), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_timestep, void (const TContinuousTime &), TThis);

  MAKE_SIGNAL_PORT(signal_key_event, void (const TInt &cmd), TThis);

  MAKE_OUT_PORT(out_world, xode::TWorld&, (void), (), TThis);

  virtual void slot_initialize_exec (void)
    {
      world_.SetCallbacks().StartOfTimeStep= boost::bind(&MDynamicsSimulator::callback_start_of_timestep,this,_1,_2);
      world_.SetCallbacks().EndOfTimeStep= boost::bind(&MDynamicsSimulator::callback_end_of_timestep,this,_1,_2);
      LMESSAGE("Loading model files..");
      for(std::list<TString>::const_iterator itr(conf_.ModelFiles.begin()),last(conf_.ModelFiles.end());itr!=last;++itr)
        world_.LoadFromFile(Agent().SearchFileName(*itr));
      world_.Create();
      LMESSAGE("Starting simulation..");
      executing_= true;
    }
  virtual void slot_start_episode_exec (void)
    {
      world_.Create();
    }

  virtual void slot_finish_loop_exec (void)
    {
      executing_= false;
    }

  virtual xode::TWorld& out_world_get() const
    {
      return world_;
    }

  void callback_start_of_timestep(xode::TWorld &w, const TReal &time_step)
    {
      signal_start_of_timestep.ExecAll(time_step);
    }
  void callback_end_of_timestep(xode::TWorld &w, const TReal &time_step)
    {
      signal_end_of_timestep.ExecAll(time_step);
    }

};  // end of MDynamicsSimulator
//-------------------------------------------------------------------------------------------



//===========================================================================================
class TRobotProbeConfigurations
//===========================================================================================
{
public:

  TString                    RobotName;

  TRealVector                PDGainKp, PDGainKd;  //!< gain parameters for lowlevel PD-controller
  TRealVector                TorqueMax;  //!< Nm

  TRealVector                CommandMax, CommandMin;  //!< upper and lower bounds for command

  TReal                      BodyContactLPFParamF;
  TReal                      BodyContactLPFParamQ;
  TReal                      BodyContactThreshold;


  TRobotProbeConfigurations (var_space::TVariableMap &mmap)
    :
      PDGainKp                  (1,2.5l),
      PDGainKd                  (1,0.08l),
      TorqueMax                 (1,1.0290l),
      CommandMax                (0),
      CommandMin                (0),
      BodyContactLPFParamF      (10.0l),
      BodyContactLPFParamQ      (0.8l),
      BodyContactThreshold      (0.1l)
    {
      Register(mmap);
    }

  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( RobotName                      );
      ADD( PDGainKp                       );
      ADD( PDGainKd                       );
      ADD( TorqueMax                      );
      ADD( CommandMax                     );
      ADD( CommandMin                     );
      ADD( BodyContactLPFParamF           );
      ADD( BodyContactLPFParamQ           );
      ADD( BodyContactThreshold           );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief Robot probe for MDynamicsSimulator that provides accessors to a robot in the models */
class MRobotProbe
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MRobotProbe        TThis;
  SKYAI_MODULE_NAMES(MRobotProbe)

  MRobotProbe (const std::string &v_instance_name)
    :
      TParent                     (v_instance_name),
      conf_                       (TParent::param_box_config_map()),
      robot_index_                (-1),
      base_index_                 (-1),
      PDC_                        (0, 0.0, 0.0, 0.0),

      slot_initialize             (*this),
      slot_start_episode          (*this),
      slot_execute_command_des_q  (*this),
      slot_execute_command_des_qd (*this),
      slot_start_time_step        (*this),
      slot_finish_time_step       (*this),
      in_world                    (*this),
      out_base_pose               (*this),
      out_base_vel                (*this),
      out_base_rot                (*this),
      out_joint_angle             (*this),
      out_joint_vel               (*this),
      out_force                   (*this),
      out_contact_raw             (*this),
      out_contact                 (*this)
    {
      add_slot_port   (slot_initialize            );
      add_slot_port   (slot_start_episode         );
      add_slot_port   (slot_execute_command_des_q );
      add_slot_port   (slot_execute_command_des_qd);
      add_slot_port   (slot_start_time_step       );
      add_slot_port   (slot_finish_time_step      );
      add_in_port     (in_world                   );
      add_out_port    (out_base_pose              );
      add_out_port    (out_base_vel               );
      add_out_port    (out_base_rot               );
      add_out_port    (out_joint_angle            );
      add_out_port    (out_joint_vel              );
      add_out_port    (out_force                  );
      add_out_port    (out_contact_raw            );
      add_out_port    (out_contact                );
    }


protected:

  TRobotProbeConfigurations conf_;

  TInt   robot_index_, base_index_;

  TPDController       PDC_;
  TLHBPFilters<std::vector<float> >  contact_LPF_;
  TRealVector         current_command_;
  TRealVector         tq_input_;
  mutable TContinuousState  tmp_joint_state_;
  mutable TContinuousState  tmp_base_pose_, tmp_base_vel_, tmp_joint_angle_, tmp_joint_vel_;
  mutable TRealMatrix       tmp_base_rot_;
  mutable TRealVector       tmp_force_;
  mutable TBoolVector       tmp_contact_raw_, tmp_contact_filtered_;
  mutable std::vector<float>  tmp_contact_float_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  //! u: desired joint angle (using the controller's constraint mode)
  MAKE_SLOT_PORT(slot_execute_command_des_q, void, (const TRealVector &u), (u), TThis);

  //! u: desired displacement of the joint angle (using the controller's constraint mode)
  MAKE_SLOT_PORT(slot_execute_command_des_qd, void, (const TRealVector &u), (u), TThis);


  MAKE_SLOT_PORT(slot_start_time_step, void, (const TContinuousTime &dt), (dt), TThis);

  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TContinuousTime &dt), (dt), TThis);


  MAKE_IN_PORT(in_world, xode::TWorld& (void), TThis);

  //!\brief output base link pose (position and rotation) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_pose, const TContinuousState&, (void), (), TThis);
  //!\brief output base link velocities (of position and rotation) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_vel, const TContinuousState&, (void), (), TThis);

  //!\brief output base rotation matrix[3x3]
  MAKE_OUT_PORT(out_base_rot, const TRealMatrix&, (void), (), TThis);

  //!\brief output joint angles according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_angle, const TContinuousState&, (void), (), TThis);
  //!\brief output joint angular velocities according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_vel, const TContinuousState&, (void), (), TThis);

  MAKE_OUT_PORT(out_force, const TContinuousState&, (void), (), TThis);
  MAKE_OUT_PORT(out_contact_raw, const TBoolVector&, (void), (), TThis);
  MAKE_OUT_PORT(out_contact, const TBoolVector&, (void), (), TThis);

  virtual void slot_initialize_exec (void)
    {
      slot_start_episode_exec();
    }

  virtual void slot_start_episode_exec (void)
    {
      if(in_world.ConnectionSize()==0)
      {
        LERROR("MRobotProbe: in_world should be connected.");
        lexit(df);
      }

      xode::TWorld &w(in_world.GetFirst());

      robot_index_= w.RobotIndex(conf_.RobotName);
      if(robot_index_<0)
      {
        LERROR("MRobotProbe: robot "<<conf_.RobotName<<" is not defined.");
        lexit(df);
      }
      base_index_= w.RootLinkBodyIndex(conf_.RobotName);
      if(base_index_<0)
      {
        LERROR("MRobotProbe: cannot get the root index of the robot "<<conf_.RobotName);
        lexit(df);
      }

      LASSERT1op1(w.JointAngleNum(robot_index_),==,w.JointAngVelNum(robot_index_));
      LASSERT1op1(w.JointAngleNum(robot_index_),==,w.JointTorqueInputNum(robot_index_));
      int dof(w.JointAngleNum(robot_index_));
      PDC_= TPDController(dof,0.0,0.0,0.0);
      #define SET_PARAM(x_dest,x_src)  \
        do{int size(GenSize(x_src));                                     \
          if(size==0)        x_dest= ColumnVector(dof,0.0);              \
          else if(size==1)   x_dest= ColumnVector(dof,GenAt(x_src,0));   \
          else if(size==dof) x_dest= x_src;                              \
          else {LERROR("MRobotProbe: invalid "#x_src" size: "<<size);    \
                x_dest= ColumnVector(dof,0.0);}                          \
        }while(0)
      SET_PARAM(PDC_.Kp, conf_.PDGainKp);
      SET_PARAM(PDC_.Kd, conf_.PDGainKd);
      SET_PARAM(PDC_.UMax, conf_.TorqueMax);
      #undef SET_PARAM

      contact_LPF_.Initialize (TLHBPFilters<std::vector<float> >::LPF2,
        w.Params().TimeStep, conf_.BodyContactLPFParamF/*f*/, conf_.BodyContactLPFParamQ/*q*/,
        std::vector<float>(w.LinkContacts(robot_index_).size(),0.0f)/*, tmp_contact_float_ :initial*/);
    }

  virtual void slot_execute_command_des_q_exec (const TRealVector &u)
    {
      current_command_= u;
    }
  virtual void slot_execute_command_des_qd_exec (const TRealVector &u)
    {
      LASSERT1op1(robot_index_,>=,0);
      in_world.GetFirst().GetJointAngles(robot_index_, tmp_joint_angle_);
      current_command_= tmp_joint_angle_ + u;
    }

  virtual void slot_start_time_step_exec (const TContinuousTime &dt)
    {
      LASSERT1op1(robot_index_,>=,0);
      if(GenSize(current_command_)>0)
      {
        xode::TWorld &w(in_world.GetFirst());
        LASSERT1op1(GenSize(current_command_), ==, w.JointTorqueInputNum(robot_index_));
        ConstrainVector(current_command_, conf_.CommandMin, conf_.CommandMax);
        GenResize(tmp_joint_state_, w.JointAngleNum(robot_index_)+w.JointAngVelNum(robot_index_));
        TypeExt<TContinuousState>::iterator  vel_begin(GenBegin(tmp_joint_state_)+w.JointAngleNum(robot_index_));
        w.GetJointAngles(robot_index_, GenBegin(tmp_joint_state_), vel_begin);
        w.GetJointAngVels(robot_index_, vel_begin, GenEnd(tmp_joint_state_));
        tq_input_= PDC_(tmp_joint_state_, current_command_);
        w.AddToJointTorques(robot_index_, GenBegin(tq_input_),GenEnd(tq_input_));
      }
    }

  virtual void slot_finish_time_step_exec (const TContinuousTime &dt)
    {
      if(out_contact.ConnectionSize()>0)
      {
        const std::vector<bool> &contacts= in_world.GetFirst().LinkContacts(robot_index_);
        tmp_contact_float_.resize(contacts.size());
        std::vector<bool>::const_iterator bitr(contacts.begin());
        for(std::vector<float>::iterator itr(tmp_contact_float_.begin()),last(tmp_contact_float_.end());itr!=last;++itr)
          *itr= (*bitr ? 1.0 : 0.0);
        contact_LPF_ (tmp_contact_float_);
      }
    }

  virtual const TContinuousState& out_base_pose_get() const;
  virtual const TContinuousState& out_base_vel_get() const;
  virtual const TRealMatrix& out_base_rot_get() const;
  virtual const TContinuousState& out_joint_angle_get() const;
  virtual const TContinuousState& out_joint_vel_get() const;
  virtual const TContinuousState& out_force_get() const;
  virtual const TBoolVector& out_contact_raw_get() const;
  virtual const TBoolVector& out_contact_get() const;

};  // end of MRobotProbe
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TDynSimUnivTaskConfigurations
//===========================================================================================
{
public:

  //! forwarding the step cost given from the environment module
  TBool   ForwardStepCost;

  TDynSimUnivTaskConfigurations (var_space::TVariableMap &mmap)
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
class TDynSimUnivTaskMemory
//===========================================================================================
{
public:

  /*! the state of the robot is stored into following variables.
      these variables are assumed to be used in user-defined functions, */
  TRealVector  BasePose;
  TRealVector  BaseVel;
  TRealMatrix  BaseRot;
  TRealVector  Force;
  std::list<bool>  Contact;

  TDynSimUnivTaskMemory (var_space::TVariableMap &mmap)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( BasePose          );
      ADD( BaseVel           );
      ADD( BaseRot           );
      ADD( Force             );
      ADD( Contact           );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief universal task module for humanoid environment
class MDynSimUnivTask
    : public MUniversalContTimeTask
//===========================================================================================
{
public:
  typedef MUniversalContTimeTask  TParent;
  typedef MDynSimUnivTask         TThis;
  SKYAI_MODULE_NAMES(MDynSimUnivTask)

  MDynSimUnivTask (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      hconf_         (TParent::param_box_config_map()),
      hmem_          (TParent::param_box_memory_map()),
      slot_step_cost         (*this),
      in_base_pose           (*this),
      in_base_vel            (*this),
      in_base_rot            (*this),
      in_force               (*this),
      in_contact             (*this)
    {
      add_slot_port   (slot_step_cost         );
      add_in_port     (in_base_pose           );
      add_in_port     (in_base_vel            );
      add_in_port     (in_base_rot            );
      add_in_port     (in_force               );
      add_in_port     (in_contact             );
    }

protected:

  TDynSimUnivTaskConfigurations  hconf_;
  TDynSimUnivTaskMemory          hmem_;

  MAKE_SLOT_PORT(slot_step_cost, void, (const TSingleReward &c), (c), TThis);

  MAKE_IN_PORT(in_base_pose          , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_vel           , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_base_rot           , const TRealMatrix& (void), TThis);
  MAKE_IN_PORT(in_force              , const TRealVector& (void), TThis);
  MAKE_IN_PORT(in_contact            , const TBoolVector& (void), TThis);

  virtual void slot_step_cost_exec (const TSingleReward &c)
    {
      if(hconf_.ForwardStepCost)  signal_reward.ExecAll(c);
    }

  template <typename t_container_dest, typename t_container_src>
  static void copy_container(t_container_dest &dest, const t_container_src &src)
    {
      GenResize(dest, GenSize(src));
      std::copy(GenBegin(src),GenEnd(src),GenBegin(dest));
    }

  override void sense_common()
    {
      if (in_base_pose          .ConnectionSize()!=0)  hmem_.BasePose          = in_base_pose          .GetFirst();
      if (in_base_vel           .ConnectionSize()!=0)  hmem_.BaseVel           = in_base_vel           .GetFirst();
      if (in_base_rot           .ConnectionSize()!=0)  hmem_.BaseRot           = in_base_rot           .GetFirst();
      if (in_force              .ConnectionSize()!=0)  copy_container(hmem_.Force, in_force.GetFirst());
      if (in_contact            .ConnectionSize()!=0)  copy_container(hmem_.Contact, in_contact.GetFirst());
    }

};  // end of MDynSimUnivTask
//-------------------------------------------------------------------------------------------



//===========================================================================================
// supplementary functions to work with ODE
//===========================================================================================

extern MDynamicsSimulator *ptr_dynamics_simulator;

void ODEDS_DSStart();
void ODEDS_DSStop();
void ODEDS_DSKeyEvent (int cmd);
void ODEDS_DSStep (int ds_pause);
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // app_libdynsim_h
//-------------------------------------------------------------------------------------------
