//-------------------------------------------------------------------------------------------
/*! \file    maze2d.cpp
    \brief   benchmarks - test libskyai on a simple toyproblem: navigation task in 2d-maze
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.23, 2009-

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
#include "detail/humanoid01.h"
#include <skyai/skyai.h>
#include <skyai/modules_core/learning_manager.h>
#include <skyai/parser.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

ENUM_STR_MAP_BEGIN_NS(humanoid_controller, TControlConstraintKind)
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckUnspecified        )
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckNone               )
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckStrictSymmetric    )
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckWideSymmetric      )
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckFBSymmetric        )
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckLowerBody          )
  ENUM_STR_MAP_ADD_NS(humanoid_controller, cckLowerBodyAutoFoot  )
ENUM_STR_MAP_END_NS  (humanoid_controller, TControlConstraintKind)
SPECIALIZE_TVARIABLE_TO_ENUM(humanoid_controller::TControlConstraintKind)


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

  // parameters for humanoid_controller::THumanoidControllerCondition hmdctrlcnd
  TRealVector                PDGainKp, PDGainKd;  //!< gain parameters for lowlevel controller (PD)
  TRealVector                TorqueMax;
  humanoid_controller::TControlConstraintKind   ControlConstraintKind;

  // parameters for TSimulationCondition simulationcnd
  int                        MaxContactNum;  //!< maximum number of contact points per body
  dSurfaceParameters         Surface;       //!< parameters of contact face
  double                     BodyContactLPFParamF;
  double                     BodyContactLPFParamQ;
  bool                       ForceInitFeetContactWithGround;  //!< true: force to initialize _bodies_contact_with_ground[FEET]=1
  bool                       UsingQuickStep;  //!< use quickStep to step the world. this mode is fast, but sometimes lose the accuracy
  int                        QuickStepIterationNum;  //!< number of iteration used in quickStep

  THumanoidEnvironmentConfigurations (var_space::TVariableMap &mmap) :
      TimeStep            (0.0002l),
      FPS                 (50.0l),
      ViewPoint           (6,0.0),
      UseInitPose         (false),
      // parameters for humanoid_controller::hmdctrlcnd:
      PDGainKp                 (JOINT_NUM,2.5l),
      PDGainKd                 (JOINT_NUM,0.08l),
      TorqueMax                (JOINT_NUM, loco_rabbits::TorqueMax),
      ControlConstraintKind    (humanoid_controller::cckStrictSymmetric),
      // parameters for simulationcnd:
      MaxContactNum             (10),
      BodyContactLPFParamF      (10.0),
      BodyContactLPFParamQ      (0.8),
      ForceInitFeetContactWithGround (false),
      UsingQuickStep                 (false),
      QuickStepIterationNum          (20)
    {
      ViewPoint= OctGen1<ColumnVector>(6, 0.35,0.30,0.30, -135.0000,-20.0000,0.0000);

      Surface.mode       = dContactBounce | dContactSoftCFM;
      Surface.mu         = dInfinity;
      Surface.mu2        = 0;
      Surface.bounce     = 0.00001;  // 0.1;
      Surface.bounce_vel = 0.1; // 0.1;
      Surface.soft_erp   = 0.0;
      Surface.soft_cfm   = 0.001;   //  0.01;
      Surface.motion1    = 0.0;
      Surface.motion2    = 0.0;
      Surface.slip1      = 0.0;
      Surface.slip2      = 0.0;

      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( TimeStep                   );
      ADD( FPS                        );
      ADD( ViewPoint                  );
      ADD( UseInitPose                );
      ADD( InitBodyPosRotQ            );
      ADD( InitJointAngles            );

      // parameters for humanoid_controller::hmdctrlcnd:
      ADD( PDGainKp                       );
      ADD( PDGainKd                       );
      ADD( TorqueMax                      );
      ADD( ControlConstraintKind          );

      // parameters for humanoid_controller::hmdctrlcnd:
      ADD( MaxContactNum                              );
      ADD( Surface                                    );
      ADD( BodyContactLPFParamF                       );
      ADD( BodyContactLPFParamQ                       );
      ADD( ForceInitFeetContactWithGround             );
      ADD( UsingQuickStep                             );
      ADD( QuickStepIterationNum                      );
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

  MHumanoidEnvironment (const std::string &v_instance_name)
    : TParent                     (v_instance_name),
      conf_                       (TParent::param_box_config_map()),
      tq_input_                   (JOINT_NUM,0.0),
      executing_                  (false),
      console_mode_               (false),
      repaint_time_               (0),
      old_fps_                    (-1),

      slot_initialize             (*this),
      slot_start_episode          (*this),
      slot_execute_command_des_q  (*this),
      slot_execute_command_des_qd (*this),
      slot_step_loop              (*this),
      slot_finish_loop            (*this),
      signal_start_of_timestep    (*this),
      signal_end_of_timestep      (*this),
      signal_system_reward        (*this),
      signal_end_of_episode       (*this),
      out_state_cc                (*this),
      out_base_state              (*this),
      out_joint_state             (*this),
      out_contact_with_ground     (*this)
    {
      add_slot_port   (slot_initialize            );
      add_slot_port   (slot_start_episode         );
      add_slot_port   (slot_execute_command_des_q );
      add_slot_port   (slot_execute_command_des_qd);
      add_slot_port   (slot_step_loop             );
      add_slot_port   (slot_finish_loop           );
      add_signal_port (signal_start_of_timestep   );
      add_signal_port (signal_end_of_timestep     );
      add_signal_port (signal_system_reward       );
      add_signal_port (signal_end_of_episode      );
      add_out_port    (out_state_cc               );
      add_out_port    (out_base_state             );
      add_out_port    (out_joint_state            );
      add_out_port    (out_contact_with_ground    );
    }

  void StepLoop (int ds_pause=0)
    {
      if (!console_mode_)
      {
        if (!executing_)  {dsStop(); return;}

        if (old_fps_!=conf_.FPS)
        {
          repaint_time_= int(real_round(1.0l/conf_.TimeStep/conf_.FPS));
          old_fps_= conf_.FPS;
        }

        if (!ds_pause)
        {
          for(int rptime(repaint_time_); rptime>0;--rptime)
          {
            slot_step_loop.Exec();
            if (console_mode_ || !executing_)  {dsStop(); break;}
          }
        }
        draw_world(ds_pause);
      }
      else
      {
        slot_step_loop.Exec();
      }
    }

  void ODEDS_Start()
    {
      start();
      float xyzhpr[6];
      for(int i(0); i<6; ++i)  xyzhpr[i]= static_cast<float>(conf_.ViewPoint(i));
      dsSetViewpoint (xyzhpr,xyzhpr+3);
      // xdsSetLightPos(-0.4f,-0.4f);
    }

  void ODEDS_Stop()
    {
      executing_= false;
    }

  void ODEDS_KeyEvent (int cmd)
    {
      using namespace std;
      if (cmd=='i'||cmd=='I')
      {
        show_robot_info();
      }
      else if (cmd=='s'||cmd=='S')
      {
        DRAW_SEQUENCE_MODE= !DRAW_SEQUENCE_MODE;
        _SEQUENCE_LIST.clear();
        if(DRAW_SEQUENCE_MODE)  LMESSAGE("DRAW_SEQUENCE_MODE is \"on\".");
        else                    LMESSAGE("DRAW_SEQUENCE_MODE is \"off\".");
      }
      else if (cmd=='v')
      {
        AUTO_VIEWPOINT_MODE=(AUTO_VIEWPOINT_MODE==GetAutoViewpointModeCount())?0:AUTO_VIEWPOINT_MODE+1;
        LMESSAGE("AUTO_VIEWPOINT_MODE= "<<AUTO_VIEWPOINT_MODE<<".");
      }
      else if (cmd=='V')
      {
        static float xyz[3], hpr[3];
        cout<<"input XYZHPR: ";
        for(int i(0);i<3;++i)  cin>>xyz[i];
        for(int i(0);i<3;++i)  cin>>hpr[i];
        dsSetViewpoint (xyz,hpr);
      }
      else if (cmd=='f')
      {
        --display_fps_index;
        if(display_fps_index<0) display_fps_index=sizeof(display_fps_set)/sizeof(display_fps_set[0])-1;
        conf_.FPS = display_fps_set[display_fps_index];
        LMESSAGE("current FPS= "<<conf_.FPS);
      }
      else if (cmd=='F')
      {
        cout<<"input FPS: ";
        cin>>conf_.FPS;
        LMESSAGE("current FPS= "<<conf_.FPS);
      }
      else
        LMESSAGE("unrecognized key command: "<<cmd);
    }

  bool Executing() const {return executing_;}
  bool ConsoleMode() const {return console_mode_;}
  void SetConsoleMode(bool m)  {console_mode_= m;}

protected:

  THumanoidEnvironmentConfigurations conf_;

  TRealVector         current_command_;
  TRealVector         tq_input_;
  mutable TContinuousState  tmp_state_;
  mutable TContinuousState  tmp_jangle_;
  mutable TContinuousState  tmp_base_state_, tmp_joint_state_;
  mutable TBoolVector       tmp_contact_with_ground_;

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

  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  //!\brief output state according to the controller's constraint mode
  MAKE_OUT_PORT(out_state_cc, const TContinuousState&, (void), (), TThis);

  //! output base link state (position, pose, and their velocities) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_state, const TContinuousState&, (void), (), TThis);

  //! output joint state (angles and angular velocities) according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_state, const TContinuousState&, (void), (), TThis);

  MAKE_OUT_PORT(out_contact_with_ground, const TBoolVector&, (void), (), TThis);

  virtual void slot_initialize_exec (void)
    {
      // humanoid_controller::hmdctrlcnd.ControlConstraintKind= humanoid_controller::cckStrictSymmetric;

      // copy parameters to humanoid_controller::hmdctrlcnd
      using namespace humanoid_controller;
      hmdctrlcnd.PDGainKp               =  conf_.PDGainKp               ;
      hmdctrlcnd.PDGainKd               =  conf_.PDGainKd               ;
      hmdctrlcnd.TorqueMax              =  conf_.TorqueMax              ;
      hmdctrlcnd.ControlConstraintKind  =  conf_.ControlConstraintKind  ;

      INITIALIZE_DIMENSIONS();

      // copy parameters to simulationcnd
      simulationcnd.MaxContactNum                   = conf_.MaxContactNum                   ;
      simulationcnd.Surface                         = conf_.Surface                         ;
      simulationcnd.BodyContactLPFParamF            = conf_.BodyContactLPFParamF            ;
      simulationcnd.BodyContactLPFParamQ            = conf_.BodyContactLPFParamQ            ;
      simulationcnd.ForceInitFeetContactWithGround  = conf_.ForceInitFeetContactWithGround  ;
      simulationcnd.UsingQuickStep                  = conf_.UsingQuickStep                  ;
      simulationcnd.QuickStepIterationNum           = conf_.QuickStepIterationNum           ;

      use_contact_LPF= true;

      InitializeODE();
      initSimulation2();
      LMESSAGE("start simulation..");
      SetupODE();

      executing_= true;
    }

  virtual void slot_start_episode_exec (void)
    {
      initSimulation2(/*using_cart=*/false);
      // setting initial pose...
      if (conf_.UseInitPose)
      {
        dVector3 newpos;
        dQuaternion newq;
        std::copy (GenBegin(conf_.InitBodyPosRotQ),GenBegin(conf_.InitBodyPosRotQ)+3, newpos);
        std::copy (GenBegin(conf_.InitBodyPosRotQ)+3,GenBegin(conf_.InitBodyPosRotQ)+7, newq);
        ODESetArticulatedBodyPosRotQ (body[0].id(), newpos, newq);
        setJointAngle (conf_.InitJointAngles);
      }
    }

  virtual void slot_execute_command_des_q_exec (const TRealVector &u)
    {
      current_command_= u;
    }
  virtual void slot_execute_command_des_qd_exec (const TRealVector &u)
    {
      //!\todo FIXME: make efficient computation!
      getState(tmp_state_);
      humanoid_controller::extractControlPos(tmp_jangle_,tmp_state_,0);
      current_command_= tmp_jangle_ + u;
    }

  virtual void slot_step_loop_exec (void)
    {
      signal_start_of_timestep.ExecAll(conf_.TimeStep);

      /*dbg*/if (current_command_.length()==0)  {LERROR("slot_execute_command_des_qd is not called!!!");}

      ////////////////////////
      humanoid_controller::lowLevelRobotModel (current_command_, tq_input_);
      TSingleReward step_cost(0.0l);
      worldStep (tq_input_, conf_.TimeStep, step_cost);
      if (ModuleMode()==TModuleInterface::mmDebug)
        {DebugStream()<<slot_step_loop.UniqueCode()<<":  WorldStep is executed (cost="<<step_cost<<")"<<std::endl;}
      ////////////////////////

      TSingleReward reward(0.0l);
      reward+= -0.1l*step_cost*conf_.TimeStep;

      signal_system_reward.ExecAll(reward);

      signal_end_of_timestep.ExecAll(conf_.TimeStep);
    }

  virtual void slot_finish_loop_exec (void)
    {
      executing_= false;
    }

  virtual const TContinuousState& out_state_cc_get () const
    {
      getState(tmp_state_);
      return tmp_state_;
    }

  virtual const TContinuousState& out_base_state_get() const
    {
      using namespace humanoid_controller;
      GenResize(tmp_base_state_,BASE_STATE_DIM);
      getBaseState(tmp_base_state_);
      return tmp_base_state_;
    }

  virtual const TContinuousState& out_joint_state_get() const
    {
      using namespace humanoid_controller;
      GenResize(tmp_joint_state_,JOINT_STATE_DIM);
      getJointState(tmp_joint_state_);
      return tmp_joint_state_;
    }

  virtual const TBoolVector& out_contact_with_ground_get() const
    {
      GenResize(tmp_contact_with_ground_,BODY_NUM);
      bodies_contact_with_ground (tmp_contact_with_ground_);
      return tmp_contact_with_ground_;
    }

};  // end of MHumanoidEnvironment
//-------------------------------------------------------------------------------------------


enum TMotionLearningTaskKind
{
  mltkJump          =0,
  mltkMove          ,
  mltkStandup       ,
  mltkForwardroll   ,
  mltkMoveA         //
};
ENUM_STR_MAP_BEGIN( TMotionLearningTaskKind )
  ENUM_STR_MAP_ADD( mltkJump            )
  ENUM_STR_MAP_ADD( mltkMove            )
  ENUM_STR_MAP_ADD( mltkStandup         )
  ENUM_STR_MAP_ADD( mltkForwardroll     )
  ENUM_STR_MAP_ADD( mltkMoveA           )
ENUM_STR_MAP_END  ( TMotionLearningTaskKind )
SPECIALIZE_TVARIABLE_TO_ENUM(TMotionLearningTaskKind)


//===========================================================================================
class TMotionLearningTaskConfigurations
//===========================================================================================
{
public:

  TMotionLearningTaskKind  TaskKind;
  TReal                    FinishVelocityNorm;
  TSingleReward            SumOfRmin;  //!< if sum of reward in an episode is less than this value, episode is terminated
  TContinuousTime          MaxTime;

  // parameters for mltkMove:
  TReal                    ForwardRewardGain   ;
  TReal                    SidewardPenaltyGain ;


  TMotionLearningTaskConfigurations (var_space::TVariableMap &mmap)
    :
      TaskKind             (mltkMove),
      FinishVelocityNorm   (1.0l),
      SumOfRmin            (-40.0l),
      MaxTime              (20.0l),
      ForwardRewardGain    (0.01l),
      SidewardPenaltyGain  (0.1l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( TaskKind               );
      ADD( FinishVelocityNorm     );
      ADD( SumOfRmin              );
      ADD( MaxTime                );

      ADD( ForwardRewardGain      );
      ADD( SidewardPenaltyGain    );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief task module
class MMotionLearningTask
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MMotionLearningTask  TThis;
  SKYAI_MODULE_NAMES(MMotionLearningTask)

  MMotionLearningTask (const std::string &v_instance_name)
    : TParent                (v_instance_name),
      conf_                  (TParent::param_box_config_map()),
      slot_initialize        (*this),
      slot_start_episode     (*this),
      slot_finish_time_step  (*this),
      signal_end_of_episode  (*this),
      signal_task_reward     (*this),
      signal_damage_reward   (*this),
      in_state               (*this),
      in_sum_of_reward       (*this)
    {
      add_slot_port   (slot_initialize       );
      add_slot_port   (slot_start_episode    );
      add_slot_port   (slot_finish_time_step );
      add_signal_port (signal_end_of_episode );
      add_signal_port (signal_task_reward    );
      add_signal_port (signal_damage_reward  );
      add_in_port     (in_state              );
      add_in_port     (in_sum_of_reward      );
    }

protected:

  TMotionLearningTaskConfigurations  conf_;

  bool             is_jumped_;
  TReal            init_head_height_;
  TContinuousTime  time_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_start_episode, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_finish_time_step, void, (const TContinuousTime &dt), (dt), TThis);

  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);

  MAKE_SIGNAL_PORT(signal_task_reward, void (const TSingleReward&), TThis);
  MAKE_SIGNAL_PORT(signal_damage_reward, void (const TSingleReward&), TThis);

  MAKE_IN_PORT(in_state, const TContinuousState& (void), TThis);
  MAKE_IN_PORT(in_sum_of_reward, const TSingleReward& (void), TThis);


  virtual void slot_initialize_exec (void)
    {
    }

  virtual void slot_start_episode_exec (void)
    {
      is_jumped_= false;
      init_head_height_= body[HEADLINK_INDEX].getPosition()[2];
      time_= 0.0l;
    }

  virtual void slot_finish_time_step_exec (const TContinuousTime &dt)
    {
      time_+=dt;

      // calculate task reward
      {
        TSingleReward  task_reward(0.0l);
        switch (conf_.TaskKind)
        {
          case mltkJump        :
            task_reward= getGoalRewardJump6(dt, init_head_height_);
            if (task_reward>DBL_TINY)  is_jumped_=true;
            break;
          case mltkMove        : task_reward= getGoalRewardMove3(conf_.ForwardRewardGain, conf_.SidewardPenaltyGain);  break;
          case mltkMoveA       : task_reward= getGoalRewardMove4();    break;
          // case mltkStandup     : break;
          case mltkForwardroll : task_reward= getGoalRewardForwardroll1(dt); break;
          default :
            LERROR("invalid TaskKind= "<<conf_.TaskKind);
            lexit(df);
        }
        signal_task_reward.ExecAll(task_reward);
      }

      // calculate damage reward
      bool fallen_down(false);
      {
        if (conf_.TaskKind==mltkJump || conf_.TaskKind==mltkMove || conf_.TaskKind==mltkMoveA)
        {
          if ((fallen_down=fallenDown()))
          {
            TSingleReward dreward=-4.0l;
            signal_damage_reward.ExecAll(dreward);
          }
        }
      }

      // is the end of the episode?
      {
        bool is_end_of_episode(false);
        switch (conf_.TaskKind)
        {
          case mltkJump        :
            if (fallen_down)  {is_end_of_episode=true; break;}
            if (!bodies_contact_with_ground(LFOOT_INDEX) || !bodies_contact_with_ground(RFOOT_INDEX))
              break;
            if (!is_jumped_)  break;
            if (real_fabs(humanoid_controller::extractDynVel(in_state.GetFirst()).max()) < conf_.FinishVelocityNorm)
              {is_end_of_episode=true; break;}
            break;
          case mltkMove        :
          case mltkMoveA       :
          case mltkForwardroll :
            if (in_sum_of_reward.GetFirst() <= conf_.SumOfRmin)
              is_end_of_episode=true;
            break;
          // case mltkStandup     : break;
          default :
            LERROR("invalid TaskKind= "<<conf_.TaskKind);
            lexit(df);
        }
        if (conf_.MaxTime>0.0l && time_+CONT_TIME_TOL>=conf_.MaxTime)
        {
          is_end_of_episode= true;
          if (time_-CONT_TIME_TOL<conf_.MaxTime+dt)
            {LMESSAGE("end of episode @"<<time_<<"[s]: x="<<body[BASELINK_INDEX].getPosition()[0]<<"[m]");}
        }
        if (is_end_of_episode)
        {
          signal_end_of_episode.ExecAll();
        }
      }
    }

};  // end of MMotionLearningTask
//-------------------------------------------------------------------------------------------



SKYAI_ADD_MODULE(MHumanoidEnvironment)
SKYAI_ADD_MODULE(MMotionLearningTask)

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------


MHumanoidEnvironment *ptr_environment(NULL);
void ODEDS_Start()
{
  ptr_environment->ODEDS_Start();
}
void ODEDS_Stop()
{
  ptr_environment->ODEDS_Stop();
}
void ODEDS_KeyEvent (int cmd)
{
  ptr_environment->ODEDS_KeyEvent(cmd);
}
void ODEDS_StepLoop (int ds_pause)
{
  ptr_environment->StepLoop (ds_pause);
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  TOptionParser option(argc,argv);
  option["notex"]; option["noshadow"]; option["noshadows"]; option["pause"];  // these options are used by ODE
  string outdir= option("outdir", "result/");
  bool console_mode= ConvertFromStr<bool>(option("console","false"));

  TAgent  agent;
  if (option("agent")=="")
    {LERROR("fatal! -agent option is needed."); lexit(df);}
  /*load agent files*/{
    TTokenizer tokenizer(option("agent"));
    string agent_file;
    while(!tokenizer.EOL())
    {
      tokenizer.ReadSeparators();
      agent_file= tokenizer.ReadNonSeparators();
      if (!LoadAgentFromFile(agent,agent_file))
        {LERROR("failed to read "<<agent_file); lexit(df);}
    }
  }

  MBasicLearningManager &lmanager = agent.ModuleAs<MBasicLearningManager>("lmanager");
  MHumanoidEnvironment &environment = agent.ModuleAs<MHumanoidEnvironment>("environment");


  if (ConvertFromStr<bool>(option("available_mods","false")))
  {
    LMESSAGE("TModuleManager::ShowAllModules():");
    TModuleManager::ShowAllModules(option("show_conf"));
    return 0;
  }
  if (ConvertFromStr<bool>(option("show_mods","false")))
  {
    LMESSAGE("agent's modules:");
    agent.ShowAllModules(option("show_conf"),cout);
    return 0;
  }
  if (ConvertFromStr<bool>(option("dot_mod","false")))
  {
    std::ofstream mdot((outdir+"manoi01-modules.dot").c_str());
    agent.ExportToDOT(mdot);
    return 0;
  }
  if (ConvertFromStr<bool>(option("show_connect","false")))
  {
    LMESSAGE("agent's connections:");
    agent.ShowAllConnections(cout);
    return 0;
  }

  std::ofstream debug;
  if (ConvertFromStr<bool>(option("dump_debug","false")))
  {
    debug.open((outdir+"manoi01-debug.dat").c_str());
    agent.SetDebugStream (debug);
    agent.SetAllModuleMode (TModuleInterface::mmDebug);
  }


  SaveAgentToFile (agent, outdir+"manoi01-before.agent");
  // return 0;

  {
    stringstream optss;
    if (option("help")!="")
      {cerr<<"valid options:"<<endl; option.PrintUsed(); return 0;}
    if (option.PrintNotAccessed(optss))
      {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}
  }

  //////////////////////////////////////////////////////
  /// start learning:
  ptr_environment= &environment;
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &ODEDS_Start;
  fn.step = &ODEDS_StepLoop;
  fn.command = &ODEDS_KeyEvent;
  fn.stop = &ODEDS_Stop;
  char path_to_textures[] = "materials/textures";
  fn.path_to_textures = path_to_textures;
  int xwindow_width(400), xwindow_height(400);

  lmanager.Initialize();
  lmanager.StartLearning();

  environment.SetConsoleMode(console_mode);
  while(environment.Executing())
  {
    if (!environment.ConsoleMode())
      {dsSimulationLoop (argc,argv,xwindow_width,xwindow_height,&fn);}
    else
      {while(environment.Executing()) environment.StepLoop();}
  }
  TerminateODE();
  //////////////////////////////////////////////////////

  /// result:

  SaveAgentToFile (agent, outdir+"manoi01-after.agent");

  return 0;
}
//-------------------------------------------------------------------------------------------

