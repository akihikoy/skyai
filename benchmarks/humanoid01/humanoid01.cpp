//-------------------------------------------------------------------------------------------
/*! \file    maze2d.cpp
    \brief   benchmarks - motion learning task of a simulation humanoid robot
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
#include <skyai/utility.h>
#include <skyai/modules_core/learning_manager.h>
#include <lora/variable_space_impl.h>
#include <lora/ctrl_tools.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{


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
      PDGainKp                 (JOINT_NUM,2.5l),
      PDGainKd                 (JOINT_NUM,0.08l),
      TorqueMax                (JOINT_NUM, loco_rabbits::TorqueMax),
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

      ADD( PDGainKp                       );
      ADD( PDGainKd                       );
      ADD( TorqueMax                      );

      // parameters for TSimulationCondition simulationcnd
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
      PDC_                        (JOINT_NUM, 0.0, 0.0, 0.0),
      tq_input_                   (JOINT_NUM, 0.0),
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
      out_base_pose               (*this),
      out_base_vel                (*this),
      out_joint_angle             (*this),
      out_joint_vel               (*this),
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
      add_out_port    (out_base_pose              );
      add_out_port    (out_base_vel               );
      add_out_port    (out_joint_angle            );
      add_out_port    (out_joint_vel              );
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

  TPDController       PDC_;
  TRealVector         current_command_;
  TRealVector         tq_input_;
//   mutable TContinuousState  tmp_state_;
  mutable TContinuousState  tmp_joint_state_;
  mutable TContinuousState  tmp_base_pose_, tmp_base_vel_, tmp_joint_angle_, tmp_joint_vel_;
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

  //!\brief output base link pose (position and rotation) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_pose, const TContinuousState&, (void), (), TThis);
  //!\brief output base link velocities (of position and rotation) according to the controller's constraint mode
  MAKE_OUT_PORT(out_base_vel, const TContinuousState&, (void), (), TThis);

  //!\brief output joint angles according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_angle, const TContinuousState&, (void), (), TThis);
  //!\brief output joint angular velocities according to the controller's constraint mode
  MAKE_OUT_PORT(out_joint_vel, const TContinuousState&, (void), (), TThis);

  MAKE_OUT_PORT(out_contact_with_ground, const TBoolVector&, (void), (), TThis);

  virtual void slot_initialize_exec (void)
    {
      // copy parameters to simulationcnd
      simulationcnd.MaxContactNum                   = conf_.MaxContactNum                   ;
      simulationcnd.Surface                         = conf_.Surface                         ;
      simulationcnd.BodyContactLPFParamF            = conf_.BodyContactLPFParamF            ;
      simulationcnd.BodyContactLPFParamQ            = conf_.BodyContactLPFParamQ            ;
      simulationcnd.ForceInitFeetContactWithGround  = conf_.ForceInitFeetContactWithGround  ;
      simulationcnd.UsingQuickStep                  = conf_.UsingQuickStep                  ;
      simulationcnd.QuickStepIterationNum           = conf_.QuickStepIterationNum           ;

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
        TODEJointAngleMap angle_map;
        for(int j(0); j<JOINT_NUM; ++j)  angle_map[joint[j].id()]= TODEAngle(conf_.InitJointAngles(j));
        ODESetArticulatedBodyJointAngles (body[0].id(), angle_map);
      }
      PDC_.Kp   = conf_.PDGainKp;
      PDC_.Kd   = conf_.PDGainKd;
      PDC_.UMax = conf_.TorqueMax;
    }

  virtual void slot_execute_command_des_q_exec (const TRealVector &u)
    {
      current_command_= u;
    }
  virtual void slot_execute_command_des_qd_exec (const TRealVector &u)
    {
      //!\todo FIXME: make efficient computation!
      GetJointAngle(tmp_joint_angle_);
      current_command_= tmp_joint_angle_ + u;
    }

  virtual void slot_step_loop_exec (void)
    {
      signal_start_of_timestep.ExecAll(conf_.TimeStep);

      /*dbg*/if (current_command_.length()==0)  {LERROR("slot_execute_command_des_qd is not called!!!");}

      ////////////////////////
      GetJointState(tmp_joint_state_);
      tq_input_= PDC_(tmp_joint_state_, current_command_);
      TSingleReward step_cost(0.0l);
      worldStep (tq_input_, conf_.TorqueMax, conf_.TimeStep, step_cost);
      if (ModuleMode()==TModuleInterface::mmDebug)
        {DebugStream()<<"HUMANOID(slot_step_loop: "<<&slot_step_loop<<"):  WorldStep is executed (cost="<<step_cost<<")"<<std::endl;}
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

  virtual const TContinuousState& out_base_pose_get() const
    {
      GenResize(tmp_base_pose_,POSROT_DIM);
      GetBasePosRot(tmp_base_pose_);
      return tmp_base_pose_;
    }
  virtual const TContinuousState& out_base_vel_get() const
    {
      GenResize(tmp_base_vel_,PRVEL_DIM);
      GetBaseVel(tmp_base_vel_);
      return tmp_base_vel_;
    }

  virtual const TContinuousState& out_joint_angle_get() const
    {
      GenResize(tmp_joint_angle_,JOINT_NUM);
      GetJointAngle(tmp_joint_angle_);
      return tmp_joint_angle_;
    }
  virtual const TContinuousState& out_joint_vel_get() const
    {
      GenResize(tmp_joint_vel_,JOINT_NUM);
      GetJointAngVel(tmp_joint_vel_);
      return tmp_joint_vel_;
    }

  virtual const TBoolVector& out_contact_with_ground_get() const
    {
      GenResize(tmp_contact_with_ground_,BODY_NUM);
      bodies_contact_with_ground (tmp_contact_with_ground_);
      return tmp_contact_with_ground_;
    }

};  // end of MHumanoidEnvironment
//-------------------------------------------------------------------------------------------


SKYAI_ADD_MODULE(MHumanoidEnvironment)

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
  bool console_mode= ConvertFromStr<bool>(option("console","false"));

  TAgent  agent;
  std::ofstream debug;
  if (!ParseCmdLineOption (agent, option, debug))  return 0;

  MBasicLearningManager *p_lmanager = dynamic_cast<MBasicLearningManager*>(agent.SearchModule("lmanager"));
  MHumanoidEnvironment *p_environment = dynamic_cast<MHumanoidEnvironment*>(agent.SearchModule("environment"));
  if(p_lmanager==NULL)  {LERROR("module `lmanager' is not defined correctly"); return 1;}
  if(p_environment==NULL)  {LERROR("module `environment' is not defined correctly"); return 1;}
  MBasicLearningManager &lmanager(*p_lmanager);
  MHumanoidEnvironment &environment(*p_environment);

  agent.SaveToFile (agent.GetDataFileName("humanoid01-before.agent"));

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
  char path_to_textures[] = "m/textures";
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

  agent.SaveToFile (agent.GetDataFileName("humanoid01-after.agent"));

  return 0;
}
//-------------------------------------------------------------------------------------------

