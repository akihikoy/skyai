//-------------------------------------------------------------------------------------------
/*! \file    libhumanoid01.cpp
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
#include "libhumanoid01.h"
#include "detail/humanoid01.h"
#include <lora/variable_literal.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class THumanoidEnvironmentConfigurations
//===========================================================================================

THumanoidEnvironmentConfigurations::THumanoidEnvironmentConfigurations (var_space::TVariableMap &mmap)
  :
    TimeStep            (0.0002l),
    FPS                 (50.0l),
    ViewPoint           (6,0.0),
    UseInitPose         (false),
    PDGainKp                  (JOINT_NUM,2.5l),
    PDGainKd                  (JOINT_NUM,0.08l),
    TorqueMax                 (JOINT_NUM, loco_rabbits::TorqueMax),
    CommandMax                (0),
    CommandMin                (0),
    // parameters for simulationcnd:
    MaxContactNum             (10),
    BodyContactLPFParamF      (10.0),
    BodyContactLPFParamQ      (0.8),
    ForceInitFeetContactWithGround (false),
    UsingQuickStep                 (false),
    QuickStepIterationNum          (20),
    MazeScale                 (5.0),
    MazeMapKind               (-1)
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
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MHumanoidEnvironment
//===========================================================================================

MHumanoidEnvironment::MHumanoidEnvironment (const std::string &v_instance_name)
  :
    TParent                     (v_instance_name),
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
    out_base_pose               (*this),
    out_base_vel                (*this),
    out_base_rot                (*this),
    out_base_euler              (*this),
    out_base_atan1202           (*this),
    out_joint_angle             (*this),
    out_joint_vel               (*this),
    out_contact_with_ground     (*this),
    out_contact_with_object     (*this)
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
  add_out_port    (out_base_pose              );
  add_out_port    (out_base_vel               );
  add_out_port    (out_base_rot               );
  add_out_port    (out_base_euler             );
  add_out_port    (out_base_atan1202          );
  add_out_port    (out_joint_angle            );
  add_out_port    (out_joint_vel              );
  add_out_port    (out_contact_with_ground    );
  add_out_port    (out_contact_with_object    );
}
//-------------------------------------------------------------------------------------------

void MHumanoidEnvironment::StepLoop (int ds_pause)
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
//-------------------------------------------------------------------------------------------

void MHumanoidEnvironment::ODEDS_Start()
{
  start();
  float xyzhpr[6];
  for(int i(0); i<6; ++i)  xyzhpr[i]= static_cast<float>(conf_.ViewPoint(i));
  dsSetViewpoint (xyzhpr,xyzhpr+3);
  // xdsSetLightPos(-0.4f,-0.4f);
}
//-------------------------------------------------------------------------------------------

void MHumanoidEnvironment::ODEDS_KeyEvent (int cmd)
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
  else if (cmd=='a')
  {
    AUTO_VIEWPOINT_MODE=(AUTO_VIEWPOINT_MODE==GetAutoViewpointModeCount())?0:AUTO_VIEWPOINT_MODE+1;
    LMESSAGE("AUTO_VIEWPOINT_MODE= "<<AUTO_VIEWPOINT_MODE<<".");
  }
  else if (cmd=='A')
  {
    cout<<"setting viewpoint mode..."<<endl;
    cout<<"  0: manual view"<<endl;
    cout<<"  1: following x-position of robot"<<endl;
    cout<<"  2: following y-position of robot"<<endl;
    cout<<"  3: tracking robot from static camera position"<<endl;
    cout<<"  4: following x,y-position of robot (camera is just above the robot)"<<endl;
    cout<<"input number: ";
    cin>>AUTO_VIEWPOINT_MODE;
    AUTO_VIEWPOINT_MODE=ApplyRange(AUTO_VIEWPOINT_MODE,0,GetAutoViewpointModeCount());
    LMESSAGE("AUTO_VIEWPOINT_MODE= "<<AUTO_VIEWPOINT_MODE<<".");
  }
  else if (cmd=='v' || cmd=='V')
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
//-------------------------------------------------------------------------------------------

void MHumanoidEnvironment::set_global_config (void)
{
  // copy parameters to simulationcnd
  simulationcnd.MaxContactNum                   = conf_.MaxContactNum                   ;
  simulationcnd.Surface                         = conf_.Surface                         ;
  simulationcnd.BodyContactLPFParamF            = conf_.BodyContactLPFParamF            ;
  simulationcnd.BodyContactLPFParamQ            = conf_.BodyContactLPFParamQ            ;
  simulationcnd.ForceInitFeetContactWithGround  = conf_.ForceInitFeetContactWithGround  ;
  simulationcnd.UsingQuickStep                  = conf_.UsingQuickStep                  ;
  simulationcnd.QuickStepIterationNum           = conf_.QuickStepIterationNum           ;

  MAZESCALE = conf_.MazeScale;
  MAP_KIND  = conf_.MazeMapKind;
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MHumanoidEnvironment::slot_initialize_exec (void)
{
  set_global_config();

  InitializeODE();
  initSimulation2();
  LMESSAGE("start simulation..");
  SetupODE();

  executing_= true;
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MHumanoidEnvironment::slot_start_episode_exec (void)
{
  set_global_config();

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
/*TEST*/current_command_.resize(JOINT_NUM);
/*TEST*/std::fill(GenBegin(current_command_),GenEnd(current_command_),0.0);
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MHumanoidEnvironment::slot_execute_command_des_qd_exec (const TRealVector &u)
{
  //!\todo FIXME: make efficient computation!
  GetJointAngle(tmp_joint_angle_);
  current_command_= tmp_joint_angle_ + u;
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MHumanoidEnvironment::slot_step_loop_exec (void)
{
  signal_start_of_timestep.ExecAll(conf_.TimeStep);

  /*dbg*/if (current_command_.length()==0)  {LERROR("slot_execute_command_des_q is not called!!!");}

  ConstrainVector(current_command_, conf_.CommandMin, conf_.CommandMax);

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
//-------------------------------------------------------------------------------------------

/*virtual*/const TContinuousState& MHumanoidEnvironment::out_base_pose_get() const
{
  GenResize(tmp_base_pose_,POSROT_DIM);
  GetBasePosRot(tmp_base_pose_);
  return tmp_base_pose_;
}
/*virtual*/const TContinuousState& MHumanoidEnvironment::out_base_vel_get() const
{
  GenResize(tmp_base_vel_,PRVEL_DIM);
  GetBaseVel(tmp_base_vel_);
  return tmp_base_vel_;
}
/*virtual*/const TRealMatrix& MHumanoidEnvironment::out_base_rot_get() const
{
  tmp_base_rot_.resize(3,3);
  GetRot(0,tmp_base_rot_);
  return tmp_base_rot_;
}
/*virtual*/const TContinuousState& MHumanoidEnvironment::out_base_euler_get() const
{
  GenResize(tmp_base_euler_, 3);
  tmp_base_euler_(0)= GetRoll(0);
  tmp_base_euler_(1)= GetPitch(0);
  tmp_base_euler_(2)= GetYaw(0);
  return tmp_base_euler_;
}
/*virtual*/const TContinuousState& MHumanoidEnvironment::out_base_atan1202_get() const
{
  GenResize(tmp_base_atan1202_, 1);
  tmp_base_atan1202_(0)= GetAtan1202(0);
  return tmp_base_atan1202_;
}
//-------------------------------------------------------------------------------------------

/*virtual*/const TContinuousState& MHumanoidEnvironment::out_joint_angle_get() const
{
  GenResize(tmp_joint_angle_,JOINT_NUM);
  GetJointAngle(tmp_joint_angle_);
  return tmp_joint_angle_;
}
/*virtual*/const TContinuousState& MHumanoidEnvironment::out_joint_vel_get() const
{
  GenResize(tmp_joint_vel_,JOINT_NUM);
  GetJointAngVel(tmp_joint_vel_);
  return tmp_joint_vel_;
}
//-------------------------------------------------------------------------------------------

/*virtual*/const TBoolVector& MHumanoidEnvironment::out_contact_with_ground_get() const
{
  GenResize(tmp_contact_with_ground_,BODY_NUM);
  bodies_contact_with_ground (tmp_contact_with_ground_);
  return tmp_contact_with_ground_;
}
/*virtual*/const TBoolVector& MHumanoidEnvironment::out_contact_with_object_get() const
{
  GenResize(tmp_contact_with_object_,BODY_NUM);
  bodies_contact_with_object (tmp_contact_with_object_);
  return tmp_contact_with_object_;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MHumanoidEnvironment)
SKYAI_ADD_MODULE(MHumanoidUnivTask)
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


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

