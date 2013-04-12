//-------------------------------------------------------------------------------------------
/*! \file    mountain_car.cpp
    \brief   skyai - mountain car tutorial
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.12, 2012

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
#include <skyai/skyai.h>
#include <skyai/execs/general_agent.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//===========================================================================================
class TMountainCarTaskConfigurations
//===========================================================================================
{
public:

  int    NumEpisodes;     //!< number of episodes
  double TimeStep;        //!< time-step
  double MaxTime;         //!< max time per episode

  double Gravity;
  double Mass;    //!< mass of car
  double Fric;    //!< friction

  int DispWidth;
  int DispHeight;

  int   SleepUTime;

  TMountainCarTaskConfigurations (var_space::TVariableMap &mmap) :
      NumEpisodes (200),
      TimeStep    (0.01),
      MaxTime     (100.0),
      Gravity     (9.8),
      Mass        (0.2),
      Fric        (0.3),
      DispWidth   (40),
      DispHeight  (15),
      SleepUTime  (1000)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( NumEpisodes );
      ADD( TimeStep    );
      ADD( MaxTime     );
      ADD( Gravity     );
      ADD( Mass        );
      ADD( Fric        );
      ADD( DispWidth   );
      ADD( DispHeight  );
      ADD( SleepUTime  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Mountain Car task (environment+task) module
class MMountainCarTaskModule
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface        TParent;
  typedef MMountainCarTaskModule  TThis;
  SKYAI_MODULE_NAMES(MMountainCarTaskModule)

  MMountainCarTaskModule (const std::string &v_instance_name)
    : TParent          (v_instance_name),
      conf_            (TParent::param_box_config_map()),
      slot_start              (*this),
      slot_execute_action     (*this),
      signal_initialization   (*this),
      signal_start_of_episode (*this),
      signal_finish_episode   (*this),
      signal_end_of_episode   (*this),
      signal_start_of_timestep(*this),
      signal_end_of_timestep  (*this),
      signal_reward           (*this),
      out_state               (*this),
      out_cont_time           (*this)
    {
      add_slot_port   (slot_start              );
      add_slot_port   (slot_execute_action     );
      add_signal_port (signal_initialization   );
      add_signal_port (signal_start_of_episode );
      add_signal_port (signal_finish_episode   );
      add_signal_port (signal_end_of_episode   );
      add_signal_port (signal_start_of_timestep);
      add_signal_port (signal_end_of_timestep  );
      add_signal_port (signal_reward           );
      add_out_port    (out_state               );
      add_out_port    (out_cont_time           );
    }

  void Start()
    {
      slot_start.Exec();
    }

protected:

  TMountainCarTaskConfigurations  conf_;

  TRealVector  accel_;  //!< 1-dim acceleration
  TRealVector  state_;  //!< position, velocity

  TReal time_;
  TInt  num_episode_;

  MAKE_SLOT_PORT(slot_start, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_execute_action, void, (const TRealVector &a), (a), TThis);

  MAKE_SIGNAL_PORT(signal_initialization, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_start_of_episode, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_finish_episode, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_start_of_timestep, void (const TReal &dt), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_timestep, void (const TReal &dt), TThis);
  MAKE_SIGNAL_PORT(signal_reward, void (const TSingleReward &), TThis);

  MAKE_OUT_PORT(out_state, const TRealVector&, (void), (), TThis);
  MAKE_OUT_PORT(out_cont_time, const TReal&, (void), (), TThis);

  virtual void slot_start_exec (void);

  virtual void slot_execute_action_exec (const TRealVector &a)
    {
      accel_= a;
    }

  virtual const TRealVector& out_state_get (void) const
    {
      return state_;
    }

  virtual const TReal& out_cont_time_get (void) const
    {
      return time_;
    }

  void init_environment (void);
  bool step_environment (void);
  void show_environment (void);

};  // end of MMountainCarTaskModule
//-------------------------------------------------------------------------------------------

/*virtual*/void MMountainCarTaskModule::slot_start_exec (void)
{
  init_environment();
  signal_initialization.ExecAll();

  for(num_episode_=0; num_episode_<conf_.NumEpisodes; ++num_episode_)
  {
    init_environment();

    signal_start_of_episode.ExecAll();

    bool running(true);
    while(running)
    {
      signal_start_of_timestep.ExecAll(conf_.TimeStep);

      running= step_environment();
      show_environment();
      usleep(conf_.SleepUTime);

      if(time_>=conf_.MaxTime)
      {
        signal_finish_episode.ExecAll();
        running= false;
      }
      signal_end_of_timestep.ExecAll(conf_.TimeStep);
    }

    signal_end_of_episode.ExecAll();
  }
}
//-------------------------------------------------------------------------------------------

void MMountainCarTaskModule::init_environment (void)
{
  state_.resize(2);
  state_(0)= -0.5;
  state_(1)= 0.0;
  accel_.resize(1);
  accel_(0)= 0.0;
  time_= 0.0l;
}
//-------------------------------------------------------------------------------------------

bool MMountainCarTaskModule::step_environment (void)
{
  state_(1)= state_(1) + (-conf_.Gravity*conf_.Mass*std::cos(3.0*state_(0))+accel_(0)/conf_.Mass-conf_.Fric*state_(1))*conf_.TimeStep;
  state_(0)= state_(0) + state_(1)*conf_.TimeStep;
  time_+= conf_.TimeStep;

  TReal reward= 0.1l*(1.0l / (1.0l + Square(0.6l-state_(0))) - 1.0l);
  signal_reward.ExecAll(reward);

  if(state_(0)<=-1.2)
  {
    state_(0)=-1.2;
    state_(1)=0.0;
  }

  if(state_(0)>=0.6)
  {
    signal_finish_episode.ExecAll();
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

void MMountainCarTaskModule::show_environment (void)
{
  std::cout<<"("<<state_(0)<<","<<state_(1)<<"), "<<accel_(0)<<", "<<time_<<"/"<<num_episode_<<std::endl;
  std::vector<int> curve(conf_.DispWidth);
  for(int x(0);x<conf_.DispWidth;++x)
  {
    double rx= (0.6+1.2)*x/static_cast<TReal>(conf_.DispWidth)-1.2;
    curve[x]= static_cast<TReal>(conf_.DispHeight-1)*0.5*(1.0-sin(3.0*rx))+1;
    std::cout<<"-";
  }
  std::cout<<std::endl;
  int pos= static_cast<TReal>(conf_.DispWidth)*(state_(0)+1.2)/(0.6+1.2);
  for(int y(0);y<conf_.DispHeight;++y)
  {
    for(int x(0);x<conf_.DispWidth;++x)
    {
      if(x==pos && y==curve[x]-1)  std::cout<<"#";
      else if(x==conf_.DispWidth-1 && y==curve[x]-1)  std::cout<<"G";
      else if(y>=curve[x] || x==0)  std::cout<<"^";
      else std::cout<<" ";
    }
    std::cout<<std::endl;
  }
  for(int x(0);x<conf_.DispWidth;++x)  std::cout<<"-";
  std::cout<<std::endl<<std::endl;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Random action module
class MRandomActionModule
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MRandomActionModule  TThis;
  SKYAI_MODULE_NAMES(MRandomActionModule)

  MRandomActionModule (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_timestep  (*this),
      signal_action  (*this)
    {
      add_slot_port   (slot_timestep);
      add_signal_port (signal_action);
    }

protected:

  MAKE_SLOT_PORT(slot_timestep, void, (const TReal &dt), (dt), TThis);

  MAKE_SIGNAL_PORT(signal_action, void (const TRealVector &), TThis);

  virtual void slot_timestep_exec (const TReal &dt)
    {
      static int time(0);
      static TRealVector a(1);
      if(time%50==0)
        switch(rand() % 3)
        {
        case 0: a(0)=0.0;  break;
        case 1: a(0)=+0.2;  break;
        case 2: a(0)=-0.2;  break;
        }
      signal_action.ExecAll(a);
      ++time;
    }

};  // end of MRandomActionModule
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MMountainCarTaskModule)
SKYAI_ADD_MODULE(MRandomActionModule)
//-------------------------------------------------------------------------------------------


}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int TutMountainCarSkyAIMain(TOptionParser &option, TAgent &agent)
{
  MMountainCarTaskModule *p_mountaincar_task = dynamic_cast<MMountainCarTaskModule*>(agent.SearchModule("mountaincar_task"));
  if(p_mountaincar_task==NULL)  {LERROR("module `mountaincar_task' is not defined as an instance of MMountainCarTaskModule"); return 1;}

  agent.SaveToFile (agent.GetDataFileName("before.agent"),"before-");

  p_mountaincar_task->Start();

  agent.SaveToFile (agent.GetDataFileName("after.agent"),"after-");

  return 0;
}
//-------------------------------------------------------------------------------------------
SKYAI_SET_MAIN(TutMountainCarSkyAIMain)
//-------------------------------------------------------------------------------------------
