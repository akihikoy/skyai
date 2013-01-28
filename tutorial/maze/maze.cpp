//-------------------------------------------------------------------------------------------
/*! \file    maze.cpp
    \brief   skyai - maze tutorial
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
#include <lora/variable_space_impl.h>  // to store std::vector<TIntVector>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//===========================================================================================
class TMazeTaskConfigurations
//===========================================================================================
{
public:

  int   NumEpisodes;     //!< number of episodes
  int   MaxSteps;        //!< number of max steps per episode
  int   StartX, StartY;  //!< start position

  double GoalReward;
  double StepCost;

  int   SleepUTime;

  std::vector<std::vector<int> >   Map;  //!< Map[y][x], 0:free space, 1:wall, 2:goal, every element should have the same size

  TMazeTaskConfigurations (var_space::TVariableMap &mmap) :
      NumEpisodes   (1000),
      MaxSteps      (1000),
      StartX        (1),
      StartY        (1),
      GoalReward    (1.0),
      StepCost      (-0.01),
      SleepUTime    (1000)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( NumEpisodes );
      ADD( StartX );
      ADD( StartY );
      ADD( GoalReward );
      ADD( StepCost );
      ADD( SleepUTime );
      ADD( Map );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief Maze task (environment+task) module
class MMazeTaskModule
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface TParent;
  typedef MMazeTaskModule  TThis;
  SKYAI_MODULE_NAMES(MMazeTaskModule)

  MMazeTaskModule (const std::string &v_instance_name)
    : TParent          (v_instance_name),
      conf_            (TParent::param_box_config_map()),
      state_set_size_  (0),
      action_set_size_ (4),
      current_action_  (0),
      slot_start              (*this),
      slot_execute_action     (*this),
      signal_initialization   (*this),
      signal_start_of_episode (*this),
      signal_finish_episode   (*this),
      signal_end_of_episode   (*this),
      signal_start_of_step    (*this),
      signal_end_of_step      (*this),
      signal_reward           (*this),
      out_state_set_size      (*this),
      out_action_set_size     (*this),
      out_state               (*this),
      out_time                (*this)
    {
      add_slot_port   (slot_start              );
      add_slot_port   (slot_execute_action     );
      add_signal_port (signal_initialization   );
      add_signal_port (signal_start_of_episode );
      add_signal_port (signal_finish_episode   );
      add_signal_port (signal_end_of_episode   );
      add_signal_port (signal_start_of_step    );
      add_signal_port (signal_end_of_step      );
      add_signal_port (signal_reward           );
      add_out_port    (out_state_set_size      );
      add_out_port    (out_action_set_size     );
      add_out_port    (out_state               );
      add_out_port    (out_time                );
    }

  void Start()
    {
      slot_start.Exec();
    }

protected:

  TMazeTaskConfigurations  conf_;

  mutable int state_set_size_;
  const int action_set_size_;
  int  current_action_;
  int  pos_x_, pos_y_;

  mutable int tmp_state_;
  TReal current_time_;
  TInt  num_episode_;

  MAKE_SLOT_PORT(slot_start, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_execute_action, void, (const TInt &a), (a), TThis);

  MAKE_SIGNAL_PORT(signal_initialization, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_start_of_episode, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_finish_episode, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_episode, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_start_of_step, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_end_of_step, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_reward, void (const TSingleReward &), TThis);

  MAKE_OUT_PORT(out_state_set_size, const TInt&, (void), (), TThis);
  MAKE_OUT_PORT(out_action_set_size, const TInt&, (void), (), TThis);
  MAKE_OUT_PORT(out_state, const TInt&, (void), (), TThis);
  MAKE_OUT_PORT(out_time, const TReal&, (void), (), TThis);

  virtual void slot_start_exec (void);

  virtual void slot_execute_action_exec (const TInt &a)
    {
      current_action_= a;
    }

  virtual const TInt& out_state_set_size_get (void) const
    {
      state_set_size_= conf_.Map[0].size() * conf_.Map.size();
      return state_set_size_;
    }

  virtual const TInt& out_action_set_size_get (void) const
    {
      return action_set_size_;
    }

  virtual const TInt& out_state_get (void) const
    {
      return tmp_state_=serialize(pos_x_,pos_y_);
    }

  virtual const TReal& out_time_get (void) const
    {
      return current_time_;
    }

  int serialize (int x, int y) const
    {
      return y * conf_.Map[0].size() + x;
    }

  void init_environment (void);
  bool step_environment (void);
  void show_environment (void);

};  // end of MMazeTaskModule
//-------------------------------------------------------------------------------------------

/*virtual*/void MMazeTaskModule::slot_start_exec (void)
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
      signal_start_of_step.ExecAll();

      running= step_environment();
      show_environment();
      usleep(conf_.SleepUTime);

      if(current_time_>=conf_.MaxSteps)
      {
        signal_finish_episode.ExecAll();
        running= false;
      }
      signal_end_of_step.ExecAll();
    }

    signal_end_of_episode.ExecAll();
  }
}
//-------------------------------------------------------------------------------------------

void MMazeTaskModule::init_environment (void)
{
  pos_x_= conf_.StartX;
  pos_y_= conf_.StartY;
  current_time_= 0.0l;
}
//-------------------------------------------------------------------------------------------

bool MMazeTaskModule::step_environment (void)
{
  int next_x(pos_x_), next_y(pos_y_);
  switch(current_action_)
  {
  case 0: ++next_x; break;  // right
  case 1: --next_y; break;  // up
  case 2: --next_x; break;  // left
  case 3: ++next_y; break;  // down
  default: LERROR("invalid action:"<<current_action_);
  }

  ++current_time_;
  signal_reward.ExecAll(conf_.StepCost);

  switch(conf_.Map[next_y][next_x])
  {
  case 0:  // free space
    pos_x_=next_x;
    pos_y_=next_y;
    break;
  case 1:  // wall
    break;
  case 2:  // goal
    pos_x_=next_x;
    pos_y_=next_y;
    signal_reward.ExecAll(conf_.GoalReward);
    signal_finish_episode.ExecAll();
    return false;
  default: LERROR("invalid map element: "<<conf_.Map[next_y][next_x]);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

void MMazeTaskModule::show_environment (void)
{
  int x(0),y(0);
  std::cout<<"("<<pos_x_<<","<<pos_y_<<")  "<<current_time_<<"/"<<num_episode_<<std::endl;
  for(std::vector<std::vector<int> >::const_iterator yitr(conf_.Map.begin()),ylast(conf_.Map.end());yitr!=ylast;++yitr,++y)
  {
    x=0;
    for(std::vector<int>::const_iterator xitr(yitr->begin()),xlast(yitr->end());xitr!=xlast;++xitr,++x)
    {
      std::cout<<" ";
      if(x==pos_x_ && y==pos_y_)
        std::cout<<"R";
      else if(x==conf_.StartX && y==conf_.StartY)
        std::cout<<"S";
      else
        switch(*xitr)
        {
        case 0:  std::cout<<" "; break;
        case 1:  std::cout<<"#"; break;
        case 2:  std::cout<<"G"; break;
        default: std::cout<<"?"; break;
        }
    }
    std::cout<<" "<<std::endl;
  }
  std::cout<<std::endl;
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
      slot_step      (*this),
      signal_action  (*this)
    {
      add_slot_port   (slot_step    );
      add_signal_port (signal_action);
    }

protected:

  MAKE_SLOT_PORT(slot_step, void, (void), (), TThis);

  MAKE_SIGNAL_PORT(signal_action, void (const TInt &), TThis);

  virtual void slot_step_exec (void)
    {
      signal_action.ExecAll(rand() % 4);
    }

};  // end of MRandomActionModule
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MMazeTaskModule)
SKYAI_ADD_MODULE(MRandomActionModule)
//-------------------------------------------------------------------------------------------


}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int TutMazeSkyAIMain(TOptionParser &option, TAgent &agent)
{
  MMazeTaskModule *p_maze_task = dynamic_cast<MMazeTaskModule*>(agent.SearchModule("maze_task"));
  if(p_maze_task==NULL)  {LERROR("module `maze_task' is not defined as an instance of MMazeTaskModule"); return 1;}

  agent.SaveToFile (agent.GetDataFileName("before.agent"),"before-");

  p_maze_task->Start();

  agent.SaveToFile (agent.GetDataFileName("after.agent"),"after-");

  return 0;
}
//-------------------------------------------------------------------------------------------
SKYAI_SET_MAIN(TutMazeSkyAIMain)
//-------------------------------------------------------------------------------------------
