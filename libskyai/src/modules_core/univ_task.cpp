//-------------------------------------------------------------------------------------------
/*! \file    univ_task.cpp
    \brief   libskyai - universal task modules (source)
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
#include <skyai/modules_core/univ_task.h>
#include <lora/variable_literal.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//===========================================================================================
// class MUniversalContTimeTask
//===========================================================================================

#define DEF_SLOT(x_event,x_time_step_assign)   \
    if(conf_.F##x_event!="")                        \
    {                                               \
      if(!is_used())  return;                       \
      x_time_step_assign                            \
      if(conf_.SensingAt##x_event)                  \
        sense_at_##x_event();                       \
      std::list<var_space::TLiteral> argv;          \
      argv.push_back(var_space::LiteralId(InstanceName()));  \
      mem_.Reward= 0.0l;                            \
      mem_.EndOfEps= false;                         \
      if(!ExecuteFunction(conf_.F##x_event, argv))  \
        lexit(df);                                  \
      signal_reward.ExecAll(mem_.Reward);           \
      if(mem_.EndOfEps)                             \
        signal_end_of_episode.ExecAll();            \
    }

/*virtual*/void MUniversalContTimeTask::slot_start_episode_exec (void)
{
  DEF_SLOT(EpisodeStart,)
}
/*virtual*/void MUniversalContTimeTask::slot_finish_episode_exec (void)
{
  DEF_SLOT(EpisodeEnd,)
}

/*virtual*/void MUniversalContTimeTask::slot_start_of_action_exec (void)
{
  DEF_SLOT(ActionStart,)
}
/*virtual*/void MUniversalContTimeTask::slot_end_of_action_exec (void)
{
  DEF_SLOT(ActionEnd,)
}

/*virtual*/void MUniversalContTimeTask::slot_start_time_step_exec (const TReal &dt)
{
  DEF_SLOT(TimeStepStart,mem_.TimeStep=dt;)
}
/*virtual*/void MUniversalContTimeTask::slot_finish_time_step_exec (const TReal &dt)
{
  DEF_SLOT(TimeStepEnd,mem_.TimeStep=dt;)
}

#undef DEF_SLOT
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
