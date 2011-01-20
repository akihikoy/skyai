//-------------------------------------------------------------------------------------------
/*! \file    grid_action_space.cpp
    \brief   libskyai - define a grid action spcae (DEPRECATED)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.22, 2009-

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
#include <skyai/modules_std/grid_action_space.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace grid_action_space_detail
{
using namespace std;
// using namespace boost;

#define DEPRECATED_MSG(x_mod,x_alt)   \
  do{static int msg_count(20);        \
  if(msg_count) {--msg_count; LWARNING(x_mod << " is deprecated. Use " << x_alt << " instead." );}}while(0)


//===========================================================================================
// class MGridActionSpace
//===========================================================================================

override void MGridActionSpace::slot_initialize_exec (void)
{
DEPRECATED_MSG("MGridActionSpace", "MDiscretizer, MLCHolder_TRealVector, and a continuous action space");
  ltime_= INVALID_CONT_TIME;
  grid_.Init (GenBegin(conf_.Levels), GenEnd(conf_.Levels),
              GenBegin(conf_.ActionMin),
              GenBegin(conf_.ActionMax));
  current_command_.resize (grid_.Digits());
  // LDBGVAR(grid_.Size());
}
//-------------------------------------------------------------------------------------------

override void MGridActionSpace::slot_execute_action_exec (const TAction &a)
{
DEPRECATED_MSG("MGridActionSpace", "MDiscretizer, MLCHolder_TRealVector, and a continuous action space");
  grid_.IntToContVector (a, GenBegin(current_command_));
  ltime_= conf_.Interval;
}
//-------------------------------------------------------------------------------------------

override void MGridActionSpace::slot_start_time_step_exec (const TContinuousTime &time_step)
{
DEPRECATED_MSG("MGridActionSpace", "MDiscretizer, MLCHolder_TRealVector, and a continuous action space");
  if (ltime_<=0.0l)  return;
  signal_execute_command.ExecAll(current_command_);
}
//-------------------------------------------------------------------------------------------

override void MGridActionSpace::slot_finish_time_step_exec (const TContinuousTime &time_step)
{
DEPRECATED_MSG("MGridActionSpace", "MDiscretizer, MLCHolder_TRealVector, and a continuous action space");
  if (ltime_<=0.0l)  return;
  ltime_-= time_step;
  if (ltime_<=0.0l)
  {
    signal_end_of_action.ExecAll();
  }
}
//-------------------------------------------------------------------------------------------

override void MGridActionSpace::slot_finish_action_immediately_exec (void)
{
DEPRECATED_MSG("MGridActionSpace", "MDiscretizer, MLCHolder_TRealVector, and a continuous action space");
  if (ltime_<=0.0l)  return;

  ltime_= -1.0l;
  signal_end_of_action.ExecAll();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MGridActionSpace)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of grid_action_space_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

