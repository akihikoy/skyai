//-------------------------------------------------------------------------------------------
/*! \file    lc_holder.cpp
    \brief   libskyai - low-level controller: holder type  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jan.14, 2010-

    Copyright (C) 2010  Akihiko Yamaguchi

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
#include <skyai/modules_std/lc_holder.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MLCHolder
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_action>
#define XMODULE       MLCHolder <t_action>
#define XMODULE_STR  "MLCHolder <t_action>"

TEMPLATE_DEC
override void XMODULE::slot_execute_action_exec (const TAction &a)
{
  ltime_= conf_.Interval;
  current_command_= a;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_start_time_step_exec (const TContinuousTime &time_step)
{
  if (ltime_<=0.0l)  return;
  signal_execute_command.ExecAll(current_command_);
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_finish_time_step_exec (const TContinuousTime &time_step)
{
  if (ltime_<=0.0l)  return;
  ltime_-= time_step;
  if (ltime_<=0.0l)
  {
    signal_end_of_action.ExecAll();
  }
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
override void XMODULE::slot_finish_action_immediately_exec (void)
{
  if (ltime_<=0.0l)  return;

  ltime_= -1.0l;
  signal_end_of_action.ExecAll();
}
//-------------------------------------------------------------------------------------------


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLCHolder,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLCHolder,TRealVector)
SKYAI_ADD_MODULE(MLCHolder_TInt)
SKYAI_ADD_MODULE(MLCHolder_TRealVector)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

