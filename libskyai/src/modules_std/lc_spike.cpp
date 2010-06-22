//-------------------------------------------------------------------------------------------
/*! \file    lc_spike.cpp
    \brief   libskyai - low-level controller: spike type  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Mar.08, 2010-

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
#include <skyai/modules_std/lc_spike.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MLCSpike
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_action>
#define XMODULE       MLCSpike <t_action>
#define XMODULE_STR  "MLCSpike <t_action>"

TEMPLATE_DEC
override void XMODULE::slot_execute_action_exec (const TAction &a)
{
  ltime_= conf_.Interval;
  signal_execute_command.ExecAll(a);
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


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLCSpike,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLCSpike,TRealVector)
SKYAI_ADD_MODULE(MLCSpike_TInt)
SKYAI_ADD_MODULE(MLCSpike_TRealVector)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
