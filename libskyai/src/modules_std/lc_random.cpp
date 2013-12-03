//-------------------------------------------------------------------------------------------
/*! \file    lc_random.cpp
    \brief   libskyai - low-level controller: random command generator  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.09, 2013

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
#include <skyai/modules_std/lc_random.h>
#include <lora/rand.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

template <typename t_command>
static void generate_random_command(t_command &out, const t_command &min, const t_command &max);
template <>
void generate_random_command<TInt>(TInt &out, const TInt &min, const TInt &max)
{
  out= Rand(min,max);
}
template <>
void generate_random_command<TRealVector>(TRealVector &out, const TRealVector &min, const TRealVector &max)
{
  LASSERT1op1(GenSize(min),==,GenSize(max));
  if(GenSize(out)!=GenSize(max))  GenResize(out,GenSize(max));
  TypeExt<TRealVector>::const_iterator min_itr(GenBegin(min));
  TypeExt<TRealVector>::const_iterator max_itr(GenBegin(max));
  for(TypeExt<TRealVector>::iterator o_itr(GenBegin(out)),o_last(GenEnd(out)); o_itr!=o_last; ++o_itr,++min_itr,++max_itr)
    *o_itr= Rand(*min_itr,*max_itr);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class MLCRandom
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_command>
#define XMODULE       MLCRandom <t_command>
#define XMODULE_STR  "MLCRandom <t_command>"

TEMPLATE_DEC
/*virtual*/void XMODULE::slot_execute_exec (void)
{
  active_= true;
  ltime_= -1.0l;
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/void XMODULE::slot_finish_exec (void)
{
  active_= false;

  if (ltime_<=0.0l)  return;
  ltime_= -1.0l;
  signal_end_of_command.ExecAll();
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/void XMODULE::slot_start_time_step_exec (const TContinuousTime &time_step)
{
  if (!active_)  return;
  if (ltime_<=0.0l)
  {
    ltime_= conf_.Interval;
    generate_random_command<TCommand>(current_command_,conf_.Min,conf_.Max);
    signal_start_of_command.ExecAll();
  }
  signal_execute_command.ExecAll(current_command_);
}
//-------------------------------------------------------------------------------------------

TEMPLATE_DEC
/*virtual*/void XMODULE::slot_finish_time_step_exec (const TContinuousTime &time_step)
{
  if (!active_)  return;
  ltime_-= time_step;
  if (ltime_<=0.0l)
  {
    signal_end_of_command.ExecAll();
  }
}
//-------------------------------------------------------------------------------------------

#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLCRandom,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLCRandom,TRealVector)
SKYAI_ADD_MODULE(MLCRandom_TInt)
SKYAI_ADD_MODULE(MLCRandom_TRealVector)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
