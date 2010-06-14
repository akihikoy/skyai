//-------------------------------------------------------------------------------------------
/*! \file    dcob.cpp
    \brief   libskyai - DCOB module (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.08, 2010-

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
#include <skyai/modules_std/dcob.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MActionSetDCOB
//===========================================================================================

/*virtual*/void MActionSetDCOB::slot_execute_action_exec (const TDiscreteAction &a)
{
  const TInt NI (conf_.IntervalSet.length());
  const TInt interval = a % NI;
  const TInt target   = (a-interval) / NI;

  const TRealVector  &target_state (get_center_state_set()[target]);
  bf_trans_action_.resize (1+target_state.length());
  bf_trans_action_(0)= conf_.IntervalSet(interval);
  std::copy (GenBegin(target_state),GenEnd(target_state), GenBegin(bf_trans_action_)+1);

  signal_execute_command.ExecAll (bf_trans_action_);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MActionSetDCOB)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

