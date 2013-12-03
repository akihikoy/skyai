//-------------------------------------------------------------------------------------------
/*! \file    fitted_qi_ls.cpp
    \brief   libskyai - Fitted Q Iteration implementation with a gradient descent method for the least square error
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Apr.13, 2010-

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
#include <skyai/modules_std/fitted_qi_ls.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;
namespace fitted_qi_detail
{


//===========================================================================================
// template <typename t_state, typename t_action>
// class MFittedQIterationSL
//===========================================================================================

#define TEMPLATE_DEC  template <typename t_state, typename t_action>
#define XMODULE       MFittedQIterationSL <t_state, t_action>
#define XMODULE_STR  "MFittedQIterationSL <t_state, t_action>"

//===========================================================================================
// MFittedQIterationSL implementation of protected member functions
//===========================================================================================

TEMPLATE_DEC inline TReal XMODULE::get_alpha (void) const
{
  TReal alpha (0.0l);
  alpha= sl_conf_.Alpha * exp (-sl_conf_.AlphaDecreasingFactor * static_cast<TReal>(mem_.EpisodeNumber));
  return ApplyRange(alpha,sl_conf_.AlphaMin,sl_conf_.Alpha);
}
//-------------------------------------------------------------------------------------------

//! execute this function to start new trial
TEMPLATE_DEC
/*override*/ void XMODULE::reset_episode ()
{
  TParent::reset_episode();

  difference_.Allocate();
  gradient_.Allocate();
}
//-------------------------------------------------------------------------------------------

//! supervised learning iteration
TEMPLATE_DEC
/*override*/ void XMODULE::exec_supervised_learning (int number_of_used_samples)
{
  const TReal alpha (get_alpha());

  if (sl_conf_.LSMethodType == mtBatch)
  {
    for (int NSL(0); NSL<sl_conf_.MaxNumberOfSLIteration; ++NSL)
    {
      get_avf_zero_parameter (difference_());
      get_avf_zero_parameter (gradient_());
      TValue sum_tderror(0.0l), tderror;
      TValue action_value;
      TStateActionAttribute attrib;
      attrib.ActionValue= &action_value;
      attrib.Gradient= &(gradient_());
      for (typename TFQIData<TState,TAction>::T::i_iterator itr(fqi_data_.ibegin(-number_of_used_samples)); itr!=fqi_data_.iend(); ++itr)
      {
        if (itr->IsTerminal)  continue;
        get_avf_evaluate (itr->State, itr->Action, attrib);
        tderror= itr->StateValue - action_value;
        sum_tderror+= real_fabs(tderror);
        difference_().AddProd (tderror, gradient_());
      }
      TReal sqerr_grad_norm(difference_().Norm());
      if (sl_conf_.SqErrGradientNormLimit > 0.0l && sqerr_grad_norm > sl_conf_.SqErrGradientNormLimit)
        difference_()*= alpha * sl_conf_.SqErrGradientNormLimit/sqerr_grad_norm;
      else
        difference_()*= alpha;
      signal_avf_add_to_parameter.ExecAll (difference_());
LDBGVAR(sqerr_grad_norm);
LDBGVAR(sum_tderror);
    }  // SL-iteration
  }  // sl_conf_.LSMethodType == mtBatch
  else if (sl_conf_.LSMethodType == mtPointByPoint)
  {
    for (int NSL(0); NSL<sl_conf_.MaxNumberOfSLIteration; ++NSL)
    {
      get_avf_zero_parameter (difference_());
      get_avf_zero_parameter (gradient_());
      TValue sum_tderror(0.0l), tderror;
      TValue action_value;
      TStateActionAttribute attrib;
      attrib.ActionValue= &action_value;
      attrib.Gradient= &(gradient_());
      for (typename TFQIData<TState,TAction>::T::i_iterator itr(fqi_data_.ibegin(-number_of_used_samples)); itr!=fqi_data_.iend(); ++itr)
      {
        if (itr->IsTerminal)  continue;
        get_avf_evaluate (itr->State, itr->Action, attrib);
        tderror= itr->StateValue - action_value;
        sum_tderror+= real_fabs(tderror);
        difference_()= gradient_();

        TReal sqerr_grad_norm(tderror * gradient_().Norm());
        if (sl_conf_.SqErrGradientNormLimit > 0.0l && sqerr_grad_norm > sl_conf_.SqErrGradientNormLimit)
          difference_()*= alpha* sl_conf_.SqErrGradientNormLimit/sqerr_grad_norm;
        else
          difference_()*= alpha*tderror;
        signal_avf_add_to_parameter.ExecAll (difference_());
      }
LDBGVAR(sum_tderror);
    }  // SL-iteration
  }  // sl_conf_.LSMethodType == mtPointByPoint
}
//-------------------------------------------------------------------------------------------


#undef XMODULE_STR
#undef XMODULE
#undef TEMPLATE_DEC
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationSL,TContinuousState,TContinuousAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationSL,TContinuousState,TDiscreteAction)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFittedQIterationSL,TCompositeState,TCompositeAction)

SKYAI_ADD_MODULE(MFittedQIterationSL_TContinuousState_TContinuousAction)
SKYAI_ADD_MODULE(MFittedQIterationSL_TContinuousState_TDiscreteAction)
SKYAI_ADD_MODULE(MFittedQIterationSL_TCompositeState_TCompositeAction)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of fitted_qi_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

