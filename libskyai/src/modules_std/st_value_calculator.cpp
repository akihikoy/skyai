//-------------------------------------------------------------------------------------------
/*! \file    st_value_calculator.cpp
    \brief   libskyai - state/action value calculator
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jan.01, 2010-

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
#include <skyai/modules_std/st_value_calculator.h>
#include <lora/small_classes.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MStateValueCalculator1F
//===========================================================================================

/*virtual*/void MStateValueCalculator1F::slot_calculate_exec (void)
{
  TContinuousState  x (get_state());
  TRealVector phi;
  TValue V;
  state_values_.resize(conf_.Division+1);
  TReal dx= (conf_.Max-conf_.Min) / static_cast<TReal>(conf_.Division);
  x(conf_.Index)= conf_.Min;
  for (int i(0); i<=conf_.Division; ++i)
  {
    get_state_to_feature (x, phi);
    get_state_value (phi, V);
    state_values_(i)= V;
    x(conf_.Index)+= dx;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MStateValueCalculatorNF
//===========================================================================================

/*virtual*/void MStateValueCalculatorNF::slot_calculate_exec (void)
{
  if (conf_.Min.length()!=conf_.Max.length() || conf_.Min.length()==0)
    {LERROR("size disagreement!");  LDBGVAR(conf_.Min.length()); LDBGVAR(conf_.Max.length()); lexit(df);}
  if (static_cast<int>(conf_.Division.size())!=conf_.Max.length() && conf_.Division.size()!=1)
    {LERROR("conf_.Division.size()(="<<conf_.Division.size()<<") should be 1 or "<<conf_.Max.length());  lexit(df);}
  TGridGenerator    x_grid;
  if (static_cast<int>(conf_.Division.size())==conf_.Max.length())
  {
    x_grid.Init (GenBegin(conf_.Division), GenEnd(conf_.Division),
                 GenBegin(conf_.Min),
                 GenBegin(conf_.Max));
  }
  else  // conf_.Division.size()==1
  {
    TIntVector  divs (conf_.Max.length(), conf_.Division[1]);
    x_grid.Init (GenBegin(divs), GenEnd(divs),
                 GenBegin(conf_.Min),
                 GenBegin(conf_.Max));
  }
  TContinuousState  x (conf_.Max.length());
  TRealVector phi;
  TValue V;
  state_values_.resize (x_grid.Size(), x.length()+1, 0.0);
  int r(0);
  for (x_grid.Init(); x_grid.Cont(); x_grid.Increment(), ++r)
  {
    x_grid.CurrentContVector (GenBegin(x));
    get_state_to_feature (x, phi);
    get_state_value (phi, V);
    for (int c(0); c<x.length(); ++c)
      state_values_(r,c)= x(c);
    state_values_(r,x.length())= V;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MActionValueCalculatorNF
//===========================================================================================

/*virtual*/void MActionValueCalculatorNF::slot_calculate_exec (void)
{
  if (conf_.Min.length()!=conf_.Max.length() || conf_.Min.length()==0)
    {LERROR("size disagreement!");  LDBGVAR(conf_.Min.length()); LDBGVAR(conf_.Max.length()); lexit(df);}
  if (static_cast<int>(conf_.Division.size())!=conf_.Max.length() && conf_.Division.size()!=1)
    {LERROR("conf_.Division.size()(="<<conf_.Division.size()<<") should be 1 or "<<conf_.Max.length());  lexit(df);}
  TGridGenerator    x_grid;
  if (static_cast<int>(conf_.Division.size())==conf_.Max.length())
  {
    x_grid.Init (GenBegin(conf_.Division), GenEnd(conf_.Division),
                 GenBegin(conf_.Min),
                 GenBegin(conf_.Max));
  }
  else  // conf_.Division.size()==1
  {
    TIntVector  divs (conf_.Max.length(), conf_.Division[1]);
    x_grid.Init (GenBegin(divs), GenEnd(divs),
                 GenBegin(conf_.Min),
                 GenBegin(conf_.Max));
  }
  TContinuousState  x (conf_.Max.length());
  TRealVector phi;
  TRealVector Qs;

  get_state_to_feature (conf_.Min, phi);  // calculation to obtain the size of Qs
  get_action_value (phi, Qs);             // ditto
  action_values_.resize (x_grid.Size(), x.length()+Qs.length(), 0.0);
  int r(0);
  for (x_grid.Init(); x_grid.Cont(); x_grid.Increment(), ++r)
  {
    x_grid.CurrentContVector (GenBegin(x));
    get_state_to_feature (x, phi);
    get_action_value (phi, Qs);
    for (int c(0); c<x.length(); ++c)
      action_values_(r,c)= x(c);
    for (int c(0); c<Qs.length(); ++c)
      action_values_(r,c+x.length())= Qs(c);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MStateValueCalculator1F)
SKYAI_ADD_MODULE(MStateValueCalculatorNF)
SKYAI_ADD_MODULE(MActionValueCalculatorNF)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

