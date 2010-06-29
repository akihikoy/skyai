//-------------------------------------------------------------------------------------------
/*! \file    calculations.cpp
    \brief   libskyai - modules for basic calculations
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.25, 2010

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
#include <skyai/modules_core/calculations.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MConstMultiplier
//===========================================================================================

/*!\brief Specialization to Factor: real vector, x,y: real vector; y[i]=Factor[i]*x[i] for all i */
template<>void MConstMultiplier<TRealVector,TRealVector>::function (const TInput &x, TOutput &y) const
{
  GenResize(y, GenSize(x));
  TypeExt<TRealVector>::const_iterator iitr(GenBegin(x)), ilastitr(GenEnd(x)), fitr(GenBegin(conf_.Factor));
  if (GenSize(conf_.Factor)==GenSize(x))
  {
    for (TypeExt<TRealVector>::iterator oitr(GenBegin(y)); iitr!=ilastitr; ++iitr,++oitr,++fitr)
      (*oitr)= (*iitr) * (*fitr);
  }
  else if (GenSize(conf_.Factor)==1)
  {
    for (TypeExt<TRealVector>::iterator oitr(GenBegin(y)); iitr!=ilastitr; ++iitr,++oitr)
      (*oitr)= (*iitr) * (*fitr);
  }
  else
    {LERROR("size disagreement!"); LDBGVAR(GenSize(conf_.Factor)); LDBGVAR(GenSize(x)); lexit(df);}
}
//-------------------------------------------------------------------------------------------

/*!\brief Specialization to Factor: real matrix, x,y: real vector; y = Factor * x */
template<>void MConstMultiplier<TRealMatrix,TRealVector>::function (const TInput &x, TOutput &y) const
{
  y= conf_.Factor * x;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MElemSquarer
//===========================================================================================

/*!\brief Specialization to x,y: TInt; y=x^2 */
template<> void MElemSquarer<TInt>::function (const TInput &x, TOutput &y) const
{
  y= Square(x);
}
//-------------------------------------------------------------------------------------------

/*!\brief Specialization to x,y: TReal; y=x^2 */
template<> void MElemSquarer<TReal>::function (const TInput &x, TOutput &y) const
{
  y= Square(x);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// MBool{And,Or}
//===========================================================================================

void MBoolAnd::function (const TInput &x, TOutput &y) const
{
  for (TypeExt<TInput>::const_iterator x_itr(GenBegin(x)),x_last(GenEnd(x)); x_itr!=x_last; ++x_itr)
    if (!(*x_itr))  {y=false; return;}
  y= true;
}
//-------------------------------------------------------------------------------------------

void MBoolOr::function (const TInput &x, TOutput &y) const
{
  for (TypeExt<TInput>::const_iterator x_itr(GenBegin(x)),x_last(GenEnd(x)); x_itr!=x_last; ++x_itr)
    if (*x_itr)  {y=true; return;}
  y= false;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCaster,TInt ,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCaster,TInt ,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCaster,TReal,TInt )
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCaster,TReal,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCaster,TBool,TInt )
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCaster,TBool,TReal)
SKYAI_ADD_MODULE(MCaster_TInt_TReal)
SKYAI_ADD_MODULE(MCaster_TInt_TBool)
SKYAI_ADD_MODULE(MCaster_TReal_TInt )
SKYAI_ADD_MODULE(MCaster_TReal_TBool)
SKYAI_ADD_MODULE(MCaster_TBool_TInt )
SKYAI_ADD_MODULE(MCaster_TBool_TReal)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MConstMultiplier,TRealVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MConstMultiplier,TRealMatrix,TRealVector)
SKYAI_ADD_MODULE(MConstMultiplier_TRealVector_TRealVector)
SKYAI_ADD_MODULE(MConstMultiplier_TRealMatrix_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemSquarer,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemSquarer,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemSquarer,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemSquarer,TRealVector)
SKYAI_ADD_MODULE(MElemSquarer_TInt)
SKYAI_ADD_MODULE(MElemSquarer_TReal)
SKYAI_ADD_MODULE(MElemSquarer_TIntVector)
SKYAI_ADD_MODULE(MElemSquarer_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MBoolAnd)
SKYAI_ADD_MODULE(MBoolOr)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMinElementValue,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMaxElementValue,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMinElementIndex,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMaxElementIndex,TRealVector)
SKYAI_ADD_MODULE(MMinElementValue_TIntVector)
SKYAI_ADD_MODULE(MMaxElementValue_TRealVector)
SKYAI_ADD_MODULE(MMinElementIndex_TIntVector)
SKYAI_ADD_MODULE(MMaxElementIndex_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MNonzeroElements,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MNonzeroElements,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MNonzeroElementsCounter,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MNonzeroElementsCounter,TRealVector)
SKYAI_ADD_MODULE(MNonzeroElements_TIntVector)
SKYAI_ADD_MODULE(MNonzeroElements_TRealVector)
SKYAI_ADD_MODULE(MNonzeroElementsCounter_TIntVector)
SKYAI_ADD_MODULE(MNonzeroElementsCounter_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorToScalar,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorToScalar,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorToScalar,TBoolVector)
SKYAI_ADD_MODULE(MVectorToScalar_TIntVector)
SKYAI_ADD_MODULE(MVectorToScalar_TRealVector)
SKYAI_ADD_MODULE(MVectorToScalar_TBoolVector)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

