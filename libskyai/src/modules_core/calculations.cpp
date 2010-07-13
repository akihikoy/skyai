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
  //!\todo Reduce computational cost (memory re-allocation):
  y= conf_.Factor * x;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// MBoolVector{And,Or}
//===========================================================================================

void MBoolVectorAnd::function (const TInput &x, TOutput &y) const
{
  for (TypeExt<TInput>::const_iterator x_itr(GenBegin(x)),x_last(GenEnd(x)); x_itr!=x_last; ++x_itr)
    if (!(*x_itr))  {y=false; return;}
  y= true;
}
//-------------------------------------------------------------------------------------------

void MBoolVectorOr::function (const TInput &x, TOutput &y) const
{
  for (TypeExt<TInput>::const_iterator x_itr(GenBegin(x)),x_last(GenEnd(x)); x_itr!=x_last; ++x_itr)
    if (*x_itr)  {y=true; return;}
  y= false;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TRealMatrix)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TBool)
// SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TBoolVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MConstant,TString)
SKYAI_ADD_MODULE(MConstant_TInt)
SKYAI_ADD_MODULE(MConstant_TReal)
SKYAI_ADD_MODULE(MConstant_TIntVector)
SKYAI_ADD_MODULE(MConstant_TRealVector)
SKYAI_ADD_MODULE(MConstant_TRealMatrix)
SKYAI_ADD_MODULE(MConstant_TBool)
// SKYAI_ADD_MODULE(MConstant_TBoolVector)
SKYAI_ADD_MODULE(MConstant_TString)
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
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MVectorToScalar,TIntVector,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MVectorToScalar,TRealVector,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MVectorToScalar,TBoolVector,TBool)
SKYAI_ADD_MODULE(MVectorToScalar_TIntVector_TInt)
SKYAI_ADD_MODULE(MVectorToScalar_TRealVector_TReal)
SKYAI_ADD_MODULE(MVectorToScalar_TBoolVector_TBool)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorShuffler,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorShuffler,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorShuffler,TBoolVector)
SKYAI_ADD_MODULE(MVectorShuffler_TIntVector)
SKYAI_ADD_MODULE(MVectorShuffler_TRealVector)
SKYAI_ADD_MODULE(MVectorShuffler_TBoolVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MAdd,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MAdd,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MAdd,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MAdd,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSubtract,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSubtract,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSubtract,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSubtract,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TInt,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TInt,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TReal,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TReal,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TInt,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TIntVector,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TReal,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TRealVector,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMultiply,TRealMatrix,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MDivide,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MDivide,TReal)
SKYAI_ADD_MODULE(MAdd_TInt)
SKYAI_ADD_MODULE(MAdd_TReal)
SKYAI_ADD_MODULE(MAdd_TIntVector)
SKYAI_ADD_MODULE(MAdd_TRealVector)
SKYAI_ADD_MODULE(MSubtract_TInt)
SKYAI_ADD_MODULE(MSubtract_TReal)
SKYAI_ADD_MODULE(MSubtract_TIntVector)
SKYAI_ADD_MODULE(MSubtract_TRealVector)
SKYAI_ADD_MODULE(MMultiply_TInt_TInt)
SKYAI_ADD_MODULE(MMultiply_TInt_TReal)
SKYAI_ADD_MODULE(MMultiply_TReal_TInt)
SKYAI_ADD_MODULE(MMultiply_TReal_TReal)
SKYAI_ADD_MODULE(MMultiply_TInt_TIntVector)
SKYAI_ADD_MODULE(MMultiply_TIntVector_TInt)
SKYAI_ADD_MODULE(MMultiply_TReal_TRealVector)
SKYAI_ADD_MODULE(MMultiply_TRealVector_TReal)
SKYAI_ADD_MODULE(MMultiply_TRealMatrix_TRealVector)
SKYAI_ADD_MODULE(MDivide_TInt)
SKYAI_ADD_MODULE(MDivide_TReal)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemMultiply,TIntVector,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemMultiply,TIntVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemMultiply,TRealVector,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemMultiply,TRealVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemDivide,TIntVector,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemDivide,TIntVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemDivide,TRealVector,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MElemDivide,TRealVector,TRealVector)
SKYAI_ADD_MODULE(MElemMultiply_TIntVector_TIntVector)
SKYAI_ADD_MODULE(MElemMultiply_TIntVector_TRealVector)
SKYAI_ADD_MODULE(MElemMultiply_TRealVector_TIntVector)
SKYAI_ADD_MODULE(MElemMultiply_TRealVector_TRealVector)
SKYAI_ADD_MODULE(MElemDivide_TIntVector_TIntVector)
SKYAI_ADD_MODULE(MElemDivide_TIntVector_TRealVector)
SKYAI_ADD_MODULE(MElemDivide_TRealVector_TIntVector)
SKYAI_ADD_MODULE(MElemDivide_TRealVector_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MConstMultiplier,TInt,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MConstMultiplier,TReal,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MConstMultiplier,TRealVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MConstMultiplier,TRealMatrix,TRealVector)
SKYAI_ADD_MODULE(MConstMultiplier_TInt_TInt)
SKYAI_ADD_MODULE(MConstMultiplier_TReal_TReal)
SKYAI_ADD_MODULE(MConstMultiplier_TRealVector_TRealVector)
SKYAI_ADD_MODULE(MConstMultiplier_TRealMatrix_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSquare,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSquare,TReal)
SKYAI_ADD_MODULE(MSquare_TInt)
SKYAI_ADD_MODULE(MSquare_TReal)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemSquare,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemSquare,TRealVector)
SKYAI_ADD_MODULE(MElemSquare_TIntVector)
SKYAI_ADD_MODULE(MElemSquare_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEqualTo,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEqualTo,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MGreaterThan,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MGreaterThan,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MGreaterEqual,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MGreaterEqual,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLessThan,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLessThan,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLessEqual,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MLessEqual,TReal)
SKYAI_ADD_MODULE(MEqualTo_TInt)
SKYAI_ADD_MODULE(MEqualTo_TReal)
SKYAI_ADD_MODULE(MGreaterThan_TInt)
SKYAI_ADD_MODULE(MGreaterThan_TReal)
SKYAI_ADD_MODULE(MGreaterEqual_TInt)
SKYAI_ADD_MODULE(MGreaterEqual_TReal)
SKYAI_ADD_MODULE(MLessThan_TInt)
SKYAI_ADD_MODULE(MLessThan_TReal)
SKYAI_ADD_MODULE(MLessEqual_TInt)
SKYAI_ADD_MODULE(MLessEqual_TReal)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemEqualTo,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemEqualTo,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemGreaterThan,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemGreaterThan,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemGreaterEqual,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemGreaterEqual,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemLessThan,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemLessThan,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemLessEqual,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MElemLessEqual,TRealVector)
SKYAI_ADD_MODULE(MElemEqualTo_TIntVector)
SKYAI_ADD_MODULE(MElemEqualTo_TRealVector)
SKYAI_ADD_MODULE(MElemGreaterThan_TIntVector)
SKYAI_ADD_MODULE(MElemGreaterThan_TRealVector)
SKYAI_ADD_MODULE(MElemGreaterEqual_TIntVector)
SKYAI_ADD_MODULE(MElemGreaterEqual_TRealVector)
SKYAI_ADD_MODULE(MElemLessThan_TIntVector)
SKYAI_ADD_MODULE(MElemLessThan_TRealVector)
SKYAI_ADD_MODULE(MElemLessEqual_TIntVector)
SKYAI_ADD_MODULE(MElemLessEqual_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MBoolVectorAnd)
SKYAI_ADD_MODULE(MBoolVectorOr)
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

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

