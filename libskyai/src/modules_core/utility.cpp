//-------------------------------------------------------------------------------------------
/*! \file    utility.cpp
    \brief   libskyai - misc. utility modules (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.18, 2009-

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
#include <skyai/modules_core/utility.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//-------------------------------------------------------------------------------------------
// SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMinMaxElement,TIntVector)
// SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMinMaxElement,TRealVector)
// SKYAI_ADD_MODULE(MMinMaxElement_TIntVector)
SKYAI_ADD_MODULE(MMinMaxElementRv)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MNonzeroElements,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MNonzeroElements,TRealVector)
SKYAI_ADD_MODULE(MNonzeroElements_TIntVector)
SKYAI_ADD_MODULE(MNonzeroElements_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MWeightedMixer)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TComposite1)
SKYAI_ADD_MODULE(MRemoveSignalArguments_TInt)
SKYAI_ADD_MODULE(MRemoveSignalArguments_TReal)
SKYAI_ADD_MODULE(MRemoveSignalArguments_TIntVector)
SKYAI_ADD_MODULE(MRemoveSignalArguments_TRealVector)
SKYAI_ADD_MODULE(MRemoveSignalArguments_TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MSignalDistributor0)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSignalDistributor1,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSignalDistributor1,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSignalDistributor1,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSignalDistributor1,TRealVector)
SKYAI_ADD_MODULE(MSignalDistributor1_TInt)
SKYAI_ADD_MODULE(MSignalDistributor1_TReal)
SKYAI_ADD_MODULE(MSignalDistributor1_TIntVector)
SKYAI_ADD_MODULE(MSignalDistributor1_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MFunctionSelector0,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MFunctionSelector0,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MFunctionSelector0,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MFunctionSelector0,TRealVector)
SKYAI_ADD_MODULE(MFunctionSelector0_TInt)
SKYAI_ADD_MODULE(MFunctionSelector0_TReal)
SKYAI_ADD_MODULE(MFunctionSelector0_TIntVector)
SKYAI_ADD_MODULE(MFunctionSelector0_TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TRealVector)
SKYAI_ADD_MODULE(MFunctionSelector1_TVoid_TInt)
SKYAI_ADD_MODULE(MFunctionSelector1_TVoid_TReal)
SKYAI_ADD_MODULE(MFunctionSelector1_TVoid_TIntVector)
SKYAI_ADD_MODULE(MFunctionSelector1_TVoid_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MReturnToSameModule10,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MReturnToSameModule10,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MReturnToSameModule10,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MReturnToSameModule10,TRealVector)
SKYAI_ADD_MODULE(MReturnToSameModule10_TInt)
SKYAI_ADD_MODULE(MReturnToSameModule10_TReal)
SKYAI_ADD_MODULE(MReturnToSameModule10_TIntVector)
SKYAI_ADD_MODULE(MReturnToSameModule10_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MRewardAccumulator)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TRealVector)
SKYAI_ADD_MODULE(MSimpleAccumulator_TInt)
SKYAI_ADD_MODULE(MSimpleAccumulator_TReal)
SKYAI_ADD_MODULE(MSimpleAccumulator_TIntVector)
SKYAI_ADD_MODULE(MSimpleAccumulator_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEmitOnce,TReal)
SKYAI_ADD_MODULE(MEmitOnce_TReal)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHolder,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHolder,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHolder,TComposite1)
SKYAI_ADD_MODULE(MHolder_TInt)
SKYAI_ADD_MODULE(MHolder_TRealVector)
SKYAI_ADD_MODULE(MHolder_TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCompositHolder2,TInt,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCompositHolder2,TRealVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCompositHolder2,TInt,TInt)
SKYAI_ADD_MODULE(MCompositHolder2_TInt_TRealVector)
SKYAI_ADD_MODULE(MCompositHolder2_TRealVector_TRealVector)
SKYAI_ADD_MODULE(MCompositHolder2_TInt_TInt)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MConstMultiplierRv)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MDiscretizer)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MIntToIntMapper)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MIntToBoolVectorMapper)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

