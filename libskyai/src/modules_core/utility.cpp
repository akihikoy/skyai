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
SKYAI_ADD_MODULE(MPrinter)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TBoolVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TRealMatrix)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MMediator0,TRealVectorSet)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TBoolVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MMediator1,TVoid,TRealMatrix)
SKYAI_ADD_MODULE(MMediator0_TInt)
SKYAI_ADD_MODULE(MMediator0_TReal)
SKYAI_ADD_MODULE(MMediator0_TBool)
SKYAI_ADD_MODULE(MMediator0_TIntVector)
SKYAI_ADD_MODULE(MMediator0_TRealVector)
SKYAI_ADD_MODULE(MMediator0_TBoolVector)
SKYAI_ADD_MODULE(MMediator0_TRealMatrix)
SKYAI_ADD_MODULE(MMediator0_TRealVectorSet)
SKYAI_ADD_MODULE(MMediator1_TVoid_TInt)
SKYAI_ADD_MODULE(MMediator1_TVoid_TReal)
SKYAI_ADD_MODULE(MMediator1_TVoid_TBool)
SKYAI_ADD_MODULE(MMediator1_TVoid_TIntVector)
SKYAI_ADD_MODULE(MMediator1_TVoid_TRealVector)
SKYAI_ADD_MODULE(MMediator1_TVoid_TBoolVector)
SKYAI_ADD_MODULE(MMediator1_TVoid_TRealMatrix)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorMixer,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorMixer,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MVectorMixer,TBoolVector)
SKYAI_ADD_MODULE(MVectorMixer_TIntVector)
SKYAI_ADD_MODULE(MVectorMixer_TRealVector)
SKYAI_ADD_MODULE(MVectorMixer_TBoolVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MWeightedMixerRv)
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
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleAccumulator,TRealVector)
SKYAI_ADD_MODULE(MSimpleAccumulator_TInt)
SKYAI_ADD_MODULE(MSimpleAccumulator_TReal)
SKYAI_ADD_MODULE(MSimpleAccumulator_TIntVector)
SKYAI_ADD_MODULE(MSimpleAccumulator_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MStatisticsFilter)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TInt,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TReal,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TIntVector,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TRealVector,TRealVector)
SKYAI_ADD_MODULE(MFunctionSISOSharer_TInt_TInt)
SKYAI_ADD_MODULE(MFunctionSISOSharer_TReal_TReal)
SKYAI_ADD_MODULE(MFunctionSISOSharer_TIntVector_TIntVector)
SKYAI_ADD_MODULE(MFunctionSISOSharer_TRealVector_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHolder,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHolder,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MHolder,TComposite1)
SKYAI_ADD_MODULE(MHolder_TInt)
SKYAI_ADD_MODULE(MHolder_TRealVector)
SKYAI_ADD_MODULE(MHolder_TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCompositeHolder2,TInt,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCompositeHolder2,TRealVector,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MCompositeHolder2,TInt,TInt)
SKYAI_ADD_MODULE(MCompositeHolder2_TInt_TRealVector)
SKYAI_ADD_MODULE(MCompositeHolder2_TRealVector_TRealVector)
SKYAI_ADD_MODULE(MCompositeHolder2_TInt_TInt)
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

