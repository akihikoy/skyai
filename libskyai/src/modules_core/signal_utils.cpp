//-------------------------------------------------------------------------------------------
/*! \file    signal_utils.cpp
    \brief   libskyai - signal/slot utilities
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Jun.28, 2010

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
#include <skyai/modules_core/signal_utils.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MForwarder0)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MForwarder1,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MForwarder1,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MForwarder1,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MForwarder1,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MForwarder1,TComposite1)
SKYAI_ADD_MODULE(MForwarder1_TInt)
SKYAI_ADD_MODULE(MForwarder1_TReal)
SKYAI_ADD_MODULE(MForwarder1_TIntVector)
SKYAI_ADD_MODULE(MForwarder1_TRealVector)
SKYAI_ADD_MODULE(MForwarder1_TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEmitOnce,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEmitOnce,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEmitOnce,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MEmitOnce,TRealVector)
SKYAI_ADD_MODULE(MEmitOnce_TInt)
SKYAI_ADD_MODULE(MEmitOnce_TReal)
SKYAI_ADD_MODULE(MEmitOnce_TIntVector)
SKYAI_ADD_MODULE(MEmitOnce_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MEmitIf)
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
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TBool)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TString)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TInt_TInt)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TInt_TReal)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TInt_TIntVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TInt_TRealVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TInt_TBool)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TReal_TInt)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TReal_TReal)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TReal_TIntVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TReal_TRealVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TReal_TBool)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TBool_TInt)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TBool_TReal)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TBool_TIntVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TBool_TRealVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TBool_TBool)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TVoid_TInt)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TVoid_TReal)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TVoid_TIntVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TVoid_TRealVector)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TVoid_TBool)
SKYAI_ADD_MODULE(MReplaceSignalArguments_TVoid_TString)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

