//-------------------------------------------------------------------------------------------
/*! \file    learning_manager.cpp
    \brief   libskyai - learning management (e.g. number of episode) module  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.09, 2009-

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
// #include <iostream>
//-------------------------------------------------------------------------------------------
#include <skyai/modules_core/learning_manager.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MBasicLearningManager)
SKYAI_ADD_MODULE(MManualLearningManager)
//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MFunctionScheduler)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

