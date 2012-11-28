//-------------------------------------------------------------------------------------------
/*! \file    libbioloid.cpp
    \brief   benchmarks - motion learning task of a real robot (Bioloid, ROBOTIS)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.04, 2010

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
#include "libbioloid.h"
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

namespace var_space{
  void Register (marker_tracker::TMarkerTrackerConfig &x, TVariableMap &mmap)
  {
    #define ADD(x_member)  AddToVarMap(mmap, #x_member, x.x_member)
    ADD( CameraDeviceID                );

    ADD( MarkerFileName                );
    ADD( MarkerDetectionThreshold      );

    ADD( NumOfParticles                );

    ADD( Dt                            );
    ADD( InitCX                        );
    ADD( InitCY                        );
    ADD( InitCZ1                       );
    ADD( InitCZ2                       );
    ADD( InitV                         );
    ADD( InitW                         );
    ADD( InitL11                       );
    ADD( InitL12                       );
    ADD( InitL21                       );
    ADD( InitL22                       );
    ADD( InitF1                        );
    ADD( InitF2                        );

    ADD( NoiseC                        );
    ADD( NoiseR                        );
    ADD( NoiseV                        );
    ADD( NoiseW                        );

    ADD( NoiseL1                       );
    ADD( NoiseL2                       );
    ADD( NoiseF                        );

    ADD( L1EqL2                        );

    ADD( ScaleX                        );
    ADD( ScaleY                        );
    ADD( Width                         );
    ADD( Height                        );
    ADD( WeightSigma                   );
    ADD( Epsilon                       );

    ADD( PrintResult                   );
    ADD( NumOfDisplayLines             );
    ADD( DisplayResult                 );
    ADD( WaitKeyDelay                  );
    ADD( WindowName                    );
    #undef ADD
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MBioloidEnvironment)
SKYAI_ADD_MODULE(MMotionLearningTask)
SKYAI_ADD_MODULE(MMarkerTracker)
SKYAI_ADD_MODULE(MBioloidUnivTask)
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

