//-------------------------------------------------------------------------------------------
/*! \file    marker-tracker.cpp
    \brief   skyai - marker tracker test code
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.28, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#include <lora/marker_tracker.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  int no_obs(0);
  marker_tracker::TMarkerTracker mtracker;
  mtracker.Initialize();
  while(mtracker.Step())
  {
    if(mtracker.EstimatedObservation().c[0]<0 || mtracker.EstimatedObservation().c[0]>mtracker.ImageWidth()
      || mtracker.EstimatedObservation().c[1]<0 || mtracker.EstimatedObservation().c[1]>mtracker.ImageHeight())
    {
      LMESSAGE("marker may be out of image!");
    }
    if(!mtracker.Observed())
    {
      ++no_obs;
      LMESSAGE("no observation count: "<<no_obs);
    }
    else
      no_obs= 0;
    int key = cv::waitKey (10);
    if (key == 'q' || key == 'Q')
      break;
  }
  return 0;
}
//-------------------------------------------------------------------------------------------
