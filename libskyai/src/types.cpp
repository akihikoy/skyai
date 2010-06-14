//-------------------------------------------------------------------------------------------
/*! \file    types.cpp
    \brief   libskyai - libskyai basic type definitions
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.13, 2010-

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
#include <skyai/types.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

const std::string Composite1ToStr (const TComposite1 &val)
{
  string delim("");
  stringstream ss;
  for (std::vector<TInt>::const_iterator itr(val.DiscSet.begin()),last(val.DiscSet.end()); itr!=last; ++itr)
  {
    ss<<delim<<ConvertToStr(*itr);
    if(delim.empty())  delim=" ";
  }
  if(!delim.empty())  delim="  ";
  for (std::vector<TRealVector>::const_iterator itr(val.ContSet.begin()),last(val.ContSet.end()); itr!=last; ++itr)
  {
    ss<<delim<<ConvertToStr(*itr);
    if(delim.empty())  delim="  ";
  }
  return ss.str();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

