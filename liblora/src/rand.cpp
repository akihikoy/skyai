//-------------------------------------------------------------------------------------------
/*! \file    rand.cpp
    \brief   liblora - random number utility  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

    Copyright (C) 2008, 2010  Akihiko Yamaguchi

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
#include <lora/rand.h>
#include <boost/random.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

namespace loco_rabbits_detail_ndrand
{
  boost::variate_generator<
          boost::mt19937,
          boost::normal_distribution<> >
              ndrand (boost::mt19937(0),
                      boost::normal_distribution<>(0.0, 1.0));
}

//!\brief set the seed of the NDRand
void NDSrand (unsigned long seed)
{
  loco_rabbits_detail_ndrand::ndrand.engine().seed (static_cast<unsigned long>(seed));
}
//-------------------------------------------------------------------------------------------

//!\brief generate normal distribution
double NDRand (void)
{
  return loco_rabbits_detail_ndrand::ndrand();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
