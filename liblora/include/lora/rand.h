//-------------------------------------------------------------------------------------------
/*! \file    rand.h
    \brief   liblora - random number utility  (header)
    \author  Akihiko Yamaguchi
    \date    2008-

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
#ifndef loco_rabbits_rand_h
#define loco_rabbits_rand_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <cstdlib>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

inline unsigned Srand(void)
{
  unsigned seed ((unsigned)time(NULL));
  srand(seed);
  return seed;
}
//-------------------------------------------------------------------------------------------

inline TReal Rand (const double &max)
{
  return (max)*static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
}
//-------------------------------------------------------------------------------------------

inline TReal Rand (const double &min, const double &max)
{
  return Rand(max - min) + min;
}
//-------------------------------------------------------------------------------------------

inline TReal Rand (const long double &max)
{
  return (max)*static_cast<long double>(rand()) / static_cast<long double>(RAND_MAX);
}
//-------------------------------------------------------------------------------------------

inline TReal Rand (const long double &min, const long double &max)
{
  return Rand(max - min) + min;
}
//-------------------------------------------------------------------------------------------

inline int Rand (int min, int max)
  // return [min,max]
{
  // return static_cast<int>(Rand(static_cast<double>(min),static_cast<double>(max+1)));
  return static_cast<int>(real_floor(Rand(static_cast<double>(min),static_cast<double>(max+1))));
}
//-------------------------------------------------------------------------------------------

inline bool RollADice (const TReal &probability)
{
  return (Rand(GetOne<TReal>()) < probability);
}
//-------------------------------------------------------------------------------------------

//!\brief set the seed of the NDRand
void NDSrand (unsigned long seed);
//!\brief generate normal distribution
double NDRand (void);
//-------------------------------------------------------------------------------------------

inline void XSrand (unsigned long seed)
{
  srand (seed);
  NDSrand (seed);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_rand_h
//-------------------------------------------------------------------------------------------
