//-------------------------------------------------------------------------------------------
/*! \file    variable_space_oct.h
    \brief   liblora - liboctave extension of variable-space  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.21, 2010-

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
#ifndef loco_rabbits_variable_space_oct_h
#define loco_rabbits_variable_space_oct_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space.h>
//-------------------------------------------------------------------------------------------
class ColumnVector;
class RowVector;
class Matrix;
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------


class TRowOfMatrix;

//===========================================================================================
// specialization of TVariable::generator for octave types
//===========================================================================================


#define SPECIALIZER(x_type)  \
  template<> struct TVariable::generator<x_type>                                 \
  {                                                                              \
    TVariable &o;                                                                \
    generator(TVariable &outer) : o(outer) {}                                    \
    void operator() (x_type &x);                                                 \
  };
SPECIALIZER(ColumnVector)
SPECIALIZER(RowVector)
SPECIALIZER(TRowOfMatrix)
SPECIALIZER(Matrix)
#undef SPECIALIZER


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_space_oct_h
//-------------------------------------------------------------------------------------------
