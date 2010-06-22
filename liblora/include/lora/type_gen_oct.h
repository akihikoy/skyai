//-------------------------------------------------------------------------------------------
/*! \file    type_gen_oct.h
    \brief   liblora - some templates to handle different types with the same interfaces, for liboctave;
              specialization for octave types.
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.23, 2010-

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
#ifndef loco_rabbits_type_gen_oct_h
#define loco_rabbits_type_gen_oct_h
//-------------------------------------------------------------------------------------------
#include <lora/type_gen.h>
#include <lora/octave_fwd.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/*!\brief type extension class specialized for ColumnVector */
template <>
struct TypeExt<ColumnVector>
{
  typedef double            value_type;
  typedef double*           iterator;
  typedef const double*     const_iterator;
  typedef double&           reference;
  typedef const double&     const_reference;
};

/*!\brief type extension class specialized for RowVector */
template <>
struct TypeExt<RowVector>
{
  typedef double            value_type;
  typedef double*           iterator;
  typedef const double*     const_iterator;
  typedef double&           reference;
  typedef const double&     const_reference;
};
//-------------------------------------------------------------------------------------------

/* specialization for liboctave ColumnVector and RowVector */

#define OCT_SPECIALIZER(x_type)  \
  template<> inline TypeExt<x_type>::iterator GenBegin (x_type &x)              {return OctBegin(x);}    \
  template<> inline TypeExt<x_type>::const_iterator GenBegin (const x_type &x)  {return OctBegin(x);}    \
  template<> inline TypeExt<x_type>::iterator GenEnd (x_type &x)                {return OctEnd(x);}      \
  template<> inline TypeExt<x_type>::const_iterator GenEnd (const x_type &x)    {return OctEnd(x);}      \
  template<> inline TypeExt<x_type>::reference GenAt (x_type &x,int i)                {return x(i);}    \
  template<> inline TypeExt<x_type>::const_reference GenAt (const x_type &x,int i)    {return *(OctBegin(x)+i);}  \
  template<> inline int   GenSize (const x_type &x)  {return x.length();}                                    \
  template<> inline void  GenResize (x_type &x, int s)      {return x.resize(s);}                                   \
  template<> inline void  GenResize (x_type &x, int s, const double &vfill)  {return x.resize(s,vfill);}
OCT_SPECIALIZER(ColumnVector)
OCT_SPECIALIZER(RowVector)
#undef OCT_SPECIALIZER

//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_type_gen_oct_h
//-------------------------------------------------------------------------------------------
