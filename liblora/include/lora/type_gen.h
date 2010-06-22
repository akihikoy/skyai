//-------------------------------------------------------------------------------------------
/*! \file    type_gen.h
    \brief   liblora - some templates to handle different types with the same interfaces
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.11, 2010-

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
#ifndef loco_rabbits_type_gen_h
#define loco_rabbits_type_gen_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


/*!\brief type extension class */
template <typename t_type>
struct TypeExt
{
  typedef typename t_type::value_type         value_type;
  typedef typename t_type::iterator           iterator;
  typedef typename t_type::const_iterator     const_iterator;
  typedef typename t_type::reference          reference;
  typedef typename t_type::const_reference    const_reference;
};

/*!\brief type extension class partially specialized for const t_type */
template <typename t_type>
struct TypeExt <const t_type>
{
  typedef typename TypeExt<t_type>::value_type         value_type;
  typedef typename TypeExt<t_type>::iterator           iterator;
  typedef typename TypeExt<t_type>::const_iterator     const_iterator;
  typedef typename TypeExt<t_type>::reference          reference;
  typedef typename TypeExt<t_type>::const_reference    const_reference;
};

//-------------------------------------------------------------------------------------------

/* generic functions */

template <typename t_type> inline typename TypeExt<t_type>::iterator GenBegin (t_type &x)              {return x.begin();}
template <typename t_type> inline typename TypeExt<t_type>::const_iterator GenBegin (const t_type &x)  {return x.begin();}
template <typename t_type> inline typename TypeExt<t_type>::iterator GenEnd (t_type &x)                {return x.end();}
template <typename t_type> inline typename TypeExt<t_type>::const_iterator GenEnd (const t_type &x)    {return x.end();}
template <typename t_type> inline typename TypeExt<t_type>::reference GenAt (t_type &x,int i)              {return x[i];}
template <typename t_type> inline typename TypeExt<t_type>::const_reference GenAt (const t_type &x,int i)  {return x[i];}
template <typename t_type> inline int   GenSize (const t_type &x)  {return x.size();}
template <typename t_type> inline void  GenResize (t_type &x, int s)  {return x.resize(s);}
template <typename t_type> inline void  GenResize (t_type &x, int s, const typename TypeExt<t_type>::value_type &vfill)  {return x.resize(s,vfill);}

//-------------------------------------------------------------------------------------------

/*! generic stream operator.

  usage:
  \code
    #include <lora/stl_ext.h>  // for PrintContainer
    ...
    ColumnVector x;
    ...
    std::cout<<"x= "<<GenPrint(x)<<std::endl
  \endcode
*/

template <typename t_type>
struct TGenPrint
{
  const t_type &Entity;
  const std::string Delim;
  TGenPrint(const t_type &v_entity, const std::string &v_delim)
    : Entity(v_entity), Delim(v_delim) {}
};
// TGenPrint generator:
template <typename t_type>
TGenPrint<t_type>  GenPrint(const t_type &v_entity, const std::string &v_delim=" ")
  {return TGenPrint<t_type>(v_entity,v_delim);}

// forward declaration:
template <typename t_fwd_iterator>
inline void PrintContainer (t_fwd_iterator first, t_fwd_iterator last, std::ostream &os, const std::string &delim);

template <typename t_type>
std::ostream& operator<< (std::ostream &lhs, const TGenPrint<t_type> &rhs)
{
  PrintContainer(GenBegin(rhs.Entity), GenEnd(rhs.Entity), lhs, rhs.Delim);
  return lhs;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_type_gen_h
//-------------------------------------------------------------------------------------------
