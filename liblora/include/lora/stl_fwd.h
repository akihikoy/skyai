//-------------------------------------------------------------------------------------------
/*! \file    stl_fwd.h
    \brief   liblora - forward declarations of STL containers  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Sep.02, 2010

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
#ifndef loco_rabbits_stl_fwd_h
#define loco_rabbits_stl_fwd_h
//-------------------------------------------------------------------------------------------
namespace std
{
template < class T > struct less;
template < class T1, class T2 > struct pair;
template < class T > class allocator;
template < class T, class Allocator > class list;
template < class T, class Allocator > class vector;
template < class Key, class T, class Compare, class Allocator > class map;
}
#define LIST_FWD(x_type)        std::list<x_type, std::allocator<x_type> >
#define VECTOR_FWD(x_type)      std::vector<x_type, std::allocator<x_type> >
#define MAP_FWD(x_key,x_value)  std::map<x_key, x_value, std::less<x_key>, std::allocator<std::pair<const x_key,x_value> > >
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_stl_fwd_h
//-------------------------------------------------------------------------------------------
