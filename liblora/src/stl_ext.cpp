//-------------------------------------------------------------------------------------------
/*! \file    stl_ext.cpp
    \brief   liblora - STL extension  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

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
#include <lora/stl_ext.h>
#include <cstdarg>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

/*!\brief generate vector, list (container generator)
  \note container is instantiated with std::list<T>, std::vector<T>, T={int,double,long double}  */
template <typename container>
container  ContainerGen (int size, ...)
{
  va_list argptr;
  va_start (argptr,size);

  container res(size);
  for (typename container::iterator  itr (res.begin()); itr!=res.end(); ++itr)
    *itr= va_arg (argptr, typename container::value_type);

  va_end (argptr);
  return res;
}
#define INSTANTIATOR(_cnt,_type) \
  template std::_cnt< _type > ContainerGen (int size, ...);
#define CNT list
INSTANTIATOR(CNT,int)
INSTANTIATOR(CNT,double)
INSTANTIATOR(CNT,long double)
// INSTANTIATOR(CNT,std::string)
#undef CNT
#define CNT vector
INSTANTIATOR(CNT,int)
INSTANTIATOR(CNT,double)
INSTANTIATOR(CNT,long double)
// INSTANTIATOR(CNT,std::string)
#undef CNT
#undef INSTANTIATOR
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
