//-------------------------------------------------------------------------------------------
/*! \file    string_list_ext.cpp
    \brief   liblora - TStringListEx extensions  (DEPRECATED)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.29, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

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
#include <lora/string_list_ext.h>
#include <lora/string_list_ext_impl.h>
//-------------------------------------------------------------------------------------------
#include <map>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

// instantiations:
#define INSTANTIATOR(x_key,x_value)  \
  template void SaveToStringListMap (const std::map<x_key,x_value> &m, TStringListEx &str_list, const std::string &prefix);  \
  template void LoadFromStringListMap (std::map<x_key,x_value> &m, const TStringListEx &str_list);
INSTANTIATOR(int           , signed int     )
INSTANTIATOR(int           , signed short   )
INSTANTIATOR(int           , signed long    )
INSTANTIATOR(int           , unsigned int   )
INSTANTIATOR(int           , unsigned short )
INSTANTIATOR(int           , unsigned long  )
INSTANTIATOR(int           , float          )
INSTANTIATOR(int           , double         )
INSTANTIATOR(int           , long double    )
INSTANTIATOR(int           , bool           )
INSTANTIATOR(int           , std::string    )

INSTANTIATOR(std::string   , signed int     )
INSTANTIATOR(std::string   , signed short   )
INSTANTIATOR(std::string   , signed long    )
INSTANTIATOR(std::string   , unsigned int   )
INSTANTIATOR(std::string   , unsigned short )
INSTANTIATOR(std::string   , unsigned long  )
INSTANTIATOR(std::string   , float          )
INSTANTIATOR(std::string   , double         )
INSTANTIATOR(std::string   , long double    )
INSTANTIATOR(std::string   , bool           )
INSTANTIATOR(std::string   , std::string    )
#undef INSTANTIATOR
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

