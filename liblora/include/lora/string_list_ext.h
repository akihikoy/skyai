//-------------------------------------------------------------------------------------------
/*! \file    string_list_ext.h
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
#ifndef loco_rabbits_string_list_ext_h
#define loco_rabbits_string_list_ext_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
// #include <vector>
// #include <list>
#include <map>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

// forward declarations:
class TStringListEx;
template <typename T>
void SaveToStringList (const T &m, TStringListEx &str_list, const std::string &prefix=std::string(""));
template <typename T>
void LoadFromStringList (T &m, const TStringListEx &str_list);
//-------------------------------------------------------------------------------------------

// declarations:
template <typename t_key, typename t_value>
void SaveToStringListMap (const std::map<t_key,t_value> &m, TStringListEx &str_list, const std::string &prefix=std::string(""));
template <typename t_key, typename t_value>
void LoadFromStringListMap (std::map<t_key,t_value> &m, const TStringListEx &str_list);
//-------------------------------------------------------------------------------------------

// specializations:
#define SPECIALIZER(x_key,x_value)              \
  template <>                                   \
  inline void SaveToStringList (const std::map<x_key,x_value> &m, TStringListEx &str_list, const std::string &prefix) \
    {SaveToStringListMap(m,str_list,prefix);}   \
  template <>                                   \
  inline void LoadFromStringList (std::map<x_key,x_value> &m, const TStringListEx &str_list)  \
    {LoadFromStringListMap(m,str_list);}
SPECIALIZER(int           , signed int     )
SPECIALIZER(int           , signed short   )
SPECIALIZER(int           , signed long    )
SPECIALIZER(int           , unsigned int   )
SPECIALIZER(int           , unsigned short )
SPECIALIZER(int           , unsigned long  )
SPECIALIZER(int           , float          )
SPECIALIZER(int           , double         )
SPECIALIZER(int           , long double    )
SPECIALIZER(int           , bool           )
SPECIALIZER(int           , std::string    )

SPECIALIZER(std::string   , signed int     )
SPECIALIZER(std::string   , signed short   )
SPECIALIZER(std::string   , signed long    )
SPECIALIZER(std::string   , unsigned int   )
SPECIALIZER(std::string   , unsigned short )
SPECIALIZER(std::string   , unsigned long  )
SPECIALIZER(std::string   , float          )
SPECIALIZER(std::string   , double         )
SPECIALIZER(std::string   , long double    )
SPECIALIZER(std::string   , bool           )
SPECIALIZER(std::string   , std::string    )
#undef SPECIALIZER
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_string_list_ext_h
//-------------------------------------------------------------------------------------------
