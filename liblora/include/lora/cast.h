//-------------------------------------------------------------------------------------------
/*! \file    cast.h
    \brief   liblora - lora_cast definition
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Jun.25, 2010

    Copyright (C) 2008, 2009, 2010  Akihiko Yamaguchi

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

    -----------------------------------------------------------------------------------------

    \note include lora/string.h to use conversions from/to std::string
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_cast_h
#define loco_rabbits_cast_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

template <typename t_to, typename t_from>
struct TLocoRabbitsCastPolicy
{
  static inline t_to F (const t_from &from)
  {
    return static_cast<t_to>(from);
  }
};
//-------------------------------------------------------------------------------------------

template <typename t_from>
struct TLocoRabbitsCastPolicy <t_from, t_from>
{
  static inline t_from F (const t_from &from)
  {
    return from;  // return as-is
  }
};
//-------------------------------------------------------------------------------------------

template<> struct TLocoRabbitsCastPolicy <float       , bool>
  {static inline float       F (const bool &from)  {return from? 1.0f : 0.0f;}};
template<> struct TLocoRabbitsCastPolicy <double      , bool>
  {static inline double      F (const bool &from)  {return from? 1.0  : 0.0 ;}};
template<> struct TLocoRabbitsCastPolicy <long double , bool>
  {static inline long double F (const bool &from)  {return from? 1.0l : 0.0l;}};
//-------------------------------------------------------------------------------------------

template <typename t_from> inline const std::string ConvertToStr (const t_from &val);
template <typename t_to>   inline const t_to ConvertFromStr (const std::string &str);

template <typename t_from>
struct TLocoRabbitsCastPolicy <std::string, t_from>
{
  static inline std::string F (const t_from &from)
  {
    return ConvertToStr<t_from>(from);
  }
};
template <typename t_to>
struct TLocoRabbitsCastPolicy <t_to, std::string>
{
  static inline t_to F (const std::string &from)
  {
    return ConvertFromStr<t_to>(from);
  }
};
//-------------------------------------------------------------------------------------------

template <>
struct TLocoRabbitsCastPolicy <std::string, std::string>
{
  static inline std::string F (const std::string &from)
  {
    return from;
  }
};
//-------------------------------------------------------------------------------------------

template <typename t_to, typename t_from>
inline t_to lora_cast (const t_from &from)
{
  return TLocoRabbitsCastPolicy<t_to, t_from>::F(from);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_cast_h
//-------------------------------------------------------------------------------------------
