//-------------------------------------------------------------------------------------------
/*! \file    octave_str.h
    \brief   liblora - liboctave extension about string processing  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    2008-
    \date    Oct.25, 2009 : {CV,RV}ToS, CTS, CFS

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
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_octave_str_h
#define loco_rabbits_octave_str_h
 //-------------------------------------------------------------------------------------------
#include <lora/string.h>
#include <lora/octave.h>
#include <string>
//-------------------------------------------------------------------------------------------
// class ColumnVector;
// class RowVector;
// class Matrix;
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


// forward decl.
template <typename t_from> inline const std::string ConvertToStr (const t_from &val);
template <typename t_to>   inline const t_to ConvertFromStr (const std::string &str);

class TStringListEx;
template <typename T>
void SaveToStringList (const T &m, TStringListEx &str_list, const std::string &prefix=std::string(""));
template <typename T>
void LoadFromStringList (T &m, const TStringListEx &str_list);
//-------------------------------------------------------------------------------------------

ColumnVector StringToColumnVector (const std::string &line);
RowVector StringToRowVector (const std::string &line);

std::string ColumnVectorToString (const ColumnVector &val);
std::string RowVectorToString (const RowVector &val);
//-------------------------------------------------------------------------------------------

#define SPECIALIZER(x_type,x_t2s,x_s2t) \
  template <> inline const std::string ConvertToStr (const x_type &val)   {return x_t2s(val);} \
  template <> inline const x_type ConvertFromStr (const std::string &str) {return x_s2t(str);}
SPECIALIZER(ColumnVector , ColumnVectorToString , StringToColumnVector )
SPECIALIZER(RowVector    , RowVectorToString    , StringToRowVector    )
#undef SPECIALIZER
//-------------------------------------------------------------------------------------------

template <>
void SaveToStringList (const Matrix &m, TStringListEx &str_list, const std::string &prefix);
template <>
void LoadFromStringList (Matrix &m, const TStringListEx &str_list);
template <>
void SaveToStringList (const ColumnVector &x, TStringListEx &str_list, const std::string &prefix);
template <>
void LoadFromStringList (ColumnVector &x, const TStringListEx &str_list);


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_octave_str_h
//-------------------------------------------------------------------------------------------
