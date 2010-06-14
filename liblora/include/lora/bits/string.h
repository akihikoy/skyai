//-------------------------------------------------------------------------------------------
/*! \file    string.h
    \brief   liblora - string utility  (supplementary functions)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

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
#ifndef lora_string_h
#define lora_string_h
//-------------------------------------------------------------------------------------------
#include <string>
#include <boost/lexical_cast.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace string_detail
{
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//  for stringTo*
//-------------------------------------------------------------------------------------------

inline void ReadNumbers (std::string &s, std::string::const_iterator &itr, const std::string::const_iterator &end)
{
  while (itr!=end)
  {
    if (*itr<'0'||*itr>'9')  break;
    s+=*itr; ++itr;
  }
}
//-------------------------------------------------------------------------------------------

inline long double ReadFloatFromStr (std::string::const_iterator &itr, const std::string::const_iterator &end)
{
  std::string s;
  if ((itr!=end)&&(*itr=='+'||*itr=='-'))
    {s+=*itr; ++itr;}
  ReadNumbers (s, itr, end);
  if ((itr!=end)&&(*itr=='.'))
  {
    s+=*itr; ++itr;
    ReadNumbers (s, itr, end);
  }
  if ((itr!=end)&&(*itr=='e'||*itr=='E'))
  {
    s+=*itr; ++itr;
    if ((itr!=end)&&(*itr=='+'||*itr=='-'))
      {s+=*itr; ++itr;}
    ReadNumbers (s, itr, end);
  }
  return boost::lexical_cast<long double>(s);
}
//-------------------------------------------------------------------------------------------

inline void SkipNotNumberCharacters (std::string::const_iterator &itr, const std::string::const_iterator &end, std::string *skipped=NULL)
{
  if (skipped==NULL)
    while (itr!=end&&(*itr!='+'&&*itr!='-'&&*itr!='.'&&(*itr<'0'||*itr>'9'))) ++itr;
  else
  {
    *skipped="";
    while (itr!=end&&(*itr!='+'&&*itr!='-'&&*itr!='.'&&(*itr<'0'||*itr>'9')))
      {*skipped+=*itr;  ++itr;}
  }
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace string_detail
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // lora_string_h
//-------------------------------------------------------------------------------------------
