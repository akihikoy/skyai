//-------------------------------------------------------------------------------------------
/*! \file    string_impl.h
    \brief   liblora - string utility  (implementat header)
    \author  Akihiko Yamaguchi
    \date    Jan.26,2009-
    \date    Oct. 12, 2009 : Implemented NumericalContainerToString

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

    -----------------------------------------------------------------------------------------

    In this file, StringToNumericalContainer and NumericalContainerToString are implemented.
    These functions are instantiated with every combination of {std::list, std::vector} and {int, double, ...}.
    If you need the other instances, include this header.

    \todo implement NumericalContainerToString and StringToNumericalContainer using TTokenizer

*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_string_impl_h
#define loco_rabbits_string_impl_h
//-------------------------------------------------------------------------------------------
#include <sstream>
#include <lora/string.h>
#include <lora/bits/string.h>
#include <lora/stl_ext.h>
// #include <boost/lexical_cast.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/*!\brief Convert a numerical array (vector,list,etc.) 'vec' to characters
  \note t_container is instantiated with every combination of {std::list, std::vector} and {int, double, ...}.
      If you need the other instances, include lora/string_impl.h
*/
template <typename t_container>
std::string NumericalContainerToString (const t_container &vec, const std::string &delim=" ")
{
  typename t_container::const_iterator itr(vec.begin());
  if (itr==vec.end()) return "";
  std::stringstream ss;
  ss << ConvertToStr(*itr);
  for (++itr; itr!=vec.end(); ++itr)
    ss << delim << ConvertToStr(*itr);
  return ss.str();
}
//-------------------------------------------------------------------------------------------

/*!\brief Convert a string line to a numerical array, and store it into res
  \note t_container is instantiated with every combination of {std::list, std::vector} and {int, double, ...}.
      If you need the other instances, include lora/string_impl.h
*/
template <typename t_container>
void StringToNumericalContainer (const std::string &line, t_container &res)
{
  res.clear();
  if (line=="") return;
  std::list<long double> dlist;
  std::string::const_iterator itr (line.begin());
  string_detail::SkipNotNumberCharacters (itr, line.end());
  while (itr!=line.end())
  {
    dlist.push_back(string_detail::ReadFloatFromStr(itr,line.end()));
    string_detail::SkipNotNumberCharacters (itr, line.end());
  }
  res.resize(dlist.size(),GetZero<typename t_container::value_type>());
  typename t_container::iterator  resitr (res.begin());
  for(std::list<long double>::const_iterator ditr(dlist.begin()); ditr!=dlist.end(); ++ditr,++resitr)
    *resitr = static_cast<typename t_container::value_type>(*ditr);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_string_impl_h
//-------------------------------------------------------------------------------------------

