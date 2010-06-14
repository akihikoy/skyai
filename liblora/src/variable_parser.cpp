//-------------------------------------------------------------------------------------------
/*! \file    variable_parser.cpp
    \brief   liblora - parser for variable-space  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-

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
#include <lora/variable_parser_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
using namespace std;
using namespace boost::spirit::classic;

// explicit instantiation
template class TParserAgent <file_iterator<char> >;
template class TCodeParser <file_iterator<char> >;


bool ParseFile (TVariable &var, const std::string &filename, bool *is_last)
{
  typedef file_iterator<char> TIterator;
  TIterator  first(filename);
  if (!first)
  {
    LERROR("failed to open file: "<<filename);
    return false;
  }

  TParserAgent<TIterator> pagent;

  TIterator last= first.make_end();

  parse_info<TIterator> info= pagent.Parse(var, first, last);
  if (is_last)  *is_last= (info.stop==last);
  return !pagent.Error();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

