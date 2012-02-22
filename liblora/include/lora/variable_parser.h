//-------------------------------------------------------------------------------------------
/*! \file    variable_parser.h
    \brief   liblora - parser for variable-space  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-

    Copyright (C) 2010, 2012  Akihiko Yamaguchi

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
#ifndef loco_rabbits_variable_parser_h
#define loco_rabbits_variable_parser_h
//-------------------------------------------------------------------------------------------
#include <lora/binary.h>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------


struct TParserCallbacks
{
  /*! \brief event callback in parsing a file;
        file_name: current file name, line_num: current line num, error_stat: error status */
  typedef boost::function<void(const std::string& file_name,int line_num, bool error_stat)> TCallback;

  TCallback  OnCommandPushed;
  TCallback  OnEndOfLine;
};

bool ParseFile (const std::string &filename, TBinaryStack &bin_stack, const TParserCallbacks &callbacks=TParserCallbacks());


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_parser_h
//-------------------------------------------------------------------------------------------
