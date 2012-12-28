//-------------------------------------------------------------------------------------------
/*! \file    agent_parser.h
    \brief   libskyai - certain program (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.03, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
#ifndef skyai_agent_parser_h
#define skyai_agent_parser_h
//-------------------------------------------------------------------------------------------
#include <lora/binary.h>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace agent_parser
{
//-------------------------------------------------------------------------------------------

struct TParserCallbacks
{
  /*! \brief event callback in parsing a file;
         file_name: current file name, line_num: current line num, error_stat: error status */
  typedef boost::function<void(const std::string &file_name,int line_num, bool error_stat)> TCallback;

  /*! \brief callback function to for add_path and load_library
         return true if succeeded */
  typedef boost::function<bool(const std::string &str)> TCBBool_Str;

  /*! \brief callback function to get a path from file_name by searching a path list
         return true if file_name exists, false if file_name doesn't exist */
  typedef boost::function<bool(const std::string &file_name, std::string &abs_file_name)> TGetFilePath;

  TCallback      OnCommandPushed;
  TCallback      OnEndOfLine;
  TCBBool_Str    OnAddPath;
  TCBBool_Str    OnLoadLibrary;
  TGetFilePath   OnInclude;
  TGetFilePath   OnIncludeOnce;  //!< this callback should assign "" to abs_file_name if file_name is already loaded
};

bool ParseFile (const std::string &file_name, TBinaryStack &bin_stack, const TParserCallbacks &callbacks=TParserCallbacks());


//-------------------------------------------------------------------------------------------
}  // end of agent_parser
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_agent_parser_h
//-------------------------------------------------------------------------------------------
