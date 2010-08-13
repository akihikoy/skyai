//-------------------------------------------------------------------------------------------
/*! \file    parser.h
    \brief   libskyai - agent file (script) parser (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.17, 2009-

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

    \note  Usually, use TAgent::LoadFromFile and TAgent::SaveToFile
      rather than LoadAgentFromFile and SaveAgentToFile
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_parser_h
#define skyai_parser_h
//-------------------------------------------------------------------------------------------
#include <string>
#include <list>
#include <sstream>
#include <boost/filesystem/path.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
class TCompositeModule;
class TCompositeModuleGenerator;
class TFunctionManager;
class TAgent;
namespace var_space {class TLiteralTable;}
//-------------------------------------------------------------------------------------------

struct TAgentParseMode
{
  bool  NoExport;  //!< if true, export sentences are ignored (used in a private inheritance)
  bool  Phantom;
  bool  FunctionDef;
  TAgentParseMode() : NoExport(false), Phantom(false), FunctionDef(false)  {}
};
struct TAgentParserInfoIn
{
  TCompositeModule                     &CModule;
  TAgentParseMode                      ParseMode;
  int                                  StartLineNum;
  std::list<boost::filesystem::path>   *PathList;  //!< path-list from which an agent file is searched
  std::list<std::string>               *IncludedList;  //!< included full-path (native) list
                                       /*!\note If you use include_once for multiple LoadAgentFromFile,
                                                the same included_list should be specified */
  TCompositeModuleGenerator            *CmpModuleGenerator;
  TFunctionManager                     *FunctionManager;
  var_space::TLiteralTable             *LiteralTable;

  TAgentParserInfoIn(TCompositeModule &cmod)
    : CModule(cmod), StartLineNum(1), PathList(NULL), IncludedList(NULL), CmpModuleGenerator(NULL), FunctionManager(NULL), LiteralTable(NULL)  {}
};
struct TAgentParserInfoOut
{
  bool                 IsLast;
  int                  LastLineNum;
  std::stringstream    *EquivalentCode;  //! stored if Phantom is true
  TAgentParserInfoOut(void) : IsLast(false), LastLineNum(-1), EquivalentCode(NULL)  {}
};
//-------------------------------------------------------------------------------------------

//!\brief Execute a function whose contents is func_script
bool ExecuteFunction(const std::string &func_script,
      const boost::filesystem::path &current_dir, const std::string &file_name,
      TAgentParserInfoIn &in, TAgentParserInfoOut &out);
//-------------------------------------------------------------------------------------------


/*!\brief load modules, connections, configurations from the file [path_list] */
bool LoadAgentFromFile (boost::filesystem::path file_path, TAgentParserInfoIn &in, TAgentParserInfoOut &out);

/*!\brief save modules, connections, configurations to the file [path_list] */
bool SaveAgentToFile (const TAgent &agent, const boost::filesystem::path &file_path);


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_parser_h
//-------------------------------------------------------------------------------------------
