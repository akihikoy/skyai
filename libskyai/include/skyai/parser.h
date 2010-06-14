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
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_parser_h
#define skyai_parser_h
//-------------------------------------------------------------------------------------------
#include <string>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
class TAgent;
//-------------------------------------------------------------------------------------------

//===========================================================================================

/*!\brief load modules, connections, configurations from the file [filename] */
bool LoadAgentFromFile (TAgent &agent, const std::string &filename, bool *is_last=NULL);

/*!\brief save modules, connections, configurations to the file [filename] */
bool SaveAgentToFile (const TAgent &agent, const std::string &filename);

//! \todo implemen LoadAgentFromDir and SaveAgentToDir
#if 0
/*!\brief load modules, connections, configurations from the directory [dirname] */
bool LoadAgentFromDir (TAgent &agent, const std::string &dirname);

/*!\brief save modules, connections, configurations to the directory [dirname] */
bool SaveAgentToDir (const TAgent &agent, const std::string &dirname);
#endif

//===========================================================================================


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_parser_h
//-------------------------------------------------------------------------------------------
