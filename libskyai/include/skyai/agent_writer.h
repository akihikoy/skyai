//-------------------------------------------------------------------------------------------
/*! \file    agent_writer.h
    \brief   libskyai - certain program (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.07, 2012

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
#ifndef skyai_agent_writer_h
#define skyai_agent_writer_h
//-------------------------------------------------------------------------------------------
#include <string>
#include <boost/filesystem/path.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
class TCompositeModule;
class TAgent;
namespace var_space {class TLiteralTable;}
//-------------------------------------------------------------------------------------------

/*!\brief save modules, connections, configurations to the file [path_list] */
bool SaveAgentToFile (const TAgent &agent, const boost::filesystem::path &file_path, const std::string &ext_file_prefix="");

/*!\brief save modules, connections, configurations to the stream [os] */
bool WriteAgentToStream (const TAgent &agent, std::ostream &os, bool ext_sto_available=false);
//-------------------------------------------------------------------------------------------

/*!\brief dump information */
bool DumpCModInfo (const TCompositeModule &cmodule, const std::string &filename,
      const std::string &kind, const std::string *opt=NULL, const std::string &indent="");
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_agent_writer_h
//-------------------------------------------------------------------------------------------
