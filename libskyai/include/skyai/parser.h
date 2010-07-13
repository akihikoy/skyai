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
//-------------------------------------------------------------------------------------------
// forward declaration:
namespace std {
  template < class T > class allocator;
  template < class T, class Allocator /*= allocator<T>*/ > class list;
}
namespace boost {namespace filesystem {
  struct path_traits;
  template<class String, class Traits> class basic_path;
  typedef basic_path< std::string, path_traits > path;
}}
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
class TAgent;
//-------------------------------------------------------------------------------------------

//===========================================================================================

#define LIST1(x_type)  std::list<x_type, std::allocator<x_type> >

/*!\brief load modules, connections, configurations from the file [filename]
    \param [in]path_list : path-list from which an agent file is searched
    \param [in,out]included_list  :  included full-path (native) list
    \note  If you use include_once for multiple LoadAgentFromFile, the same included_list should be specified */
bool LoadAgentFromFile (TAgent &agent, boost::filesystem::path file_path, bool *is_last=NULL,
                        LIST1(boost::filesystem::path) *path_list=NULL, LIST1(std::string) *included_list=NULL);

/*!\brief save modules, connections, configurations to the file [filename] */
bool SaveAgentToFile (const TAgent &agent, const boost::filesystem::path &file_path);

//! \todo implemen LoadAgentFromDir and SaveAgentToDir
#if 0
/*!\brief load modules, connections, configurations from the directory [dirname] */
bool LoadAgentFromDir (TAgent &agent, const std::string &dirname);

/*!\brief save modules, connections, configurations to the directory [dirname] */
bool SaveAgentToDir (const TAgent &agent, const std::string &dirname);
#endif

#undef LIST1

//===========================================================================================


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_parser_h
//-------------------------------------------------------------------------------------------
