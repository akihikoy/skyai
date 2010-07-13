//-------------------------------------------------------------------------------------------
/*! \file    utility.h
    \brief   libskyai - utility functions
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.08, 2010

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
#ifndef skyai_utility_h
#define skyai_utility_h
//-------------------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
class TOptionParser;
class TAgent;
//-------------------------------------------------------------------------------------------


/*!\brief Parse command line option for an instance of TAgent

    usage:
    \code
    int main(int argc, char**argv)
    {
      TOptionParser option(argc,argv);
      TAgent  agent;
      if(!ParseCmdLineOption (agent, option))  return 0;
      ...
    }
    \endcode
*/
bool ParseCmdLineOption (TAgent &agent, TOptionParser &option, std::ostream &debug_stream=std::cout);

/*!\brief Parse command line option for an instance of TAgent

    usage:
    \code
    int main(int argc, char**argv)
    {
      TOptionParser option(argc,argv);
      TAgent  agent;
      std::ofstream debug;
      if(!ParseCmdLineOption (agent, option, debug))  return 0;
      ...
    }
    \endcode
*/
bool ParseCmdLineOption (TAgent &agent, TOptionParser &option, std::ofstream &debug_fstream);


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_utility_h
//-------------------------------------------------------------------------------------------
