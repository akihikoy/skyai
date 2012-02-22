//-------------------------------------------------------------------------------------------
/*! \file    agent_bindef.cpp
    \brief   libskyai - certain program (source)
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
#include <skyai/agent_bindef.h>
#include <lora/variable_bindef.h>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace agent_parser
{

void CopyCommand(int command, const TBinaryStack &src, TBinaryStack &dst)
{
  using namespace loco_rabbits::agent_parser::bin;
  if(command<cmd::COMMAND_BASE)
  {
    var_space::CopyCommand(command,src,dst);
    return;
  }
  dst.Push(command);
}
//-------------------------------------------------------------------------------------------

//! copy src to dst while pred is true
void CopyWhile(const TBinaryStack &src, TBinaryStack &dst, bool(*pred)(int command))
{
  int command;
  while(!src.IsEOD())
  {
    command= src.ReadI();
    if(!pred(command))  return;
    CopyCommand(command,src,dst);
  }
}
//-------------------------------------------------------------------------------------------

void PrintLineToStream(int command, const TBinaryStack &bstack, std::ostream &os)
{
  using namespace loco_rabbits::agent_parser::bin;
  using namespace std;
  if(command<cmd::COMMAND_BASE)
  {
    var_space::PrintLineToStream(command, bstack, os);
    return;
  }
  switch(command)
  {
  #define AS_IS(x_c) case x_c: os<<#x_c<<endl; break;
  AS_IS( cmd::LINCLUDE  )
  AS_IS( cmd::DUMP1     )
  AS_IS( cmd::DUMP2     )

  AS_IS( cmd::MODULE    )
  AS_IS( cmd::REMOVE    )
  AS_IS( cmd::CONNECT   )
  AS_IS( cmd::DISCNCT   )

  AS_IS( cmd::ASGN_GCNF )
  AS_IS( cmd::ASGN_CNF  )
  AS_IS( cmd::ASGN_MEM  )
  AS_IS( cmd::ASGN_END  )
  AS_IS( cmd::EDIT      )
  AS_IS( cmd::EDIT_END  )

  AS_IS( cmd::COMPOSITE )
  AS_IS( cmd::CMP_END   )
  AS_IS( cmd::FUNC_DEF  )
  AS_IS( cmd::FDEF_END  )
  AS_IS( cmd::S_PARAMS  )
  AS_IS( cmd::RETURN    )

  AS_IS( cmd::DESTROY   )

  AS_IS( cmd::INHERIT   )
  AS_IS( cmd::INHERITPR )

  AS_IS( cmd::EXPO_P    )
  AS_IS( cmd::EXPO_P_AS )
  AS_IS( cmd::EXPO_C    )
  AS_IS( cmd::EXPO_C_AS )
  AS_IS( cmd::EXPO_M    )
  AS_IS( cmd::EXPO_M_AS )
  #undef AS_IS
  default:  FIXME("unknown command code:"<<command);
  }
}
//-------------------------------------------------------------------------------------------

void PrintToStream(const TBinaryStack &bstack, std::ostream &os)
{
  bstack.GoFirst();
  while(!bstack.IsEOD())
  {
    PrintLineToStream(bstack.ReadI(), bstack, os);
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of agent_parser
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
