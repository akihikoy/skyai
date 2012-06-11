//-------------------------------------------------------------------------------------------
/*! \file    agent_bindef.h
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
#ifndef skyai_agent_bindef_h
#define skyai_agent_bindef_h
//-------------------------------------------------------------------------------------------
#include <lora/binary.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace agent_parser
{
namespace bin
{

namespace cmd  // command
{
  const int COMMAND_BASE(100000);
#define DEF_CMD(x_name,x_code)  const int x_name (COMMAND_BASE+x_code);
  // -: command
  DEF_CMD( LINCLUDE  , 0 ) // bin=[-]; pop two values(1,2; 1:str,2:id), add str to the lazy-include-list of id;
  DEF_CMD( DUMP1     , 1 ) // bin=[-]; pop two strings(1,2), dump 2 into file 1;
  DEF_CMD( DUMP2     , 2 ) // bin=[-]; pop three strings(1,2,3), dump 3,2 into file 1;
  DEF_CMD( PRINT     , 5 ) // bin=[-]; pop a value(1), print 1 into std-out;

  DEF_CMD( MODULE    , 10 ) // bin=[-]; pop two identifiers(1,2), create module: type 2, id 1;
  DEF_CMD( REMOVE    , 11 ) // bin=[-]; pop an identifier(1), remove module: id 1;
  DEF_CMD( CONNECT   , 12 ) // bin=[-]; pop four identifiers(1,2,3,4), connect 4.3-2.1;
  DEF_CMD( DISCNCT   , 13 ) // bin=[-]; pop four identifiers(1,2,3,4), disconnect 4.3-2.1;

  DEF_CMD( ASGN_GCNF , 100 ) // bin=[-]; start assigning to context module's config;
  DEF_CMD( ASGN_GMEM , 101 ) // bin=[-]; start assigning to context module's memory;
  DEF_CMD( ASGN_CNF  , 102 ) // bin=[-]; pop an identifier, start assigning to its config;
  DEF_CMD( ASGN_MEM  , 103 ) // bin=[-]; pop an identifier, start assigning to its memory;
  DEF_CMD( ASGN_END  , 104 ) // bin=[-]; end assigning;
  DEF_CMD( EDIT      , 105 ) // bin=[-]; pop an identifier, start editing it;
  DEF_CMD( EDIT_END  , 106 ) // bin=[-]; end editing;

  DEF_CMD( COMPOSITE , 1000 ) // bin=[-]; pop an identifier, start defining a composite module;
  DEF_CMD( CMP_END   , 1001 ) // bin=[-]; end defining the composite module;
  DEF_CMD( FUNC_DEF  , 1002 ) // bin=[-]; pop identifiers (parameters,func-name), start defining a function;
  DEF_CMD( FDEF_END  , 1003 ) // bin=[-]; end defining the function;
  DEF_CMD( S_PARAMS  , 1004 ) // bin=[-]; start parameters of function-definition;
  DEF_CMD( RETURN    , 1005 ) // bin=[-]; pop a value, set it as a return value, exit the execution;

  DEF_CMD( DESTROY   , 1100 ) // bin=[-]; pop two values(1,2; 1:id,2:str), destroy 1 of kind 2;

  DEF_CMD( INHERIT   , 1200 ) // bin=[-]; pop an identifier, inherit the module;
  DEF_CMD( INHERITPR , 1201 ) // bin=[-]; pop an identifier, inherit_prv the module;

  DEF_CMD( EXPO_P    , 1300 ) // bin=[-]; pop two identifier(1,2), export the port  2.1 as 1;
  DEF_CMD( EXPO_P_AS , 1301 ) // bin=[-]; pop three identifier(1,2,3), export the port  3.2 as 1;
  DEF_CMD( EXPO_C    , 1302 ) // bin=[-]; pop two identifier(1,2), export the config  2.config.1 as 1;
  DEF_CMD( EXPO_C_AS , 1303 ) // bin=[-]; pop three identifier(1,2,3), export the config  3.config.2 as 1;
  DEF_CMD( EXPO_M    , 1304 ) // bin=[-]; pop two identifier(1,2), export the memory  2.memory.1 as 1;
  DEF_CMD( EXPO_M_AS , 1305 ) // bin=[-]; pop three identifier(1,2,3), export the memory  3.memory.2 as 1;

  DEF_CMD( CTRL_IF     , 5000 ) // bin=[- value]; pop a value, if false: skip until finding [ELSE value] or [END_IF value], if true: do nothing;
  DEF_CMD( CTRL_ELSE   , 5001 ) // bin=[- value]; skip until finding [END_IF value];
  DEF_CMD( CTRL_END_IF , 5002 ) // bin=[- value]; do nothing;
#undef DEF_CMD
}  // end of cmd

}  // end of bin


void SkipCommand(int command, const TBinaryStack &bstack);

void CopyCommand(int command, const TBinaryStack &src, TBinaryStack &dst);

//! copy src to dst while pred is true
void CopyWhile(const TBinaryStack &src, TBinaryStack &dst, bool(*pred)(int command));

void PrintLineToStream(int command, const TBinaryStack &bstack, std::ostream &os=std::cout);
void PrintToStream(const TBinaryStack &bstack, std::ostream &os=std::cout);


//-------------------------------------------------------------------------------------------
}  // end of agent_parser
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_agent_bindef_h
//-------------------------------------------------------------------------------------------
