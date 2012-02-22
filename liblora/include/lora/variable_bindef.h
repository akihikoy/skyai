//-------------------------------------------------------------------------------------------
/*! \file    variable_bindef.h
    \brief   liblora - certain program (header)
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
#ifndef loco_rabbits_variable_bindef_h
#define loco_rabbits_variable_bindef_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space_fwd.h>
#include <lora/binary.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
namespace bin
{

namespace cmd  // command
{
#define DEF_CMD(x_name,x_code)  const int x_name (x_code);
  // -: command
  DEF_CMD( PUSH      ,  0 ) // bin=[- vtype value]; push a value of vtype;
  DEF_CMD( PUSHL     ,  1 ) // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
  DEF_CMD( LAPPEND   ,  2 ) // bin=[-]; pop two values(1,2), append 1 to 2:(2,1), push the result;
  DEF_CMD( PUSH_EMPL ,  3 ) // bin=[-]; push an empty list;
  DEF_CMD( LLISTS    , 10 ) // bin=[-]; start list-of-literals (in l-o-l, PUSH, PUSHL, VLIST{S,E} are available);
  DEF_CMD( POP       , 20 ) // bin=[-]; pop a value;

  DEF_CMD( M_ASGN_P  , 100 ) // bin=[-]; pop two values(1,2;2 shoud be an identifier), assign:2=1;
  DEF_CMD( M_ASGN_CS , 101 ) // bin=[-]; pop an identifier, start composite assign:id={..};
  DEF_CMD( E_ASGN_P  , 102 ) // bin=[-]; pop two values(1,2), elemental assign:[2]=1;
  DEF_CMD( E_ASGN_CS , 103 ) // bin=[-]; pop a value, start elemental composite assign:[val]={..};
  DEF_CMD( P_ASGN_P  , 104 ) // bin=[-]; pop a value, push:[]=val;
  DEF_CMD( P_ASGN_CS , 105 ) // bin=[-]; start composite push:[]={..};
  DEF_CMD( F_ASGN_P  , 106 ) // bin=[-]; pop a value, fill:[@]=val;
  DEF_CMD( F_ASGN_CS , 107 ) // bin=[-]; start composite fill:[@]={..};
  DEF_CMD( CASGN_END , 150 ) // finish M_ASGN_CS, E_ASGN_CS, P_ASGN_CS, F_ASGN_CS;

  DEF_CMD( FUNC_CALL , 200 ) // bin=[-]; pop list-of-literals, pop an identifier, call function:id(l-o-l);

  DEF_CMD( CONCAT , 1000 ) // bin=[-]; pop two values(1,2;2 shoud be an identifier), concatenate:2##1, push the result as an identifier;
  DEF_CMD( ADD    , 1010 ) // bin=[-]; pop two values, add them(2+1), push the result;
  DEF_CMD( SUBT   , 1011 ) // bin=[-]; pop two values, subtract them(2-1), push the result;
  DEF_CMD( MULT   , 1012 ) // bin=[-]; pop two values, multiply them(2*1), push the result;
  DEF_CMD( DIV    , 1013 ) // bin=[-]; pop two values, divide them(2/1), push the result;
  DEF_CMD( MOD    , 1014 ) // bin=[-]; pop two values, compute mod(2%1), push the result;
  DEF_CMD( AND    , 1020 ) // bin=[-]; pop two values, compute and(2&&1), push the result;
  DEF_CMD( OR     , 1021 ) // bin=[-]; pop two values, compute or(2||1), push the result;
  DEF_CMD( NOT    , 1022 ) // bin=[-]; pop a value, compute not(!1), push the result;
  DEF_CMD( EQ     , 1023 ) // bin=[-]; pop two values, compute equality(2==1), push the result;
  DEF_CMD( NEQ    , 1024 ) // bin=[-]; pop two values, compute inequality(2!=1), push the result;
  DEF_CMD( LTEQ   , 1025 ) // bin=[-]; pop two values, compute relation(2<=1), push the result;
  DEF_CMD( GTEQ   , 1026 ) // bin=[-]; pop two values, compute relation(2>=1), push the result;
  DEF_CMD( LT     , 1027 ) // bin=[-]; pop two values, compute relation(2<1), push the result;
  DEF_CMD( GT     , 1028 ) // bin=[-]; pop two values, compute relation(2>1), push the result;
  DEF_CMD( MEMBER , 1050 ) // bin=[-]; pop two values(1,2;1 shoud be an identifier), get a member ref(2.1), push the result;
  DEF_CMD( ELEM   , 1051 ) // bin=[-]; pop two values, get an elemental ref(2[1]), push the result;

  DEF_CMD( CAST   , 2000 ) // bin=[-]; pop two values(1,2;2 should be a type), cast 1 to 2(cast<2>(1), push the result;

  DEF_CMD( T_TO_LIST , 10000 ) // bin=[-]; pop a type(1), push the list of it (list<t>);
#undef DEF_CMD
}
namespace vtype  // value type
{
#define DEF_CMD(x_name,x_code)  const int x_name (x_code);
  DEF_CMD( ID    , 0   ) // identifier
  DEF_CMD( INT   , 1   ) // integer
  DEF_CMD( REAL  , 2   ) // real value
  DEF_CMD( BOOL  , 3   ) // boolean
  DEF_CMD( STR   , 4   ) // string
  DEF_CMD( TYPE  , 100 ) // type
  DEF_CMD( LIST  , 110 ) // list
#undef DEF_CMD
}

inline const char* TypeStr(int t)
{
  switch(t)
  {
  case vtype::INT  : return "int" ;
  case vtype::REAL : return "real";
  case vtype::BOOL : return "bool";
  case vtype::STR  : return "str" ;
  case vtype::LIST : return "list";
  default:  LERROR("invalid type code: "<<t); lexit(df);
  }
  return "-";
}
inline int TypeCode(const std::string &s)
{
  if     (s=="int" )  return vtype::INT ;
  else if(s=="real")  return vtype::REAL;
  else if(s=="bool")  return vtype::BOOL;
  else if(s=="str" )  return vtype::STR ;
  else if(s=="list")  return vtype::LIST;
  else  LERROR("invalid type string: "<<s); lexit(df);
  return -1;
}

}  // end of bin


#define DEF_ADD_PUSH_LITERAL(x_type, x_typeid)                    \
  inline void AddPushLiteral(TBinaryStack &bstack, const x_type &value)  \
  {                                                         \
    bstack.Push(bin::cmd::PUSH);                            \
    bstack.Push(bin::vtype::x_typeid);                      \
    bstack.Push(value);                                     \
  }

DEF_ADD_PUSH_LITERAL(unsigned short  , INT  )
DEF_ADD_PUSH_LITERAL(unsigned int    , INT  )
DEF_ADD_PUSH_LITERAL(unsigned long   , INT  )
DEF_ADD_PUSH_LITERAL(signed short    , INT  )
DEF_ADD_PUSH_LITERAL(signed int      , INT  )
DEF_ADD_PUSH_LITERAL(signed long     , INT  )
DEF_ADD_PUSH_LITERAL(bool            , BOOL )
DEF_ADD_PUSH_LITERAL(std::string     , STR  )

DEF_ADD_PUSH_LITERAL(float           , REAL )
DEF_ADD_PUSH_LITERAL(double          , REAL )
DEF_ADD_PUSH_LITERAL(long double     , REAL )

#undef DEF_ADD_PUSH_LITERAL

inline void AddPushID(TBinaryStack &bstack, const TIdentifier &value)
{
  bstack.Push(bin::cmd::PUSH);
  bstack.Push(bin::vtype::ID);
  bstack.Push(value);
}
inline void AddCommand(TBinaryStack &bstack, const int &value)
{
  bstack.Push(value);
}


void CopyCommand(int command, const TBinaryStack &src, TBinaryStack &dst);

//! copy src to dst while pred is true
void CopyWhile(const TBinaryStack &src, TBinaryStack &dst, bool(*pred)(int command));

void PrintLineToStream(int command, const TBinaryStack &bstack, std::ostream &os=std::cout);

void PrintToStream(const TBinaryStack &bstack, std::ostream &os=std::cout);


}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_bindef_h
//-------------------------------------------------------------------------------------------
