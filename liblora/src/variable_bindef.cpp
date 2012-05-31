//-------------------------------------------------------------------------------------------
/*! \file    variable_bindef.cpp
    \brief   liblora - certain program (source)
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
#include <lora/variable_bindef.h>
#include <lora/string.h>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{

void SkipCommand(int command, const TBinaryStack &bstack)
{
  using namespace loco_rabbits::var_space::bin;
  int vtype,num;
  switch(command)
  {
  case cmd::PUSH: // bin=[- vtype value]; push a value of vtype;
    vtype= bstack.ReadI();
    switch(vtype)
    {
    case vtype::ID   :  bstack.ReadS(); break;
    case vtype::INT  :  bstack.ReadI(); break;
    case vtype::REAL :  bstack.ReadR(); break;
    case vtype::BOOL :  bstack.ReadB(); break;
    case vtype::STR  :  bstack.ReadS(); break;
    case vtype::TYPE :  bstack.ReadI(); break;
    default:  FIXME("unknown value type code");
    }
    break;
  case cmd::PUSHL: // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
    num= bstack.ReadI();
    vtype= bstack.ReadI();
    switch(vtype)
    {
    case vtype::INT  :  for(;num>0;--num) bstack.ReadI(); break;
    case vtype::REAL :  for(;num>0;--num) bstack.ReadR(); break;
    case vtype::BOOL :  for(;num>0;--num) bstack.ReadB(); break;
    case vtype::STR  :  for(;num>0;--num) bstack.ReadS(); break;
    default:  FIXME("unknown value type code");
    }
    break;
  default:
    break;
  }
}
//-------------------------------------------------------------------------------------------

void CopyCommand(int command, const TBinaryStack &src, TBinaryStack &dst)
{
  using namespace loco_rabbits::var_space::bin;
  int vtype,num;
  dst.Push(command);
  switch(command)
  {
  case cmd::PUSH: // bin=[- vtype value]; push a value of vtype;
    vtype= src.ReadI();
    dst.Push(vtype);
    switch(vtype)
    {
    case vtype::ID   :  dst.Push(src.ReadS()); break;
    case vtype::INT  :  dst.Push(src.ReadI()); break;
    case vtype::REAL :  dst.Push(src.ReadR()); break;
    case vtype::BOOL :  dst.Push(src.ReadB()); break;
    case vtype::STR  :  dst.Push(src.ReadS()); break;
    case vtype::TYPE :  dst.Push(src.ReadI()); break;
    default:  FIXME("unknown value type code");
    }
    break;
  case cmd::PUSHL: // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
    num= src.ReadI();
    dst.Push(num);
    vtype= src.ReadI();
    dst.Push(vtype);
    switch(vtype)
    {
    case vtype::INT  :  for(;num>0;--num) dst.Push(src.ReadI()); break;
    case vtype::REAL :  for(;num>0;--num) dst.Push(src.ReadR()); break;
    case vtype::BOOL :  for(;num>0;--num) dst.Push(src.ReadB()); break;
    case vtype::STR  :  for(;num>0;--num) dst.Push(src.ReadS()); break;
    default:  FIXME("unknown value type code");
    }
    break;
  default:
    break;
  }
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
  using namespace loco_rabbits::var_space::bin;
  using namespace std;
  int num(0);
  switch(command)
  {
  case cmd::PUSH: // bin=[- vtype value]; push a value of vtype;
    os<<"cmd::PUSH ";
    switch(bstack.ReadI())
    {
    case vtype::ID   :  os<<"vtype::ID   "<<ConvertToStr(bstack.ReadS())<<endl; break;
    case vtype::INT  :  os<<"vtype::INT  "<<ConvertToStr(bstack.ReadI())<<endl; break;
    case vtype::REAL :  os<<"vtype::REAL "<<ConvertToStr(bstack.ReadR())<<endl; break;
    case vtype::BOOL :  os<<"vtype::BOOL "<<ConvertToStr(bstack.ReadB())<<endl; break;
    case vtype::STR  :  os<<"vtype::STR  "<<ConvertToStr(bstack.ReadS())<<endl; break;
    case vtype::TYPE :  os<<"vtype::TYPE "<<bin::TypeStr(bstack.ReadI())<<endl; break;
    default:  FIXME("unknown value type code");
    }
    break;
  case cmd::PUSHL: // bin=[- N vtype value1 .. valueN]; push a list (value1,..,valueN) of vtype;
    num= bstack.ReadI();
    os<<"cmd::PUSHL "<<num<<" ";
    switch(bstack.ReadI())
    {
    case vtype::INT  :  os<<"vtype::INT  "; for(;num>0;--num) os<<" "<<ConvertToStr(bstack.ReadI()); break;
    case vtype::REAL :  os<<"vtype::REAL "; for(;num>0;--num) os<<" "<<ConvertToStr(bstack.ReadR()); break;
    case vtype::BOOL :  os<<"vtype::BOOL "; for(;num>0;--num) os<<" "<<ConvertToStr(bstack.ReadB()); break;
    case vtype::STR  :  os<<"vtype::STR  "; for(;num>0;--num) os<<" "<<ConvertToStr(bstack.ReadS()); break;
    default:  FIXME("unknown value type code");
    }
    os<<endl;
    break;
  #define AS_IS(x_c) case x_c: os<<#x_c<<endl; break;
  AS_IS(cmd::LAPPEND   )
  AS_IS(cmd::PUSH_EMPL )
  AS_IS(cmd::LLISTS    )
  AS_IS(cmd::POP       )

  AS_IS(cmd::M_ASGN_P  )
  AS_IS(cmd::M_ASGN_CS )
  AS_IS(cmd::E_ASGN_P  )
  AS_IS(cmd::E_ASGN_CS )
  AS_IS(cmd::P_ASGN_P  )
  AS_IS(cmd::P_ASGN_CS )
  AS_IS(cmd::F_ASGN_P  )
  AS_IS(cmd::F_ASGN_CS )
  AS_IS(cmd::CASGN_END )

  AS_IS(cmd::FUNC_CALL )

  AS_IS(cmd::CONCAT )
  AS_IS(cmd::ADD    )
  AS_IS(cmd::SUBT   )
  AS_IS(cmd::MULT   )
  AS_IS(cmd::DIV    )
  AS_IS(cmd::MOD    )
  AS_IS(cmd::AND    )
  AS_IS(cmd::OR     )
  AS_IS(cmd::NOT    )
  AS_IS(cmd::EQ     )
  AS_IS(cmd::NEQ    )
  AS_IS(cmd::LTEQ   )
  AS_IS(cmd::GTEQ   )
  AS_IS(cmd::LT     )
  AS_IS(cmd::GT     )
  AS_IS(cmd::MEMBER )
  AS_IS(cmd::ELEM   )

  AS_IS(cmd::CAST   )

  AS_IS(cmd::T_TO_LIST )
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
}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
