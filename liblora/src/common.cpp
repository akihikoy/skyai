//-------------------------------------------------------------------------------------------
/*! \file    common.cpp
    \brief   liblora : loco_rabbits C++ libraries
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-
    \note    use -rdynamic linker option to show function name in StackTrace
    \date    Apr.21, 2010  Added template functions: TypeMax, TypeMin, RealMax, RealMin

    Copyright (C) 2008, 2009, 2010  Akihiko Yamaguchi

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
#include <lora/common.h>
#include <cstdlib>  // for exit, abort
#ifdef __GLIBC__
  #include <execinfo.h>  // NOTE: glibc only; for backtrace* functions
#endif
#include <string>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
//-------------------------------------------------------------------------------------------

static void StackTrace (void)
{
#ifdef __GLIBC__
  void *trace[256];
  int n = backtrace(trace, sizeof(trace) / sizeof(trace[0]));
  backtrace_symbols_fd(trace, n, STDERR_FILENO);
#else
  LWARNING("StackTrace is not implemented to this system.");
#endif
}
//-------------------------------------------------------------------------------------------

namespace loco_rabbits_detail_exitlv
{
void i_i_lexit (
    TExitLevel exit_level,
    int linenum,
    const char *filename,
    const char *functionname)
{
  if (exit_level != success)
    std::cerr<<ioscc::green<<"------"<<std::endl
      <<ioscc::green<<"the program is terminated"<<std::endl
      <<ioscc::green<<"  at "<<ioscc::blue<<filename<<ioscc::green<<":"
                             <<ioscc::blue<<linenum<<ioscc::green<<":"<<std::endl
      <<ioscc::green<<"  within the function: "<<ioscc::blue<<functionname<<std::endl;
      // <<ioscc::green<<"the program is terminated at line: "<<linenum<<std::endl
      // <<ioscc::green<<"  within the function: "<<functionname<<std::endl
      // <<ioscc::green<<"  in the file: "<<filename<<std::endl;
  switch (exit_level)
  {
    case success :
      std::exit(EXIT_SUCCESS);
    case qfail   :
      std::exit(EXIT_FAILURE);
    case btfail  :
      std::cerr<<"backtrace:"<<std::endl;
      StackTrace();
      std::exit(EXIT_FAILURE);
    case abort   :
      std::abort();
    default      :
      std::cerr<<"improper usage of LEXIT (_lexit); invalid exit_level: "
        <<static_cast<int>(exit_level)<<std::endl;
      std::exit(EXIT_FAILURE);
  }
}
//-------------------------------------------------------------------------------------------
} // end of namespace loco_rabbits_detail_exitlv
//-------------------------------------------------------------------------------------------


void SaveArguments (int argc, char const*const*argv, std::ostream &os)
{
  os<< *argv;
  ++argv;
  for (--argc; argc>0; --argc,++argv)
  {
    os<< " ";
    for (const char *str(*argv); *str!='\0'; ++str)
    {
      switch(*str)
      {
        case '\\': case '!': case '#': case '$': case '%': case '^': case '&': case '*':
        case '(': case ')': case '`': case '~': case '[': case '{': case '}': case ']':
        case '\'': case '\"': case '?': case '<': case '>': case ' ':
          os<< "\\";
      }
      os<< *str;
    }
  }
  os<<std::endl;
}
//-------------------------------------------------------------------------------------------

bool AskYesNo (std::ostream &os)
{
#define _alwaysY "alY"
#define _alwaysN "alN"
  const char *prompt= "  (y|n|"_alwaysY"|"_alwaysN"|?) > ";
  static bool alwaysYes(false), alwaysNo(false);
  if (alwaysYes)      {std::cerr<<prompt<<"yes"<<std::endl; return true;}
  else if (alwaysNo)  {std::cerr<<prompt<<"no"<<std::endl; return false;}
  string str;
  while(1)
  {
    std::cerr<<prompt<<std::flush;
    std::cin>>str;
    if (str[0]=='y' || str[0]=='Y')       return true;
    else if (str[0]=='n' || str[0]=='N')  return false;
    else if (str==_alwaysY) {alwaysYes= true; return true;}
    else if (str==_alwaysN) {alwaysNo = true; return false;}
    else if (str[0]=='?')
    {
      std::cerr
        <<"    y   : answer \"yes\""<<std::endl
        <<"    n   : answer \"no\""<<std::endl
        <<"    alY : always answer \"yes\" (be careful to choose)"<<std::endl
        <<"    alN : always answer \"no\" (be careful to choose)"<<std::endl
        <<"    ?   : show this help"<<std::endl;
    }
    std::cin.clear();
    std::cin.ignore(1000,'\n');
  }
#undef _alwaysY
#undef _alwaysN
}
//-------------------------------------------------------------------------------------------

TResYesNoCancel AskYesNoCancel (std::ostream &os)
{
#define _alwaysY "alY"
#define _alwaysN "alN"
#define _alwaysC "alC"
  const char *prompt= "  (y|n|c|"_alwaysY"|"_alwaysN"|"_alwaysC"|?) > ";
  static bool alwaysYes(false), alwaysNo(false), alwaysCancel(false);
  if (alwaysYes)          {std::cerr<<prompt<<"yes"   <<std::endl; return ryncYes;}
  else if (alwaysNo)      {std::cerr<<prompt<<"no"    <<std::endl; return ryncNo;}
  else if (alwaysCancel)  {std::cerr<<prompt<<"cancel"<<std::endl; return ryncCancel;}
  string str;
  while(1)
  {
    std::cerr<<prompt<<std::flush;
    std::cin>>str;
    if (str[0]=='y' || str[0]=='Y')       return ryncYes;
    else if (str[0]=='n' || str[0]=='N')  return ryncNo;
    else if (str[0]=='c' || str[0]=='C')  return ryncCancel;
    else if (str==_alwaysY) {alwaysYes    = true; return ryncYes;}
    else if (str==_alwaysN) {alwaysNo     = true; return ryncNo;}
    else if (str==_alwaysC) {alwaysCancel = true; return ryncCancel;}
    else if (str[0]=='?')
    {
      std::cerr
        <<"    y   : answer \"yes\""<<std::endl
        <<"    n   : answer \"no\""<<std::endl
        <<"    alY : always answer \"yes\" (be careful to choose)"<<std::endl
        <<"    alN : always answer \"no\" (be careful to choose)"<<std::endl
        <<"    alC : always answer \"cancel\" (be careful to choose)"<<std::endl
        <<"    ?   : show this help"<<std::endl;
    }
    std::cin.clear();
    std::cin.ignore(1000,'\n');
  }
#undef _alwaysY
#undef _alwaysN
#undef _alwaysC
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
