//-------------------------------------------------------------------------------------------
/*! \file    common.cpp
    \brief   liblora : loco_rabbits C++ libraries
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-
    \note    use -rdynamic linker option to show function name in StackTrace
    \date    Apr.21, 2010  Added template functions: TypeMax, TypeMin, RealMax, RealMin
    \date    Jul.26, 2010  Added message_system::SetFormat to change message format of LERROR, etc.
    \date    Jul.26, 2010  Added exitlv::th which throw an exception, added exitlv::ChangeDefault
    \date    Jul.26, 2010  Added ioscc::Disable, etc. to change color scheme

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
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
//-------------------------------------------------------------------------------------------

namespace message_system
{
  LORA_MESSAGE_FORMAT_FUNCTION  fmt_function= &DefaultFormat;
  namespace detail
  {
    void OutputMessage(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss)
    {
      fmt_function(type, linenum, filename, functionname, ss);
    }
  }
  void DefaultFormat(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss)
  {
    switch (type)
    {
      case  mtMessage  : std::cerr<<ioscc::blue<<ss.str()<<std::endl; return;
      case  mtError    : std::cerr<<ioscc::red<<"error("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtWarning  : std::cerr<<ioscc::red<<"warning("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtDebug    : std::cerr<<ioscc::red<<"debug("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtTodo     : std::cerr<<ioscc::green<<"todo("<<filename<<":"<<linenum<<"): "<<ss.str()<<std::endl; return;
      case  mtFixme    : std::cerr<<ioscc::red<<"FIX "<<filename<<":"<<linenum<<": "<<ss.str()<<std::endl; return;
      default : std::cerr<<ioscc::red<<"loco_rabbits: systam fatal!"<<std::endl;
    }
  }
  // template SetFormat and GetFormat:
  template<typename t_format_function>
  void SetFormat(t_format_function fmt)
  {
    fmt_function= fmt;
  }
  template<typename t_format_function> t_format_function GetFormat(void)
  {
    return fmt_function;
  }
  // instantiations
  template void SetFormat(LORA_MESSAGE_FORMAT_FUNCTION);
  template void SetFormat(void (*)(TMessageType,int,const char*,const char*,std::stringstream&));
  template LORA_MESSAGE_FORMAT_FUNCTION GetFormat(void);
}
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

namespace exitlv
{
  // static const TExitLevel SYSTEM_DEFAULT_EXIT_LEVEL(btfail);
  static const TExitLevel SYSTEM_DEFAULT_EXIT_LEVEL(abort);
  static TExitLevel DEFAULT_EXIT_LEVEL(SYSTEM_DEFAULT_EXIT_LEVEL);

  namespace detail
  {
    void i_lexit (
        TExitLevel exit_level,
        int linenum,
        const char *filename,
        const char *functionname)
    {
      if (exit_level==df)
        exit_level= DEFAULT_EXIT_LEVEL;
      if (exit_level!=success && exit_level!=th)
        std::cerr<<ioscc::green<<"------"<<std::endl
          <<ioscc::green<<"the program is terminated"<<std::endl
          <<ioscc::green<<"  at "<<ioscc::blue<<filename<<ioscc::green<<":"
                                <<ioscc::blue<<linenum<<ioscc::green<<":"<<std::endl
          <<ioscc::green<<"  within the function: "<<ioscc::blue<<functionname<<std::endl;
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
        case th      :
          throw TException(linenum,filename,functionname);
        default      :
          std::cerr<<"improper usage of lexit(exit_level); invalid exit_level: "
            <<static_cast<int>(exit_level)<<std::endl;
          std::exit(EXIT_FAILURE);
      }
    }
  }  // end of detail

  void ChangeDefault (TExitLevel exit_level)
  {
    if(exit_level==df)
      DEFAULT_EXIT_LEVEL= SYSTEM_DEFAULT_EXIT_LEVEL;
    else
      DEFAULT_EXIT_LEVEL= exit_level;
  }
//-------------------------------------------------------------------------------------------
} // end of namespace exitlv
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// colored out
//-------------------------------------------------------------------------------------------
namespace ioscc
{
  static const char *const default_code_esc    = "\033[";
  static const char *const default_code_reset  = "0m"   ;
  static const char *const default_code_red    = "31;1m";
  static const char *const default_code_green  = "32;1m";
  static const char *const default_code_blue   = "34;1m";

  namespace detail
  {
    const char *code_esc   = default_code_esc  ;
    const char *code_reset = default_code_reset;
    const char *code_red   = default_code_red  ;
    const char *code_green = default_code_green;
    const char *code_blue  = default_code_blue ;
  }
  //!\brief Disable colored out
  void Disable(void)
  {
    using namespace detail;
    code_esc   = "";
    code_reset = "";
    code_red   = "";
    code_blue  = "";
    code_green = "";
  }
  //!\brief Change color scheme
  void SetScheme(const char *v_red, const char *v_green, const char *v_blue)
  {
    using namespace detail;
    ResetScheme();
    code_red   = v_red  ;
    code_green = v_green;
    code_blue  = v_blue ;
  }
  //!\brief Reset color scheme to default
  void ResetScheme()
  {
    using namespace detail;
    code_esc   = default_code_esc  ;
    code_reset = default_code_reset;
    code_red   = default_code_red  ;
    code_green = default_code_green;
    code_blue  = default_code_blue ;
  }
}  // end of ioscc
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



//-------------------------------------------------------------------------------------------
static bool AlwaysYes (false);
static bool AlwaysNo (false);
static bool AlwaysCancel(false);
//-------------------------------------------------------------------------------------------

bool AskYesNo (std::ostream &os)
{
#define _alwaysY "alY"
#define _alwaysN "alN"
  const char *prompt= "  (y|n|"_alwaysY"|"_alwaysN"|?) > ";
  if (AlwaysYes)      {std::cerr<<prompt<<"yes"<<std::endl; return true;}
  else if (AlwaysNo)  {std::cerr<<prompt<<"no"<<std::endl; return false;}
  string str;
  while(1)
  {
    std::cerr<<prompt<<std::flush;
    std::cin>>str;
    if (str[0]=='y' || str[0]=='Y')       return true;
    else if (str[0]=='n' || str[0]=='N')  return false;
    else if (str==_alwaysY) {AlwaysYes= true; return true;}
    else if (str==_alwaysN) {AlwaysNo = true; return false;}
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
  if (AlwaysYes)          {std::cerr<<prompt<<"yes"   <<std::endl; return ryncYes;}
  else if (AlwaysNo)      {std::cerr<<prompt<<"no"    <<std::endl; return ryncNo;}
  else if (AlwaysCancel)  {std::cerr<<prompt<<"cancel"<<std::endl; return ryncCancel;}
  string str;
  while(1)
  {
    std::cerr<<prompt<<std::flush;
    std::cin>>str;
    if (str[0]=='y' || str[0]=='Y')       return ryncYes;
    else if (str[0]=='n' || str[0]=='N')  return ryncNo;
    else if (str[0]=='c' || str[0]=='C')  return ryncCancel;
    else if (str==_alwaysY) {AlwaysYes    = true; return ryncYes;}
    else if (str==_alwaysN) {AlwaysNo     = true; return ryncNo;}
    else if (str==_alwaysC) {AlwaysCancel = true; return ryncCancel;}
    else if (str[0]=='?')
    {
      std::cerr
        <<"    y   : answer \"yes\""<<std::endl
        <<"    n   : answer \"no\""<<std::endl
        <<"    c   : answer \"cancel\""<<std::endl
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
