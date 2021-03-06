//-------------------------------------------------------------------------------------------
/*! \file    common.h
    \brief   liblora : loco_rabbits C++ libraries (common header)
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

    -----------------------------------------------------------------------------------------

    \note If you want to disable to show colored message, use ioscc::Disable() function
      in a main() function.  Use ioscc::ResetScheme to enable.

*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_common_h
#define loco_rabbits_common_h
//-------------------------------------------------------------------------------------------
#include <cmath>
#include <climits>
#include <cfloat>
#include <sstream>
#include <iostream>
#include <boost/current_function.hpp>
// #include <boost/type_traits/remove_const.hpp>
// #include <boost/type_traits/add_const.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

// extended keywords:
#define override  // that describes a function is overridden

typedef long double TReal;

static const TReal REAL_MIN (LDBL_MIN);
static const TReal REAL_MAX (LDBL_MAX);
static const TReal REAL_PI  (M_PIl);
static const TReal REAL_2PI (2.0l*REAL_PI);
static const TReal DBL_TINY (1.0e-300l);
static const TReal DBL_TINY_EXP (-700.0l);

// NOTE: pow is overloaded as pow(float,float),pow(double,double),and so on. thus, don't use powl or powf
// NOTE: use std::pow instead of pow. the latter one calls pow in C-lib
#define real_acos    std::acos
#define real_asin    std::asin
#define real_atan    std::atan
#define real_atan2   std::atan2
#define real_ceil    std::ceil
#define real_cos     std::cos
#define real_cosh    std::cosh
#define real_exp     std::exp
#define real_fabs    std::fabs
#define real_floor   std::floor
#define real_fmod    std::fmod
#define real_frexp   std::frexp
#define real_ldexp   std::ldexp
#define real_log     std::log
#define real_log10   std::log10
#define real_modf    std::modf
#define real_pow     std::pow
#define real_sin     std::sin
#define real_sinh    std::sinh
#define real_sqrt    std::sqrt
#define real_tan     std::tan
#define real_tanh    std::tanh

#define real_round   roundl // this function is provided by math.h (not cmath)

static const int NullIndex (-1);

//-------------------------------------------------------------------------------------------
// convert _code to string; if the _code is a macro, it is not expanded, i.e. the result is "macro"
// FIXME we strongly recommend BOOST_PP_STRINGIZE
#define CODE_TO_STR(_code) #_code
/* convert _cdoe to string, if the _code is a macro, it is expanded
    WARNING: this will depend on gcc/g++ because
        `The order of evaluation of # and ## operators is unspecified.'
        (ISO/IEC 14882:2003 (E): 16.3.2 The # operator)  */
#define CODE_TO_STR_M(_code) CODE_TO_STR(_code)
//-------------------------------------------------------------------------------------------
#define LORA_CURRENT_FUNCTION  BOOST_CURRENT_FUNCTION
//-------------------------------------------------------------------------------------------
namespace message_system
{
  enum TMessageType {mtMessage=0, mtError, mtWarning, mtDebug, mtTodo, mtFixme};
  namespace detail
  {
    void OutputMessage(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss);
  }
  void DefaultFormat(TMessageType type, int linenum, const char *filename, const char *functionname, std::stringstream &ss);
  #define LORA_MESSAGE_FORMAT_FUNCTION  ::boost::function< void(::loco_rabbits::message_system::TMessageType,int,const char*,const char*,::std::stringstream&) >
  /*!\brief Set message format
      \note SetFormat is instantiated for
              t_format_function = LORA_MESSAGE_FORMAT_FUNCTION , and
              t_format_function = void (*)(TMessageType,int,const char*,const char*,std::stringstream&) */
  template<typename t_format_function> void SetFormat(t_format_function fmt);
  /*!\brief Get message format
      \note SetFormat is instantiated only for LORA_MESSAGE_FORMAT_FUNCTION.  The reason of being a template
              is not to include boost/function.hpp in this header file */
  template<typename t_format_function> t_format_function GetFormat(void);
}
#define L_OUTPUT_MESSAGE(x_type,x_msg) ::loco_rabbits::message_system::detail::OutputMessage( \
              ::loco_rabbits::message_system::x_type, __LINE__,  __FILE__, LORA_CURRENT_FUNCTION, x_msg)
#define LMESSAGE(msg)  do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtMessage,ss);}while(0)
#define LERROR(msg)    do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtError  ,ss);}while(0)
#define LWARNING(msg)  do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtWarning,ss);}while(0)
#define LDEBUG(msg)    do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtDebug  ,ss);}while(0)
#define LDBGVAR(var)   do{std::stringstream ss; ss<<#var"= "<<(var); L_OUTPUT_MESSAGE(mtDebug,ss);}while(0)
#define LTODO(msg)     do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtTodo   ,ss);}while(0)
#define FIXME(msg)     do{std::stringstream ss; ss<<msg; L_OUTPUT_MESSAGE(mtFixme  ,ss);lexit(df);}while(0)
//-------------------------------------------------------------------------------------------
#define LASSERT(eqn)                                        \
    do{if(!(eqn)){                                          \
      LERROR("assertion failed: (" #eqn ")");               \
      lexit(df);                                            \
    }}while(0)
#define LASSERT1op1(lhs,op,rhs)                             \
    do{if(!((lhs) op (rhs))){                               \
      LERROR("assertion failed: (" #lhs")" #op "("#rhs")"); \
      std::cerr<<"  ("#lhs")= "<<(lhs)<<std::endl;          \
      std::cerr<<"  ("#rhs")= "<<(rhs)<<std::endl;          \
      lexit(df);                                            \
    }}while(0)
//-------------------------------------------------------------------------------------------
#define SIZE_OF_ARRAY(array)  (sizeof(array)/sizeof((array)[0]))
//-------------------------------------------------------------------------------------------

namespace exitlv
{
struct TException
{
  int LineNum;
  const char *FileName;
  const char *FunctionName;
  TException (int v_linenum,const char *v_filename,const char *v_functionname)
    : LineNum(v_linenum), FileName(v_filename), FunctionName(v_functionname)  {}
};

enum TExitLevel {
  success=0  /*! exit quietly, and return success code */,
  qfail      /*! exit with printing where the program is terminated,
                  and return failure code*/,
  btfail     /*! exit with printing where the program is terminated
                  and printing the backtrace of stack,
                  and return failure code*/,
  abort      /*! exit with printing where the program is terminated,
                  and abort. note that a core file will be generated */,
  th         /*! throw an exception of the type TException */,
  df=1000    /*! exit with default level which is modifiable by ChangeDefault;
                  many functions in loco_rabbits use this exit level */};

namespace detail
{
  void i_lexit (
      TExitLevel exit_level,
      int linenum,
      const char *filename,
      const char *functionname);
}
//! \brief Change default exit scheme
void ChangeDefault (TExitLevel exit_level);
//-------------------------------------------------------------------------------------------
} // end of namespace exitlv
//-------------------------------------------------------------------------------------------
#define lexit(x_level)  ::loco_rabbits::exitlv::detail::i_lexit( \
                                ::loco_rabbits::exitlv::x_level, \
                                __LINE__,  __FILE__, LORA_CURRENT_FUNCTION)
//-------------------------------------------------------------------------------------------

/*!\brief get a dummy typed value dereferenced from NULL
  use this object with lexit to avoid the warning: "control reaches end of non-void function" */
template <typename _type>
struct dummy_return
{
  static _type& value()
    {
      return *static_cast<_type*>(NULL);
    }
};

//! partial specialization for reference type
template <typename _type>
struct dummy_return<_type&>
{
  static _type& value()
    {
      return dummy_return<_type>::value();
    }
};

//! partial specialization for void
template <>
struct dummy_return<void>
{
  static void value()
    {
      return;
    }
};
//-------------------------------------------------------------------------------------------

template<class T>
inline void SetZero (T &val);

template<> inline void SetZero (unsigned short &val)  {val = 0;}
template<> inline void SetZero (unsigned char  &val)  {val = 0;}
template<> inline void SetZero (unsigned int   &val)  {val = 0;}
template<> inline void SetZero (unsigned long  &val)  {val = 0;}
template<> inline void SetZero (signed short   &val)  {val = 0;}
template<> inline void SetZero (signed char    &val)  {val = 0;}
template<> inline void SetZero (signed int     &val)  {val = 0;}
template<> inline void SetZero (signed long    &val)  {val = 0;}
template<> inline void SetZero (float          &val)  {val = 0.0f;}
template<> inline void SetZero (double         &val)  {val = 0.0;}
template<> inline void SetZero (long double    &val)  {val = 0.0l;}
template<> inline void SetZero (bool           &val)  {val = false;}
//-------------------------------------------------------------------------------------------
template<class T>
inline T GetZero (void)
{
  T val;
  SetZero(val);
  return val;
}
//-------------------------------------------------------------------------------------------

template<class T>
inline void SetOne (T &val);

template<> inline void SetOne (int &val)         {val = 1;}
template<> inline void SetOne (double &val)      {val = 1.0;}
template<> inline void SetOne (long double &val) {val = 1.0l;}
//-------------------------------------------------------------------------------------------
template<class T>
inline T GetOne (void)
{
  T val;
  SetOne(val);
  return val;
}
//-------------------------------------------------------------------------------------------

template<typename T>inline T TypeMax();
template<typename T>inline T TypeMin();
template<typename T>inline T RealMax();
template<typename T>inline T RealMin();

template<>inline        signed char TypeMax() {return  SCHAR_MAX;}
template<>inline      unsigned char TypeMax() {return  UCHAR_MAX;}
template<>inline               char TypeMax() {return   CHAR_MAX;}
template<>inline          short int TypeMax() {return   SHRT_MAX;}
template<>inline unsigned short int TypeMax() {return  USHRT_MAX;}
template<>inline                int TypeMax() {return    INT_MAX;}
template<>inline       unsigned int TypeMax() {return   UINT_MAX;}
template<>inline           long int TypeMax() {return   LONG_MAX;}
template<>inline  unsigned long int TypeMax() {return  ULONG_MAX;}

template<>inline        signed char TypeMin() {return  SCHAR_MIN;}
template<>inline               char TypeMin() {return   CHAR_MIN;}
template<>inline          short int TypeMin() {return   SHRT_MIN;}
template<>inline                int TypeMin() {return    INT_MIN;}
template<>inline           long int TypeMin() {return   LONG_MIN;}

template<>inline              float RealMax() {return    FLT_MAX;}
template<>inline             double RealMax() {return    DBL_MAX;}
template<>inline        long double RealMax() {return   LDBL_MAX;}

template<>inline              float RealMin() {return    FLT_MIN;}
template<>inline             double RealMin() {return    DBL_MIN;}
template<>inline        long double RealMin() {return   LDBL_MIN;}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//!\brief get the dereferenced type (e.g. dereferenced_type<int*>::type is int)
//-------------------------------------------------------------------------------------------
template <typename T>
struct dereferenced_type
{
  typedef typename T::value_type type;
};
template <typename T>
struct dereferenced_type<T*>
{
  typedef T type;
};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//!\brief eliminate 'const' for given type (e.g. elim_const_type<const int>::type is int)
//-------------------------------------------------------------------------------------------
template <typename T>
struct elim_const_type
{
  typedef T type;
};
template <typename T>
struct elim_const_type <const T>
{
  typedef T type;
};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//\brief add 'const' for given value (e.g. add_const_value(x) is const)
//-------------------------------------------------------------------------------------------
// template <typename T>
// const T& to_const (const T &val)
// {
//   return val;
// }
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// colored out
//-------------------------------------------------------------------------------------------
namespace ioscc
{
  enum TColorCode {none=0, red=1, blue, green};

  namespace detail
  {
    extern const char *code_esc   ;
    extern const char *code_reset ;
    extern const char *code_red   ;
    extern const char *code_green ;
    extern const char *code_blue  ;
  }

  //!\brief Disable colored out
  void Disable(void);
  //!\brief Change color scheme
  void SetScheme(const char *v_red, const char *v_green, const char *v_blue);
  //!\brief Reset color scheme to default
  void ResetScheme();
}

struct ColoredStream
{
  std::ostream &Os;
};

inline ColoredStream operator<< (std::ostream &os, ioscc::TColorCode ccode)
{
  switch (ccode)
  {
    case ioscc::none   : os<<ioscc::detail::code_esc<<ioscc::detail::code_reset; break;
    case ioscc::red    : os<<ioscc::detail::code_esc<<ioscc::detail::code_red;   break;
    case ioscc::green  : os<<ioscc::detail::code_esc<<ioscc::detail::code_green; break;
    case ioscc::blue   : os<<ioscc::detail::code_esc<<ioscc::detail::code_blue;  break;
    default  :  std::cerr<<"invalid color code: "<<static_cast<int>(ccode)<<std::endl;  lexit(df);
  }
  ColoredStream colos={os};
  return colos;
}

template <typename T>
inline ColoredStream operator<< (ColoredStream os, const T     &rhs)
{
  os.Os << rhs;
  return os;
}
template <>
inline ColoredStream operator<< (ColoredStream os, const ioscc::TColorCode &ccode)
{
  os.Os << ioscc::detail::code_esc<<ioscc::detail::code_reset;
  os.Os << ccode;
  return os;
}
inline ColoredStream operator<< (ColoredStream os, std::ostream& (*pf)(std::ostream&))
{
  os.Os << ioscc::detail::code_esc<<ioscc::detail::code_reset;
  pf(os.Os);
  return os;
}
//-------------------------------------------------------------------------------------------


void SaveArguments (int argc, char const*const*argv, std::ostream &os);
bool AskYesNo (std::ostream &os=std::cerr);
enum TResYesNoCancel {ryncYes=0, ryncNo, ryncCancel};
TResYesNoCancel AskYesNoCancel (std::ostream &os=std::cerr);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_common_h
//-------------------------------------------------------------------------------------------

