//-------------------------------------------------------------------------------------------
/*! \file    string.h
    \brief   liblora - string utility  (header)
    \author  Akihiko Yamaguchi
    \date    Nov.30, 2008-
    \date    Oct. 12, 2009 : Implemented ConvertFromStr, ConvertToStr, NumericalContainerToString
    \date    May. 20, 2010 : Modified FloatToStr so that ConvertToFloat(FloatToStr(x)) is precisely equal to x

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
#ifndef loco_rabbits_string_h
#define loco_rabbits_string_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <string>
#include <boost/lexical_cast.hpp>
// #include <boost/format.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
// STRING OPERATIONS
//===========================================================================================

template<> inline void SetZero (std::string    &val)  {val = "";}
//-------------------------------------------------------------------------------------------

inline std::string operator* (const int &n, const std::string &s)
{
  std::string res;
  for(int i(0);i<n;++i)  res+=s;
  return res;
}
inline std::string operator* (const std::string &s, const int &n)
{
  return operator*(n,s);
}
//-------------------------------------------------------------------------------------------

inline bool IsComment (const std::string &str)
{
  return (str.size()>=1 && str[0]=='#') || (str.size()>=2 && str[0]=='/' && str[1]=='/');
}
//-------------------------------------------------------------------------------------------

void TrimLeft (std::string &str);
void TrimRight (std::string &str);

inline void TrimBoth (std::string &str)
{
  TrimLeft (str);
  TrimRight (str);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// STRING ENCODING and DECODING
//===========================================================================================

inline bool IsContolChar (char c)
{
  switch(c)
  {
    case '\a':
    case '\b':
    case '\f':
    case '\n':
    case '\r':
    case '\t':
    case '\v':
    case '\\':
    case '\?':
    case '\'':
    case '\"':
    return true;
  }
  return false;
}

inline const char* EncodeChar (char c)
{
  switch(c)
  {
    case '\a': return "\\a";
    case '\b': return "\\b";
    case '\f': return "\\f";
    case '\n': return "\\n";
    case '\r': return "\\r";
    case '\t': return "\\t";
    case '\v': return "\\v";
    case '\\': return "\\\\";
    case '\?': return "\\?";
    case '\'': return "\\\'";
    case '\"': return "\\\"";
  }
  return NULL;
}

inline char DecodeChar (const char *c)
{
  switch(c[0])
  {
    case 'a':  return '\a';
    case 'b':  return '\b';
    case 'f':  return '\f';
    case 'n':  return '\n';
    case 'r':  return '\r';
    case 't':  return '\t';
    case 'v':  return '\v';
    case '\\': return '\\';
    case '?':  return '\?';
    case '\'': return '\'';
    case '\"': return '\"';
  }
  LERROR("invalid escape sequence: \\"<<c[0]);
  return '\0';
}
//-------------------------------------------------------------------------------------------

/*!\brief encode string STR to "STR" where special characters are escaped */
std::string EncodeString (const std::string &str);

/*!\brief decode string "STR" to STR where escaped characters are also decoded into special characters
    \note "STR", "STR, STR", and STR becomes STR
    \note if STR includes un-escaped `"' , the obtained STR terminated at that point. in this case,
          the remaining string is stored into rest if the rest is not NULL */
std::string DecodeString (const std::string &str, std::string *rest=NULL);

//-------------------------------------------------------------------------------------------

//===========================================================================================
// TYPE IDENTIFICATION
//===========================================================================================

inline bool IsSpace (char c)
{
  if(c==' '||c=='\t')  return true;
  return false;
}
inline bool IsEOL (char c)
{
  if(c=='\n'||c=='\r')  return true;
  return false;
}
inline bool IsSpaceEOL (char c)
{
  if(IsSpace(c)||IsEOL(c))  return true;
  return false;
}

inline bool IsNumber (char c)
{
  if('0'<=c && c<='9')  return true;
  return false;
}

inline bool IsAlphabetL (char c)
{
  if('a'<=c && c<='z')  return true;
  return false;
}
inline bool IsAlphabetH (char c)
{
  if('A'<=c && c<='Z')  return true;
  return false;
}
inline bool IsAlphabet (char c)  {return IsAlphabetL(c)||IsAlphabetH(c);}
inline bool IsAlphNum (char c)  {return IsAlphabet(c)||IsNumber(c);}
inline bool IsSymbol (char c)
{
  if(IsNumber(c) || IsAlphabet(c) || c=='_')  return false;
  if(' '<=c && c<='~')  return true;
  if(c=='\t')  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

bool IsNumberSeq (const std::string &str);
bool IsAlphabetSeq (const std::string &str);
bool IsAlphNumSeq (const std::string &str);
bool IsIdentifier (const std::string &str);
//-------------------------------------------------------------------------------------------


//===========================================================================================
// TYPE CONVERSIONS
//===========================================================================================

inline const std::string IntToStr (int val)
{
  return boost::lexical_cast<std::string>(val);
}
inline const std::string IntToStr (int val, int length, char fill='0')
{
  std::string  tmp (boost::lexical_cast<std::string>(val));
  int rm= length-tmp.length();
  if(rm>0)  return std::string(rm,fill)+tmp;
  return tmp;
}
//-------------------------------------------------------------------------------------------

const std::string FloatToStr (const float &val);
const std::string FloatToStr (const double &val);
const std::string FloatToStr (const long double &val);
//-------------------------------------------------------------------------------------------

inline const std::string BoolToStr (const bool &val)
{
  return val ? std::string("true") : std::string("false");
}
//-------------------------------------------------------------------------------------------

inline const int StrToInt (const std::string &str)
{
  return boost::lexical_cast<int>( str );
}
//-------------------------------------------------------------------------------------------

inline const TReal StrToFloat (const std::string &str)
{
  if (str=="inf")  return REAL_MAX;
  else if (str=="-inf")  return -REAL_MAX;
  return boost::lexical_cast<TReal>( str );
}
//-------------------------------------------------------------------------------------------

inline const bool StrToBool (const std::string &str)
{
  if (str=="true"||str=="TRUE"||str=="True"||str=="1")  return true;
  if (str=="false"||str=="FALSE"||str=="False"||str=="0")  return false;
  if (IsNumberSeq(str))  return StrToInt(str);
  LERROR("failed to convert " << str << " to bool");
  lexit(df);
  return false;
}
//-------------------------------------------------------------------------------------------

template <typename t_from> inline const std::string ConvertToStr (const t_from &val);
template <typename t_to>   inline const t_to ConvertFromStr (const std::string &str);

// specialize ConvertToStr and ConvertFromStr for basic types
#define SPECIALIZER(x_type,x_t2s,x_s2t) \
  template <> inline const std::string ConvertToStr (const x_type &val)   {return x_t2s(val);} \
  template <> inline const x_type ConvertFromStr (const std::string &str) {return x_s2t(str);}
#define SPECIALIZER_LC(x_type) \
  template <> inline const std::string ConvertToStr (const x_type &val) \
    {return boost::lexical_cast<std::string>(val);} \
  template <> inline const x_type ConvertFromStr (const std::string &str) \
    {return boost::lexical_cast<x_type>(str);}
SPECIALIZER_LC(signed int     )
SPECIALIZER_LC(signed short   )
SPECIALIZER_LC(signed long    )
SPECIALIZER_LC(signed char    )
SPECIALIZER_LC(unsigned int   )
SPECIALIZER_LC(unsigned short )
SPECIALIZER_LC(unsigned long  )
SPECIALIZER_LC(unsigned char  )
SPECIALIZER(float       ,  FloatToStr  ,  StrToFloat  )
SPECIALIZER(double      ,  FloatToStr  ,  StrToFloat  )
SPECIALIZER(long double ,  FloatToStr  ,  StrToFloat  )
// SPECIALIZER_LC(float          )
// SPECIALIZER_LC(double         )
// SPECIALIZER_LC(long double    )
SPECIALIZER(bool        ,  BoolToStr   ,  StrToBool   )
// SPECIALIZER(std::string ,              ,              )
SPECIALIZER(std::string ,  EncodeString,  DecodeString)
#undef SPECIALIZER_LC
#undef SPECIALIZER

//-------------------------------------------------------------------------------------------

template <typename t_enum>
struct TEnumMapCell
{
  t_enum e;
  const char *s;
};

//!\brief Generate a map of an enum type x_enum and string. Then generate ConvertToStr and ConvertFromStr using the map.
//!\note use these macros for loco_rabbits::x_enum
//!\note use these macros in the namespace loco_rabbits
//!\todo make available in some namespaces other than loco_rabbits
#define ENUM_STR_MAP_BEGIN(x_enum)  \
  static const loco_rabbits::TEnumMapCell<x_enum> LORA_##x_enum##_MAP[]={

#define ENUM_STR_MAP_ADD(x_element) \
  {x_element,#x_element},

//! \todo modify so that ConvertFromStr can convert from string of numbers [0-9]
//! \todo (use std::map to make faster)
#define ENUM_STR_MAP_END(x_enum)    \
  {static_cast<x_enum>(-1),""}};                                                                 \
  template <> inline const std::string ConvertToStr (const x_enum &val)                          \
  {                                                                                              \
    for (const loco_rabbits::TEnumMapCell<x_enum> *p=LORA_##x_enum##_MAP,                        \
                *pend=LORA_##x_enum##_MAP+SIZE_OF_ARRAY(LORA_##x_enum##_MAP)-1; p!=pend; ++p)    \
      if (p->e==val) return p->s;                                                                \
    LERROR(static_cast<int>(val)<<" is not an element of " #x_enum);                             \
    lexit(df); return "";                                                                        \
  }                                                                                              \
  template <> inline const x_enum ConvertFromStr (const std::string &str)                        \
  {                                                                                              \
    for (const loco_rabbits::TEnumMapCell<x_enum> *p=LORA_##x_enum##_MAP,                        \
                *pend=LORA_##x_enum##_MAP+SIZE_OF_ARRAY(LORA_##x_enum##_MAP)-1; p!=pend; ++p)    \
      if (std::string(p->s)==str) return p->e;                                                   \
    LERROR(str<<" is not an element of " #x_enum);                                               \
    lexit(df); return static_cast<x_enum>(-1);                                                   \
  }

//!\brief Generate a map of an enum type x_enum and string. Then generate ConvertToStr and ConvertFromStr using the map.
//!\note use these macros for loco_rabbits::x_namespace::x_enum
//!\note use these macros in the namespace loco_rabbits
#define ENUM_STR_MAP_BEGIN_NS(x_namespace,x_enum)  \
  static const loco_rabbits::TEnumMapCell<x_namespace::x_enum> LORA_##x_enum##_MAP[]={

#define ENUM_STR_MAP_ADD_NS(x_namespace,x_element) \
  {x_namespace::x_element,#x_element},

//! \todo modify so that ConvertFromStr can convert from string of numbers [0-9]
//! \todo (use std::map to make faster)
#define ENUM_STR_MAP_END_NS(x_namespace,x_enum)    \
  {static_cast<x_namespace::x_enum>(-1),""}};                                                    \
  template <> inline const std::string ConvertToStr (const x_namespace::x_enum &val)             \
  {                                                                                              \
    for (const loco_rabbits::TEnumMapCell<x_namespace::x_enum> *p=LORA_##x_enum##_MAP,           \
                *pend=LORA_##x_enum##_MAP+SIZE_OF_ARRAY(LORA_##x_enum##_MAP)-1; p!=pend; ++p)    \
      if (p->e==val) return p->s;                                                                \
    LERROR(static_cast<int>(val)<<" is not an element of " #x_namespace "::" #x_enum);           \
    lexit(df); return "";                                                                        \
  }                                                                                              \
  template <> inline const x_namespace::x_enum ConvertFromStr (const std::string &str)           \
  {                                                                                              \
    for (const loco_rabbits::TEnumMapCell<x_namespace::x_enum> *p=LORA_##x_enum##_MAP,           \
                *pend=LORA_##x_enum##_MAP+SIZE_OF_ARRAY(LORA_##x_enum##_MAP)-1; p!=pend; ++p)    \
      if (std::string(p->s)==str) return p->e;                                                   \
    LERROR(str<<" is not an element of " #x_namespace "::" #x_enum);                             \
    lexit(df); return static_cast<x_namespace::x_enum>(-1);                                      \
  }

//-------------------------------------------------------------------------------------------


//===========================================================================================
// READ FROM SEQUENCE
//===========================================================================================

void ReadSpacesFromStr        (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadSpaceEOLsFromStr     (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadSymbolsFromStr       (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadSeparatorsFromStr    (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadNonSeparatorsFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadEncodedStrFromStr    (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadIdentifierFromStr    (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadNumbersFromStr       (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadIntFromStr           (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadFloatFromStr         (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);
void ReadAllFromStr           (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last);

class TTokenizer
{
public:
  TTokenizer () {Init();}
  TTokenizer (const std::string &str)  {Init(str);}

  void Init()
    {
      pos_= line_.begin();
      last_= line_.end();
    }
  void Init(const std::string &str)
    {
      line_= str;
      pos_= line_.begin();
      last_= line_.end();
    }
  bool EOL()  {return pos_==last_;}

  #define GEN_FUNC(x_func)  std::string x_func()  {std::stringstream ss; x_func ## FromStr (ss, pos_, last_); return ss.str();}
  GEN_FUNC( ReadSpaces        )
  GEN_FUNC( ReadSpaceEOLs     )
  GEN_FUNC( ReadSymbols       )
  GEN_FUNC( ReadSeparators    )
  GEN_FUNC( ReadNonSeparators )
  GEN_FUNC( ReadEncodedStr    )
  GEN_FUNC( ReadIdentifier    )
  GEN_FUNC( ReadNumbers       )
  GEN_FUNC( ReadInt           )
  GEN_FUNC( ReadFloat         )
  GEN_FUNC( ReadAll           )
  #undef GEN_FUNC

  template <typename t_value> inline std::string ReadValue();

private:
  std::string                  line_;
  std::string::const_iterator  pos_;
  std::string::const_iterator  last_;

  TTokenizer (const TTokenizer &);
  const TTokenizer& operator= (const TTokenizer &);

};
//-------------------------------------------------------------------------------------------
template<> inline std::string TTokenizer::ReadValue<signed int    >()   {return ReadInt();}
template<> inline std::string TTokenizer::ReadValue<signed short  >()   {return ReadInt();}
template<> inline std::string TTokenizer::ReadValue<signed long   >()   {return ReadInt();}
template<> inline std::string TTokenizer::ReadValue<unsigned int  >()   {return ReadInt();}
template<> inline std::string TTokenizer::ReadValue<unsigned short>()   {return ReadInt();}
template<> inline std::string TTokenizer::ReadValue<unsigned long >()   {return ReadInt();}

template<> inline std::string TTokenizer::ReadValue<float      >()   {return ReadFloat();}
template<> inline std::string TTokenizer::ReadValue<double     >()   {return ReadFloat();}
template<> inline std::string TTokenizer::ReadValue<long double>()   {return ReadFloat();}

template<> inline std::string TTokenizer::ReadValue<bool>()   {return ReadIdentifier();}

template<> inline std::string TTokenizer::ReadValue<std::string>()   {return ReadEncodedStr();}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------


//===========================================================================================
// SEQUENCE OPERATIONS
//===========================================================================================

/*!\brief Write str to a stream os.  Each line is indented with indent */
void IndentString (const std::string &str, std::ostream &os, const std::string &indent);
struct TIndentString
{
  const std::string &Str;
  const std::string &Indent;
  TIndentString(const std::string &s, const std::string &i) : Str(s), Indent(i) {}
};
inline std::ostream& operator<<(std::ostream &lhs, const TIndentString &rhs)  {IndentString(rhs.Str,lhs,rhs.Indent); return lhs;}

/*!\brief split 'str' at each 'separators' , store them into 'strlist'. if skip_blank=true, blank character("") is skipped
  \note t_string_container is instantiated with std::list\<std::string\>, and  std::vector\<std::string\>
*/
template <typename t_string_container>
void SplitString (t_string_container &strlist, const std::string &str, bool skip_blank=true, const char *separators=" \t\n");

/*!\brief Convert a numerical array (vector,list,etc.) 'vec' to characters
  \note t_container is instantiated with every combination of {std::list, std::vector} and {int, double, ...}.
      If you need the other instances, include lora/string_impl.h
*/
template <typename t_container>
std::string NumericalContainerToString (const t_container &vec, const std::string &delim=" ");

/*!\brief Convert a string line to a numerical array, and store it into res
  \note t_container is instantiated with every combination of {std::list, std::vector} and {int, double, ...}.
      If you need the other instances, include lora/string_impl.h
*/
template <typename t_container>
void StringToNumericalContainer (const std::string &line, t_container &res);
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_string_h
//-------------------------------------------------------------------------------------------

