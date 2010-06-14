//-------------------------------------------------------------------------------------------
/*! \file    string.cpp
    \brief   liblora - string utility  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
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
#include <lora/string.h>
#include <boost/tokenizer.hpp>
#include <vector>
#include <list>
#include <iomanip>
#include <sstream>
#include <cfloat>
//-------------------------------------------------------------------------------------------
#include <lora/string_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

//===========================================================================================
// STRING OPERATIONS
//===========================================================================================

void TrimLeft (std::string &str)
{
  std::string::iterator last (str.begin());
  while (last!=str.end())
    if (*last==' ' || *last=='\t') ++last;
    else break;
  if (last!=str.begin())
    str.erase (str.begin(),last);
}
//-------------------------------------------------------------------------------------------

void TrimRight (std::string &str)
{
  if (str.length()==0) return;
  std::string::iterator first (str.end());
  --first;
  while (first!=str.begin())
    if (*first==' ' || *first=='\t') --first;
    else break;
  ++first;
  if (first!=str.end())
    str.erase (first,str.end());
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// STRING ENCODING and DECODING
//===========================================================================================

/*!\brief encode string STR to "STR" where special characters are escaped */
std::string EncodeString (const std::string &str)
{
  //!\todo extend to handle multi-byte letters
  std::stringstream ss;
  const char *c;
  ss<<'\"';
  for (std::string::const_iterator itr(str.begin()); itr!=str.end(); ++itr)
  {
    c= EncodeChar(*itr);
    if(c) ss<<c;  else ss<<*itr;
  }
  ss<<'\"';
  return ss.str();
}
//-------------------------------------------------------------------------------------------

/*!\brief decode string "STR" to STR where escaped characters are also decoded into special characters
    \note "STR", "STR, STR", and STR becomes STR
    \note if STR includes un-escaped `"' , the obtained STR terminated at that point. in this case,
          the remaining string is stored into rest if the rest is not NULL */
std::string DecodeString (const std::string &str, std::string *rest)
{
  //!\todo extend to handle multi-byte letters
  std::stringstream ss;
  std::string::const_iterator itr (str.begin());
  if (itr==str.end())  return "";
  if (*itr=='\"')  ++itr;
  for (; itr!=str.end(); ++itr)
  {
    if (*itr=='\"')
    {
      ++itr;
      break;
    }
    if (*itr=='\\')
    {
      ++itr;
      if (itr==str.end())
        {LERROR("string str["<<str<<"] terminates with `\\'"); lexit(df);}
      const char ec[]= {*itr};
      char c= DecodeChar(ec);
      if (c=='\0')  {lexit(df);}
      ss<<c;
    }
    else
      ss<<*itr;
  }
  if (rest && itr!=str.end())
  {
    std::stringstream rss;
    for (; itr!=str.end(); ++itr)
      rss<<*itr;
    *rest= rss.str();
  }
  return ss.str();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// TYPE CONVERSIONS
//===========================================================================================

/*!\note The following code uses "*_DIG+3" as a precision.  The value "3"
    is determined empirically.  Namely, if you use these conversion,
    the re-converted  value (real --\> string --\> real) is
    precisely equal to the original value.  The evaluation has been performed
    only on g++ 4.4.3.
    \todo Need more test.
*/

const std::string FloatToStr (const float &val)
{
  // return boost::lexical_cast<std::string>(val);
  const float EXP_TH(1.0e+8f);
  std::stringstream ss;
  if(val>EXP_TH || val<-EXP_TH)  ss<< std::scientific;
  ss<< std::setprecision(FLT_DIG+3) << val;
  return ss.str();
}
const std::string FloatToStr (const double &val)
{
  // return boost::lexical_cast<std::string>(val);
  const double EXP_TH(1.0e+8);
  std::stringstream ss;
  if(val>EXP_TH || val<-EXP_TH)  ss<< std::scientific;
  ss<< std::setprecision(DBL_DIG+3) << val;
  return ss.str();
}
const std::string FloatToStr (const long double &val)
{
  // return boost::lexical_cast<std::string>(val);
  const long double EXP_TH(1.0e+8l);
  std::stringstream ss;
  if(val>EXP_TH || val<-EXP_TH)  ss<< std::scientific;
  ss<< std::setprecision(LDBL_DIG+3) << val;
  return ss.str();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// READ FROM SEQUENCE
//===========================================================================================

void ReadSpacesFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  for (; first!=last; ++first)
  {
    if (*first==' '||*first=='\t')
      ss<<*first;
    else
      break;
  }
}
//-------------------------------------------------------------------------------------------

//! \note separators are [,\t\ ]*
void ReadSeparatorsFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  for (; first!=last; ++first)
  {
    if (*first==','||*first==' '||*first=='\t')
      ss<<*first;
    else
      break;
  }
}
//-------------------------------------------------------------------------------------------

//! \note separators are [,\t\ ]*
void ReadNonSeparatorsFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  for (; first!=last; ++first)
  {
    if (*first==','||*first==' '||*first=='\t')
      break;
    else
      ss<<*first;
  }
}
//-------------------------------------------------------------------------------------------

//! \note encoded-string should have a form: "STR" where STR can include escape sequences (e.g. '\"')
void ReadEncodedStrFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  if (first!=last&&*first=='\"')
  {
    ss<<*first;
    ++first;
  }
  else
    return;
  for (; first!=last; ++first)
  {
    if (*first=='\"')
    {
      ss<<*first;
      ++first;
      break;
    }
    if (*first=='\\')
    {
      ss<<*first;
      ++first;
      if (first==last)
        {LERROR("string ["<<ss<<"] terminates with `\\'"); lexit(df);}
      ss<<*first;
    }
    else
      ss<<*first;
  }
}
//-------------------------------------------------------------------------------------------

//! \note identifier is a string which consists of [0-9A-Za-z_]
void ReadIdentifierFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  for (; first!=last; ++first)
  {
    if (('0'<=*first && *first<='9') ||
        ('A'<=*first && *first<='Z') ||
        ('a'<=*first && *first<='z') || *first=='_')
      ss<<*first;
    else
      break;
  }
}
//-------------------------------------------------------------------------------------------

void ReadNumbersFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  for (; first!=last; ++first)
  {
    if (*first<'0' || *first>'9')  break;
    ss<<*first;
  }
}
//-------------------------------------------------------------------------------------------

void ReadIntFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  //!\todo FIXME: this function treats 001 as a proper integer
  ReadNumbersFromStr (ss, first,last);
}
//-------------------------------------------------------------------------------------------

void ReadFloatFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  //!\todo FIXME: this function treats 002.5 as a proper floating point number
  if ((first!=last)&&(*first=='+'||*first=='-'))
  {
    ss<<*first;
    ++first;
  }
  ReadNumbersFromStr(ss, first, last);
  if ((first!=last)&&(*first=='.'))
  {
    ss<<*first;
    ++first;
    ReadNumbersFromStr(ss, first, last);
  }
  if ((first!=last)&&(*first=='e'||*first=='E'))
  {
    ss<<*first;
    ++first;
    if ((first!=last)&&(*first=='+'||*first=='-'))
    {
      ss<<*first;
      ++first;
    }
    const std::string::const_iterator tmp= first;
    ReadNumbersFromStr(ss, first, last);
    if (tmp==first)  {LERROR("invalid floating number format: "<<ss); lexit(df);}
  }
}
//-------------------------------------------------------------------------------------------

void ReadAllFromStr (std::stringstream &ss, std::string::const_iterator &first, const std::string::const_iterator &last)
{
  for (; first!=last; ++first)
    ss<<*first;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// SEQUENCE OPERATIONS
//===========================================================================================

/*!\brief split 'str' at each 'separators' , store them into 'strlist'. if skip_blank=true, blank character("") is skipped
  \note t_string_container is instantiated with std::list\<std::string\>, and  std::vector\<std::string\>
*/
template <typename t_string_container>
void SplitString (t_string_container &strlist, const std::string &str, bool skip_blank, const char *separators)
{
  const boost::char_separator<char>  sep(separators);
  strlist.clear();
  if (str == "")  return;
  boost::tokenizer< boost::char_separator<char> >  tokens (str, sep);
  boost::tokenizer< boost::char_separator<char> >::iterator itr= tokens.begin();
  for (; itr!=tokens.end(); ++itr)
    if (*itr != "" || !skip_blank)
      strlist.push_back (*itr);
}
template void SplitString (std::list<std::string> &, const std::string &, bool, const char *);
template void SplitString (std::vector<std::string> &, const std::string &, bool, const char *);
//-------------------------------------------------------------------------------------------


// explicit instantiation
#define INSTANTIATOR(_type)  \
  template std::string NumericalContainerToString (const std::list<_type> &vec, const std::string &delim=" "); \
  template std::string NumericalContainerToString (const std::vector<_type> &vec, const std::string &delim=" "); \
  template void StringToNumericalContainer (const std::string &line, std::list<_type> &res); \
  template void StringToNumericalContainer (const std::string &line, std::vector<_type> &res);
INSTANTIATOR(unsigned int)
INSTANTIATOR(signed int)
INSTANTIATOR(double)
INSTANTIATOR(long double)
INSTANTIATOR(bool)
// INSTANTIATOR(std::string)
//----
INSTANTIATOR(unsigned short)
INSTANTIATOR(unsigned char)
INSTANTIATOR(unsigned long)
INSTANTIATOR(signed short)
INSTANTIATOR(signed char)
INSTANTIATOR(signed long)
INSTANTIATOR(float)
#undef INSTANTIATOR
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
