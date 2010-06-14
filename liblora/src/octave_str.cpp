//-------------------------------------------------------------------------------------------
/*! \file    octave_str.cpp
    \brief   liblora - liboctave extension about string processing  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-
    \date    Jan.02, 2009  implement StringToRowVector

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
#include <lora/octave_str.h>
#include <lora/string.h>
#include <lora/bits/string.h>
#include <lora/string_list.h>
//-------------------------------------------------------------------------------------------
#include <octave/config.h>
#include <octave/dColVector.h>
#include <octave/dRowVector.h>
#include <octave/dMatrix.h>
//-------------------------------------------------------------------------------------------
#ifdef PACKAGE_BUGREPORT
  #undef PACKAGE_BUGREPORT
  #undef PACKAGE_NAME
  #undef PACKAGE_STRING
  #undef PACKAGE_TARNAME
  #undef PACKAGE_VERSION
#endif
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

//===========================================================================================
ColumnVector StringToColumnVector (const std::string &line)
//===========================================================================================
{
  if (line=="") return ColumnVector(0);
  std::list<double> dlist;
  std::string::const_iterator itr (line.begin());
  string_detail::SkipNotNumberCharacters (itr, line.end());
  while (itr!=line.end())
  {
    dlist.push_back(string_detail::ReadFloatFromStr(itr,line.end()));
    string_detail::SkipNotNumberCharacters (itr, line.end());
  }
  ColumnVector res(dlist.size());
  int r(0);
  for(std::list<double>::const_iterator ditr(dlist.begin()); ditr!=dlist.end(); ++ditr,++r)
    res(r) = *ditr;
  return res;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
RowVector StringToRowVector (const std::string &line)
//===========================================================================================
{
  if (line=="") return RowVector(0);
  std::list<double> dlist;
  std::string::const_iterator itr (line.begin());
  string_detail::SkipNotNumberCharacters (itr, line.end());
  while (itr!=line.end())
  {
    dlist.push_back(string_detail::ReadFloatFromStr(itr,line.end()));
    string_detail::SkipNotNumberCharacters (itr, line.end());
  }
  RowVector res(dlist.size());
  int r(0);
  for(std::list<double>::const_iterator ditr(dlist.begin()); ditr!=dlist.end(); ++ditr,++r)
    res(r) = *ditr;
  return res;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
std::string ColumnVectorToString (const ColumnVector &val)
//===========================================================================================
{
  std::stringstream ss;
  ss<<val.transpose();
  return ss.str();
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
std::string RowVectorToString (const RowVector &val)
//===========================================================================================
{
  std::stringstream ss;
  ss<<val;
  return ss.str();
}
//-------------------------------------------------------------------------------------------



template <>
void SaveToStringList (const Matrix &m, TStringListEx &str_list, const std::string &prefix)
{
  /*
      rows 2
      cols 3
      row 0 1.2 1.5 2.1
      row 1 3.3 2.1 5.2
    end
  */
  #define tostr(_s) ConvertToStr(_s)
  const std::string indent("  ");
  const std::string blank(" ");
  str_list.Add (prefix+indent+"rows"+blank+tostr(m.rows()));
  str_list.Add (prefix+indent+"cols"+blank+tostr(m.cols()));
  for (int r(0); r<m.rows(); ++r)
  {
    std::stringstream ss;
    ss<<prefix<<indent<<"row"<<blank<<tostr(r);
    for (int c(0); c<m.cols(); ++c)
      ss<<blank<<tostr(m(r,c));
    str_list.Add (ss.str());
  }
  str_list.Add (prefix+std::string("end"));
  #undef tostr
}
//-------------------------------------------------------------------------------------------

template <>
void LoadFromStringList (Matrix &m, const TStringListEx &str_list)
{
  bool first_data (true);
  int rows(1), cols(1);
  std::list<std::string> token;
  while(1)
  {
    if (str_list.IsEndOfList())  break;
    token = str_list.Tokenize(true);
    std::list<std::string>::iterator itr = token.begin();
    if (token.empty() || IsComment(*itr))
    {}
    else if (*itr == "rows")
    {
      ++itr;
      rows=boost::lexical_cast<int>(*itr);
    }
    else if (*itr == "cols")
    {
      ++itr;
      cols=boost::lexical_cast<int>(*itr);
    }
    else if (*itr == "row")
    {
      if (first_data)
      {
        if (rows<=0 || cols<=0)
          {std::cerr<<"invalid matrix size rows="<<rows<<", cols="<<cols<<std::endl;return;}
        m.resize (rows,cols,0.0);
        first_data = false;
      }
      const std::string blank(" ");
      int r(0);
      ++itr;
      r = boost::lexical_cast<int>(*itr);
      if (r>=rows)
        std::cerr<<"in load matrix, r="<<r<<" is greater than rows="<<rows<<std::endl;
      else
      {
        ++itr;
        for (int c(0); itr!=token.end()&&c<cols; ++itr,++c)
          m(r,c) = boost::lexical_cast<double>(*itr);
      }
    }
    else if (*itr == "end")
      break;
    else
    {
      LERROR("invalid text format");
    }
    str_list.Increment();
  }
}
//-------------------------------------------------------------------------------------------

template <>
void SaveToStringList (const ColumnVector &x, TStringListEx &str_list, const std::string &prefix)
{
  /*
      dim1 3
      val 0 1.2
      val 1 3.3
      val 2 4.5
    end
  */
  #define tostr(_s) ConvertToStr(_s)
  const std::string indent("  ");
  const std::string blank(" ");
  str_list.Add (prefix+indent+"dim1"+blank+tostr(x.dim1()));
  for (int c(0); c<x.dim1(); ++c)
  {
    std::stringstream ss;
    ss<<prefix<<indent<<"val"<<blank<<tostr(c)<<blank<<tostr(x(c));
    str_list.Add (ss.str());
  }
  str_list.Add (prefix+std::string("end"));
  #undef tostr
}
//-------------------------------------------------------------------------------------------

template <>
void LoadFromStringList (ColumnVector &x, const TStringListEx &str_list)
{
  bool first_data (true);
  int dim1(1);
  std::list<std::string> token;
  while(1)
  {
    if (str_list.IsEndOfList())  break;
    token = str_list.Tokenize(true);
    std::list<std::string>::iterator itr = token.begin();
    if (token.empty() || IsComment(*itr))
    {}
    else if (*itr == "dim1")
    {
      ++itr;
      dim1=boost::lexical_cast<int>(*itr);
    }
    else if (*itr == "val")
    {
      if (first_data)
      {
        if (dim1<=0)
          {std::cerr<<"invalid vector size dim1="<<dim1<<std::endl;return;}
        x.resize (dim1,0.0);
        first_data = false;
      }
      int c(0);
      ++itr;
      c = boost::lexical_cast<int>(*itr);
      ++itr;
      if (c>=dim1)
        std::cerr<<"in load vector, c="<<c<<" is greater than dim1="<<dim1<<std::endl;
      else
        x(c) = boost::lexical_cast<double>(*itr);
    }
    else if (*itr == "end")
      break;
    else
    {
      LERROR("invalid text format");
    }
    str_list.Increment();
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
