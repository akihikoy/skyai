//-------------------------------------------------------------------------------------------
/*! \file    string_list.cpp
    \brief   liblora - string-list class  (DEPRECATED)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

    Copyright (C) 2008, 2010  Akihiko Yamaguchi

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
#include <lora/string_list.h>
//-------------------------------------------------------------------------------------------
#include <fstream>
#include <boost/tokenizer.hpp>
#include <lora/stl_ext.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------


inline void basic_tokenize (std::list<std::string> &res, const std::string &str, bool skip_blank)
{
  const boost::char_separator<char>  sep(" \t\n");
  // std::list<std::string> res;
  if (str == "")  return;
  boost::tokenizer< boost::char_separator<char> >  tokens (str, sep);
  boost::tokenizer< boost::char_separator<char> >::iterator itr= tokens.begin();
  for (; itr!=tokens.end(); ++itr)
    if (!skip_blank || *itr != "")
      res.push_back (*itr);
}
//-------------------------------------------------------------------------------------------

void TStringListEx::Tokenize (std::list<t_string> &token, bool skip_blank) const  //!< tokenize *position
{
  basic_tokenize (token, *position, skip_blank);
}
void TStringListEx::Tokenize (std::list<t_string> &token, int index, bool skip_blank) const  //!< tokenize at index
{
  basic_tokenize (token, *list_itr_at(data,index), skip_blank);
}
//-------------------------------------------------------------------------------------------

void TStringListEx::make_error( const char * const str, bool ex ) const
{
   if(str) {LERROR("error in TStringListEx class: " << str);}
   else    {LERROR("error in TStringListEx class");}
   if(ex)
     lexit(df);
}
//-------------------------------------------------------------------------------------------

void TStringListEx::make_error( const t_string &str, bool ex ) const
{
   make_error( str.c_str(), ex );
}
//-------------------------------------------------------------------------------------------

TStringListEx::t_string& TStringListEx::operator[]( int index )
{
  return *list_itr_at(data,index);
}
//-------------------------------------------------------------------------------------------

void TStringListEx::SetIterator (int index) const
{
  position = list_itr_at(data,index);
}
//-------------------------------------------------------------------------------------------

//! delete a line specified by index. \note position is set to the next line
void TStringListEx::Delete (int index)
{
  position=data.erase( list_itr_at(data,index) );
}
//-------------------------------------------------------------------------------------------

void TStringListEx::Insert( int index, const t_string &str )
{
  data.insert(list_itr_at(data,index), str);
}
//-------------------------------------------------------------------------------------------

bool TStringListEx::LoadFromFile( const t_string &filename )
{
   try
   {
     std::ifstream file( filename.c_str() );
     if( !file )
       make_error( std::string("cannot open the file: ")+filename, false );

     data.clear();
     std::string str;
     while( getline( file, str, '\n' ) )
       data.push_back( str );
     SetIterator(0);
     return true;
   }
   catch(...)
   {
     make_error( std::string("faital in loading ")+filename, false );
     return false;
   }
}
//-------------------------------------------------------------------------------------------

bool TStringListEx::SaveToFile( const t_string &filename ) const
{
   try
   {
     std::ofstream file( filename.c_str() );
     if( !file )
       make_error( std::string("cannot open the file: ")+filename, false );

     for( std::list<t_string>::const_iterator itr=data.begin(); itr!=data.end(); ++itr )
       file << *itr << std::endl;
     return true;
   }
   catch(...)
   {
     make_error( std::string("faital in saving ")+filename, false );
     return false;
   }
}
//-------------------------------------------------------------------------------------------

void TStringListEx::PrintToStream (std::ostream &os, const std::string &prefix) const
{
   for( std::list<t_string>::const_iterator itr=data.begin(); itr!=data.end(); ++itr )
     os << prefix << *itr << std::endl;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
