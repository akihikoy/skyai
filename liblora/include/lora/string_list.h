//-------------------------------------------------------------------------------------------
/*! \file   string_list.h
    \brief  liblora - string-list class  (DEPRECATED)
    \author Akihiko Yamaguchi
    \date   2008-

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
#ifndef loco_rabbits_string_list_h
#define loco_rabbits_string_list_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <list>
#include <string>
#include <iostream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{



//!\todo FIXME: use loco_rabbits::TTokenizer instead of Tokenize! (better solution)
//!\todo FIXME: remove the iterator from the member variables (unsafe in multi-thread)

class TStringListEx
  //! string list class
{
public:
  typedef std::string t_string;
private:
  std::list<t_string> data;
  mutable std::list<t_string>::const_iterator position;

  inline void make_error( const char * const str, bool ex ) const;
  inline void make_error( const t_string &str, bool ex ) const;

protected:
public:
  TStringListEx( void ) : data(0), position(data.begin()) {};

  t_string& operator[]( int index );
  const t_string& operator[]( int index ) const { return operator[](index); };

  int GetIndex (void) const  //!< return index of position
    {
      int i(0);
      for (std::list<t_string>::const_iterator itr(data.begin()); itr!=data.end(); ++itr,++i)
        if (itr==position)  break;
      return i;
    };
  void Clear (void)  { data.clear(); position=data.begin(); };
  t_string& Current( void )  { return const_cast<t_string&>(*position); };
  const t_string& Current( void ) const { return *position; };
  void SetIterator (int index=0) const;
  void Increment( void ) const { ++position; };
  void Decrement( void ) const { --position; };
  bool IsEndOfList( void ) const { return (position==data.end()); };

  int  Count( void ) const { return data.size(); };
  void Delete (int index);  //! delete a line specified by index. \note position is set to the next line
  void Delete( void )  { Delete(GetIndex()); };
  void Add( const t_string &str )  { data.push_back(str); };
  void Insert( int index, const t_string &str );
  void Insert( const t_string &str )  { Insert(GetIndex(),str); };

  void Tokenize (std::list<t_string> &token, bool skip_blank) const;  //!< tokenize *position
  void Tokenize (std::list<t_string> &token, int index, bool skip_blank) const;  //!< tokenize at index

  std::list<t_string> Tokenize( bool skip_blank ) const  //!< tokenize *position
    {
      std::list<std::string> res;
      Tokenize (res, skip_blank);
      return res;
    };
  std::list<t_string> Tokenize( int index, bool skip_blank ) const  //!< tokenize at index
    {
      std::list<std::string> res;
      Tokenize (res, index, skip_blank);
      return res;
    };

  bool LoadFromFile( const t_string &filename );
  bool SaveToFile( const t_string &filename ) const;

  void PrintToStream (std::ostream &os, const std::string &prefix="") const;
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//  supplementary functions
//===========================================================================================

template <typename T>
void SaveToStringList (const T &m, TStringListEx &str_list, const std::string &prefix=std::string(""));
template <typename T>
void LoadFromStringList (T &m, const TStringListEx &str_list);
//-------------------------------------------------------------------------------------------

/*! \brief general save function
    \param T  T::SaveToStringList(str_list,prefix) must be defined */
template <typename T>
inline bool SaveToFileI (const T &x, const std::string &filename)
{
  try
  {
    TStringListEx str_list;
    x.SaveToStringList (str_list, "");
    str_list.SaveToFile (filename);
    return true;
  }
  catch(...)
  {
    LERROR("fatal in SaveToFile: cannot save to  " << filename);
    return false;
  }
};
//-------------------------------------------------------------------------------------------

/*! \brief general load function
    \param T  T::LoadFromStringList(str_list) must be defined as a member function of T */
template <typename T>
inline bool LoadFromFileI (T &x, const std::string &filename)
{
  try
  {
    TStringListEx str_list;
    str_list.LoadFromFile (filename);
    x.LoadFromStringList (str_list);
    return true;
  }
  catch(...)
  {
    LERROR("fatal in LoadFromFile: cannot load from  " << filename);
    return false;
  }
};
//-------------------------------------------------------------------------------------------

/*! \brief general save function
    \param T  SaveToStringList(T,str_list,prefix) must be defined for T */
template <typename T>
inline bool SaveToFile (const T &x, const std::string &filename)
{
  try
  {
    TStringListEx str_list;
    SaveToStringList (x, str_list, "");
    str_list.SaveToFile (filename);
    return true;
  }
  catch(...)
  {
    LERROR("fatal in SaveToFile: cannot save to  " << filename);
    return false;
  }
};
//-------------------------------------------------------------------------------------------

/*! \brief general load function
    \param T  LoadFromStringList(T,str_list) must be defined for T */
template <typename T>
inline bool LoadFromFile (T &x, const std::string &filename)
{
  try
  {
    TStringListEx str_list;
    str_list.LoadFromFile (filename);
    LoadFromStringList (x, str_list);
    return true;
  }
  catch(...)
  {
    LERROR("fatal in LoadFromFile: cannot load from  " << filename);
    return false;
  }
};
//-------------------------------------------------------------------------------------------


}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_string_list_h
//-------------------------------------------------------------------------------------------


