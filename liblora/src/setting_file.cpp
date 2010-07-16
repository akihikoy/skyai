//-------------------------------------------------------------------------------------------
/*! \file    setting_file.cpp
    \brief   liblora - setting file io class (DEPRECATED!)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008
    \date    Oct. 12, 2009 : change SaveToStringList

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
#include <lora/setting_file.h>
//-------------------------------------------------------------------------------------------
#include <lora/string_list.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

struct type_cell
{
  bool is_str;
  std::string str;
  TStringListEx strlist;
  type_cell (void) : is_str(true) {};
};

typedef std::map <std::string, type_cell> type_group;

struct _type_setting_file_data
{
  std::map <std::string, type_group> table;
};
//-------------------------------------------------------------------------------------------

TSettingFile::TSettingFile (void)
  : data(NULL)
{
  data= new _type_setting_file_data;
}
//-------------------------------------------------------------------------------------------

TSettingFile::TSettingFile (const TSettingFile &rhs)
{
  data= new _type_setting_file_data;
  *data= *rhs.data;
}
//-------------------------------------------------------------------------------------------

TSettingFile::~TSettingFile (void)
{
  delete data;
  data= NULL;
}
//-------------------------------------------------------------------------------------------

void TSettingFile::add_string (const std::string &groupname, const std::string &valname, const std::string &strdata)
{
  data->table[groupname][valname].is_str = true;
  data->table[groupname][valname].str = strdata;
}
//-------------------------------------------------------------------------------------------

void TSettingFile::clear (void)
{
  data->table.clear();
}
//-------------------------------------------------------------------------------------------

bool TSettingFile::exists (const std::string &groupname, const std::string &valname) /*const*/
{
  if (data->table.find(groupname)==data->table.end())  return false;
  if (data->table[groupname].find(valname)==data->table[groupname].end())  return false;
  return true;
}
//-------------------------------------------------------------------------------------------

const std::string& TSettingFile::getString (const std::string &groupname, const std::string &valname) /*const*/
{
  if (!data->table[groupname][valname].is_str)
    {LERROR(groupname << "." << valname << " is accessed as string despite of its type is TStringListEx");}
  return data->table[groupname][valname].str;
}
//-------------------------------------------------------------------------------------------

TStringListEx& TSettingFile::addStringList (const std::string &groupname, const std::string &valname)
{
  data->table[groupname][valname].is_str = false;
  return data->table[groupname][valname].strlist;
}
//-------------------------------------------------------------------------------------------

const TStringListEx& TSettingFile::getStringList (const std::string &groupname, const std::string &valname) /*const*/
{
  if (data->table[groupname][valname].is_str)
    {LERROR(groupname << "." << valname << " is accessed as stringlist despite of its type is std::string");}
  return data->table[groupname][valname].strlist;
}
//-------------------------------------------------------------------------------------------

void TSettingFile::SaveToStringList (TStringListEx &str_list, const std::string &prefix) const
{
  /*! sample:
      group goup1
        str val1 10
        str val2 hoehoe
        strlist val3
          x 10
          y 12
        end
        :strlistend
      end
      group goup2
        ...
      end
    end
  */
  const std::string indent("  ");
  const std::string blank(" ");
  for (std::map <std::string, type_group>::const_iterator titr(data->table.begin()); titr!=data->table.end(); ++titr)
  {
    str_list.Add (prefix+indent+std::string("group")+blank+titr->first);  // groupname
    for (type_group::const_iterator cellitr(titr->second.begin()); cellitr!=titr->second.end(); ++cellitr)
    {
      if (cellitr->second.is_str)
        str_list.Add (prefix+indent+indent+std::string("str")+blank+cellitr->first+blank+cellitr->second.str);
      else
      {
        str_list.Add (prefix+indent+indent+std::string("strlist")+blank+cellitr->first);
        for (cellitr->second.strlist.SetIterator(); !cellitr->second.strlist.IsEndOfList(); cellitr->second.strlist.Increment())
          str_list.Add (prefix+indent+indent+cellitr->second.strlist.Current());
          // str_list.Add (cellitr->second.strlist.Current());
        str_list.Add (prefix+indent+indent+std::string(":strlistend"));
      }
    }
    str_list.Add (prefix+indent+std::string("end"));  // group end
  }
  str_list.Add (prefix+std::string("end"));  // setting end
}
//-------------------------------------------------------------------------------------------

/*private*/void TSettingFile::load_group (const std::string &groupname, const TStringListEx &str_list)
{
  const std::string blank(" ");
  std::list<std::string> token;
  while(1)
  {
    if (str_list.IsEndOfList())  break;
    token = str_list.Tokenize(true);
    std::list<std::string>::iterator itr = token.begin();
    if( token.empty() || IsComment(*itr) )
    {}
    else if( *itr == "str" )
    {
      ++itr;
      const std::string valname (*itr); ++itr;
      std::string line;
      if (itr!=token.end()) line= *itr;
      else line= "";
      for (++itr; itr!=token.end(); ++itr)
        line = line + blank + (*itr);
      addString (groupname,valname,line);
    }
    else if( *itr == "strlist" )
    {
      ++itr;
      TStringListEx &substrlist (addStringList(groupname,*itr));
      str_list.Increment();
      substrlist.Clear();
      while(1)
      {
        if (str_list.IsEndOfList())  break;
        std::string current (str_list.Current());
        TrimBoth (current);
        if (current==":strlistend")  break;
        substrlist.Add (str_list.Current());
        str_list.Increment();
      }
      substrlist.SetIterator();
    }
    else if( *itr == "end" )
      break;
    else
    {
      {LERROR("invalid text format");}
    }
    str_list.Increment();
  }
}
//-------------------------------------------------------------------------------------------

void TSettingFile::LoadFromStringList (const TStringListEx &str_list)
{
  clear();
  std::list<std::string> token;
  while(1)
  {
    if (str_list.IsEndOfList())  break;
    token = str_list.Tokenize(true);
    std::list<std::string>::iterator itr = token.begin();
    if( token.empty() || IsComment(*itr) )
    {}
    else if( *itr == "group" )
    {
      ++itr;
      const std::string groupname (*itr);
      str_list.Increment();
      load_group (groupname, str_list);
    }
    else if( *itr == "end" )
      break;
    else
    {
      {LERROR("invalid text format");}
    }
    str_list.Increment();
  }
}
//-------------------------------------------------------------------------------------------

void TSettingFile::PrintToStream (std::ostream &os, const std::string &prefix) const
{
  TStringListEx str_list;
  SaveToStringList (str_list, prefix);
  str_list.PrintToStream (os);
}
//-------------------------------------------------------------------------------------------

bool TSettingFile::SaveToFile (const std::string &filename) const
{
  try
  {
    TStringListEx str_list;
    SaveToStringList (str_list, "");
    str_list.SaveToFile (filename);
    return true;
  }
  catch(...)
  {
    {LERROR("fatal in SaveToFile: cannot save to  " << filename);}
    return false;
  }
}
//-------------------------------------------------------------------------------------------

bool TSettingFile::LoadFromFile (const std::string &filename)
{
  try
  {
    TStringListEx str_list;
    str_list.LoadFromFile (filename);
    LoadFromStringList (str_list);
    return true;
  }
  catch(...)
  {
    {LERROR("fatal in LoadFromFile: cannot load from  " << filename);}
    return false;
  }
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
