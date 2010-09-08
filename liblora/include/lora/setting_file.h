//-------------------------------------------------------------------------------------------
/*! \file    setting_file.h
    \brief   liblora - setting file io class (DEPRECATED!)
    \author  Akihiko Yamaguchi
    \date    2008
    \date    Oct. 12, 2009 : move getIndent to private and rename to get_indent

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
#ifndef loco_rabbits_setting_file_h
#define loco_rabbits_setting_file_h
//-------------------------------------------------------------------------------------------
#include <string>
#include <boost/lexical_cast.hpp>
#include <lora/string.h>
//-------------------------------------------------------------------------------------------

namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

#define saveToSettingFile(settingfile,group,var) \
  settingfile.addString(group,#var,var)
#define saveToSettingFileI(settingfile,group,var) \
  settingfile.addString(group,#var,(int)var)
// #define saveToSettingFileSL(settingfile,group,var)
//   var.SaveToStringList(settingfile.addStringList(group,#var),settingfile.getIndent())
#define saveToSettingFileSL(settingfile,group,var)  \
  var.SaveToStringList(settingfile.addStringList(group,#var))
  //!< var must have SaveToStringList member function
// #define saveToSettingFileSL2(settingfile,group,var)
//   SaveToStringList(var,settingfile.addStringList(group,#var),settingfile.getIndent())
#define saveToSettingFileSL2(settingfile,group,var)  \
  SaveToStringList(var,settingfile.addStringList(group,#var))
  //!< SaveToStringList must be defined for the type of var
#define loadFromSettingFile(settingfile,group,var)  \
  if(settingfile.exists(group,#var)) settingfile.getValue(group,#var,var);
#define loadFromSettingFileI(settingfile,group,var)  \
  if(settingfile.exists(group,#var)) settingfile.getValueEnum(group,#var,var);
#define loadFromSettingFileSL(settingfile,group,var)  \
  if(settingfile.exists(group,#var)) var.LoadFromStringList(settingfile.getStringList(group,#var));
  //!< var must have LoadFromStringList member function
#define loadFromSettingFileSL2(settingfile,group,var)  \
  if(settingfile.exists(group,#var)) LoadFromStringList(var,settingfile.getStringList(group,#var));
  //!< LoadFromStringList must be defined for the type of var

//-------------------------------------------------------------------------------------------

struct _type_setting_file_data;
class TStringListEx;
//-------------------------------------------------------------------------------------------
template <typename T>
void SaveToStringList (const T &m, TStringListEx &str_list, const std::string &prefix=std::string(""));
template <typename T>
void LoadFromStringList (T &m, const TStringListEx &str_list);
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TSettingFile
//===========================================================================================
{
public:
  TSettingFile (void);
  TSettingFile (const TSettingFile &rhs);
  ~TSettingFile (void);
  void clear (void);
  bool exists (const std::string &groupname, const std::string &valname) /*const*/;
  void addString (const std::string &groupname, const std::string &valname, const std::string &value)
    {
      add_string(groupname,valname,value);
    };
  void addString (const std::string &groupname, const std::string &valname, const bool &value)
    {
      add_string(groupname,valname,BoolToStr(value));
    };
  template <typename T>
  void addString (const std::string &groupname, const std::string &valname, const T &value)
    {
      add_string(groupname,valname,boost::lexical_cast<std::string>(value));
    };
  const std::string& getString (const std::string &groupname, const std::string &valname) /*const*/;
  void getValue (const std::string &groupname, const std::string &valname, bool &value)
    {
      value = StrToBool (getString(groupname,valname));
    };
  template <typename T>
  void getValue (const std::string &groupname, const std::string &valname, T &value)
    {
      try {
        value = boost::lexical_cast<T> (getString(groupname,valname));
      }catch (...){
        {LERROR("failed to assign \"" << getString(groupname,valname) << "\" to " << groupname << "." << valname);}
      }
    };
  template <typename T>
  void getValueEnum (const std::string &groupname, const std::string &valname, T &value)
    {
      try {
        value = static_cast<T>(boost::lexical_cast<int>(getString(groupname,valname)));
      }catch (...){
        {LERROR("failed to assign \"" << getString(groupname,valname) << "\" to " << groupname << "." << valname);}
      }
    };
  TStringListEx& addStringList (const std::string &groupname, const std::string &valname);
  const TStringListEx& getStringList (const std::string &groupname, const std::string &valname) /*const*/;
  void SaveToStringList (TStringListEx &str_list, const std::string &prefix=std::string("")) const;
private:
  void load_group (const std::string &groupname, const TStringListEx &str_list);
public:
  void LoadFromStringList (const TStringListEx &str_list);
  void PrintToStream (std::ostream &os, const std::string &prefix=std::string("")) const;
  bool SaveToFile (const std::string &filename) const;
  bool LoadFromFile (const std::string &filename);

private:
  _type_setting_file_data *data;
  void add_string (const std::string &groupname, const std::string &valname, const std::string &strdata);
  std::string get_indent (const std::string &prefix=std::string("")) const
    {
      const std::string indent("  ");
      return prefix+indent+indent;
    };
};
//-------------------------------------------------------------------------------------------

}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_setting_file_h
//-------------------------------------------------------------------------------------------

