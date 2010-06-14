//-------------------------------------------------------------------------------------------
/*! \file    string_list_ext_impl.h
    \brief   liblora - TStringListEx extensions  (DEPRECATED)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.29, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

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
#ifndef loco_rabbits_string_list_ext_impl_h
#define loco_rabbits_string_list_ext_impl_h
//-------------------------------------------------------------------------------------------
#include <lora/string_list_ext.h>
#include <lora/string_list.h>
#include <lora/string.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

// implementations:
template <typename t_key, typename t_value>
void SaveToStringListMap (const std::map<t_key,t_value> &m, TStringListEx &str_list, const std::string &prefix)
{
  /*
      map key0 value0
      map key1 value1
    end
  */
  const std::string indent("  ");
  const std::string blank(" ");
  for (typename std::map<t_key,t_value>::const_iterator itr(m.begin()); itr!=m.end(); ++itr)
    str_list.Add (prefix+indent+"map"+blank+ConvertToStr(itr->first)+blank+ConvertToStr(itr->second));
  str_list.Add (prefix+std::string("end"));
}
//-------------------------------------------------------------------------------------------

template <typename t_key, typename t_value>
void LoadFromStringListMap (std::map<t_key,t_value> &m, const TStringListEx &str_list)
{
  m.clear();
  TTokenizer tokenizer;
  std::string line, str;
  for (; !str_list.IsEndOfList(); str_list.Increment())
  {
    line= str_list.Current();
    trim_both(line);
    tokenizer.Init (line);
    str= tokenizer.ReadIdentifier();
    if (line=="" || IsComment(line))
    {}
    else if (str == "map")
    {
      if (tokenizer.ReadSeparators()=="")  {LERROR("invalid format: "<<line); continue;}
      t_key key= ConvertFromStr<t_key>(tokenizer.ReadValue<t_key>());
      if (tokenizer.ReadSeparators()=="")  {LERROR("invalid format: "<<line); continue;}
      t_value value= ConvertFromStr<t_value>(tokenizer.ReadValue<t_value>());
      if (tokenizer.ReadAll()!="")  {LERROR("invalid format: "<<line);}
      m[key]= value;
    }
    else if (str == "end")
    {
      if (tokenizer.ReadAll()!="")  {LERROR("invalid format: "<<line);}
      break;
    }
    else
    {
      LERROR("invalid text format");
    }
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_string_list_ext_impl_h
//-------------------------------------------------------------------------------------------
