//-------------------------------------------------------------------------------------------
/*! \file    variable_parser.cpp
    \brief   liblora - parser for variable-space  (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-

    Copyright (C) 2010, 2012  Akihiko Yamaguchi

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
#include <lora/variable_parser_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
using namespace std;
using namespace boost::spirit::classic;


bool ParseFile (const std::string &file_name, TBinaryStack &bin_stack, const TParserCallbacks &callbacks)
{
  using namespace boost::spirit::classic;
  typedef file_iterator<char> TIterator;
  TIterator  first(file_name);
  if (!first)
  {
    LERROR("failed to open file: "<<file_name);
    return false;
  }

  TIterator last= first.make_end();

  int linenum(1);
  std::string file_name2(file_name);
  bool error(false);

  TCodeParser<TIterator> parser;
  parser.SetBinStack(&bin_stack);
  parser.SetFileName(&file_name2);
  parser.SetLineNum(&linenum);
  parser.SetError(&error);
  parser.SetCallbacks(callbacks);

  LORA_MESSAGE_FORMAT_FUNCTION tmp_msg_format= message_system::GetFormat<LORA_MESSAGE_FORMAT_FUNCTION>();
  message_system::SetFormat(LORA_MESSAGE_FORMAT_FUNCTION(boost::bind(&TCodeParser<TIterator>::LoraError,&parser,_1,_2,_3,_4,_5)) );

  parse_info<TIterator> info= parse(first, last, parser);
  message_system::SetFormat(tmp_msg_format);


  LMESSAGE("loaded "<<linenum<<" lines,");
  LMESSAGE("which is "<<(info.stop==last ? "the last" : "not the last"));

  return !parser.Error();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

