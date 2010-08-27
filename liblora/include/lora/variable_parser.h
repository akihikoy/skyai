//-------------------------------------------------------------------------------------------
/*! \file    variable_parser.h
    \brief   liblora - parser for variable-space  (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    May.17, 2010-

    Copyright (C) 2010  Akihiko Yamaguchi

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
#ifndef loco_rabbits_variable_parser_h
#define loco_rabbits_variable_parser_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_space_fwd.h>
#include <lora/variable_any.h>
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <string>
#include <list>
#include <map>
#include <set>
#include <sstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
struct TLiteral
//===========================================================================================
{
  enum TLiteralType {ltIdentifier=0, ltPrimitive, ltList};
  TLiteralType    LType;
  TAnyPrimitive   LPrimitive;
  std::list<TAnyPrimitive>  LList;
};
//-------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &lhs, const TLiteral &rhs);
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TLiteralTable
//===========================================================================================
{
public:

  enum TAddResult {arFailed=0, arInserted, arOverwritten};

  typedef std::map<TIdentifier,TLiteral> TTable;

  TLiteralTable(void);

  const TLiteral* Find(const TIdentifier &id) const;

  TAddResult AddIdentifier(const TIdentifier &id, const TIdentifier &value);

  template <typename t_primitive>
  TAddResult AddLiteral(const TIdentifier &id, const t_primitive &value)
    {
      TLiteral l;
      l.LType= TLiteral::ltPrimitive;
      l.LPrimitive= TAnyPrimitive(value);
      return add_to_table(id,l);
    }
  TAddResult AddLiteral(const TIdentifier &id, const std::list<TAnyPrimitive>  &value);

  TAddResult AddLiteral(const TIdentifier &id, const TLiteral &l)
    {
      return add_to_table(id,l);
    }

private:

  TTable table_;
  std::set<pt_string>  keywords_;

  TAddResult add_to_table(const TIdentifier &id, const TLiteral &value);

};
//-------------------------------------------------------------------------------------------

struct TEvaluateLiteralConfig
{
  bool AllowId;
  bool ExitByError;
  TEvaluateLiteralConfig() : AllowId(false), ExitByError(true) {}
};
TLiteral EvaluateLiteral (const TLiteral &src, const TLiteralTable *literal_table, const TEvaluateLiteralConfig &config, bool &error);
//-------------------------------------------------------------------------------------------


//===========================================================================================

enum TParseMode {pmNormal=0, pmPhantom};

struct TParserInfoIn
{
  TVariable            &Var;
  TParseMode           ParseMode;
  std::string          FileName;
  int                  StartLineNum;
  const TLiteralTable  *LiteralTable;
  std::stringstream *EquivalentCode;  //! stored if ParseMode is pmPhantom
  TParserInfoIn(TVariable &v) : Var(v), ParseMode(pmNormal), StartLineNum(1), LiteralTable(NULL), EquivalentCode(NULL)  {}
};
struct TParserInfoOut
{
  bool  IsLast;
  int   LastLineNum;
  TParserInfoOut (void) : IsLast(false), LastLineNum(-1)  {}
};
//-------------------------------------------------------------------------------------------

//===========================================================================================

bool ParseFile (TParserInfoIn &in, TParserInfoOut *out);

//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_parser_h
//-------------------------------------------------------------------------------------------
