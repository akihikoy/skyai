//-------------------------------------------------------------------------------------------
/*! \file    variable_space_fwd.h
    \brief   liblora - variable-space : generic manipulators of variables  (header for forward declarations)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.12, 2010-

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
#ifndef loco_rabbits_variable_space_fwd_h
#define loco_rabbits_variable_space_fwd_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <lora/stl_fwd.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
namespace var_space
{
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// type definitions
//-------------------------------------------------------------------------------------------

// primitive types:
typedef long                 pt_int       ;
typedef long double          pt_real      ;
typedef bool                 pt_bool      ;
typedef std::string          pt_string    ;

typedef pt_string            TIdentifier;

class TVariable;
typedef LIST_FWD(TVariable) TVariableList;
typedef MAP_FWD(TIdentifier, TVariable) TVariableMap;
class TForwardIterator;
class TConstForwardIterator;

//!\brief dummy type to use a variable as a variable space
struct TVariableSpace {void *dummy;};
inline TVariableSpace VariableSpace()  {TVariableSpace vs; return vs;}

class TAnyPrimitive;

class TLiteral;
class TLiteralTable;

#ifndef VAR_SPACE_INDENT_STEP
#define VAR_SPACE_INDENT_STEP  "  "
#endif

#ifndef VAR_SPACE_ERR_EXIT
#define VAR_SPACE_ERR_EXIT(x_err_message)  do{LERROR(x_err_message); lexit(df);}while(0)
#endif

#define VAR_SPACE_KEYWORD_THIS "this"

//-------------------------------------------------------------------------------------------
}  // end of namespace var_space
//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_space_fwd_h
//-------------------------------------------------------------------------------------------
