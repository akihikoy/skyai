//-------------------------------------------------------------------------------------------
/*! \file    basic_syntax.h
    \brief   liblora - basic syntax for boost::spirit (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.03, 2012

    Copyright (C) 2012  Akihiko Yamaguchi

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
// #ifndef loco_rabbits_basic_syntax_h
// #define loco_rabbits_basic_syntax_h
//-------------------------------------------------------------------------------------------

lcomment
  = str_p("//")>>*(anychar_p - eol_p)
      >> *blank_p ;

blank_eol_p
  = blank_p | end_of_line | (lcomment>>end_of_line);

op_semicolon
  = *blank_p >> ch_p(';') >> *blank_p ;
op_comma
  = *blank_p >> ch_p(',') >> *blank_p ;
op_dot
  = *blank_p >> ch_p('.') >> *blank_p ;
op_eq
  = *blank_p >> ch_p('=') >> *blank_p ;
op_at
  = *blank_p >> ch_p('@') >> *blank_p ;
op_brace_l
  = *blank_p >> ch_p('{') >> *blank_p ;
op_brace_r
  = *blank_p >> ch_p('}') >> *blank_p ;
op_parenthesis_l
  = *blank_p >> ch_p('(') >> *blank_p ;
op_parenthesis_r
  = *blank_p >> ch_p(')') >> *blank_p ;
op_bracket_l
  = *blank_p >> ch_p('[') >> *blank_p ;
op_bracket_r
  = *blank_p >> ch_p(']') >> *blank_p ;

op_plus
  = *blank_p >> ch_p('+') >> *blank_p ;
op_minus
  = *blank_p >> ch_p('-') >> *blank_p ;
op_star
  = *blank_p >> ch_p('*') >> *blank_p ;
op_slash
  = *blank_p >> ch_p('/') >> *blank_p ;
op_percent
  = *blank_p >> ch_p('%') >> *blank_p ;
op_lt
  = *blank_p >> ch_p('<') >> *blank_p ;
op_gt
  = *blank_p >> ch_p('>') >> *blank_p ;
op_lteq
  = *blank_p >> str_p("<=") >> *blank_p ;
op_gteq
  = *blank_p >> str_p(">=") >> *blank_p ;
op_deq
  = *blank_p >> str_p("==") >> *blank_p ;
op_exeq
  = *blank_p >> str_p("!=") >> *blank_p ;
op_amp
  = *blank_p >> ch_p('&') >> *blank_p ;
op_bar
  = *blank_p >> ch_p('|') >> *blank_p ;
op_excl
  = *blank_p >> ch_p('!') >> *blank_p ;
op_damp
  = *blank_p >> str_p("&&") >> *blank_p ;
op_dbar
  = *blank_p >> str_p("||") >> *blank_p ;
op_cat
  = *blank_p >> str_p("##") >> *blank_p ;

//-------------------------------------------------------------------------------------------
// #endif // loco_rabbits_basic_syntax_h
//-------------------------------------------------------------------------------------------
