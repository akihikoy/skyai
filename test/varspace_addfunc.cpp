//-------------------------------------------------------------------------------------------
/*! \file    varspace_addfunc.cpp
    \brief   Test program to add functions to variable space
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.05, 2013

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
#include <lora/variable_binexec.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
#include <vector>
#include <list>
#include <map>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

struct TCell
{
  int x;
  double y;
  vector<double> z;
};
namespace loco_rabbits{ namespace var_space{
  void Register (TCell &test, TVariableMap &mmap)
  {
    #define ADD(x_member)  \
      AddToVarMap(mmap, #x_member, test.x_member)
    ADD( x );
    ADD( y );
    ADD( z );
    #undef ADD
  }
}}

struct TTest
{
  int x;
  double y;
  vector<double> z;
  list<int> a;
  string str;
  bool b;
  map<string,long double> m;
  list<bool> blist;
  vector<list<double> > _B01;
  TCell cell;
  vector<TCell> cells;
};

namespace loco_rabbits{ namespace var_space{
  void Register (TTest &test, TVariableMap &mmap)
  {
    #define ADD(x_member)  \
      AddToVarMap(mmap, #x_member, test.x_member)
    ADD( x );
    ADD( y );
    ADD( z );
    ADD( a );
    ADD( str );
    ADD( b );
    ADD( m );
    ADD( blist );
    ADD( _B01 );
    ADD( cell );
    ADD( cells );
    #undef ADD
  }
}}

void function_inline_include (var_space::TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=3)
    {VAR_SPACE_ERR_EXIT("syntax of " "inline_include" " should be void(var,str)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr); ++itr;
  TVariable &arg2(*itr); ++itr;

  TLiteralTable literal_table;
  var_space::LoadFromFile(arg2.PrimitiveGetAs<pt_string>(),arg1,literal_table);
}

void function_pi (var_space::TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=1)
    {VAR_SPACE_ERR_EXIT("syntax of " "pi" " should be real()");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr);

  res.PrimitiveSetBy<pt_real>(REAL_PI);
}

void function_increment (var_space::TBinExecutor &context, var_space::TVariableList &argv)
{
  using namespace loco_rabbits::var_space;
  if (argv.size()!=2)
    {VAR_SPACE_ERR_EXIT("syntax of " "increment" " should be list(list)");}
  TVariableList::iterator itr(argv.begin());
  TVariable &res(*itr); ++itr;
  TVariable &arg1(*itr);

  TForwardIterator a1itr,a1last;
  arg1.GetBegin(a1itr); arg1.GetEnd(a1last);
  for(; a1itr!=a1last; ++a1itr)
    res.Push().PrimitiveSetBy<pt_real>( a1itr->PrimitiveGetAs<pt_real>()+1.0 );
}

int main(int argc, char**argv)
{
  using namespace var_space;
  TBuiltinFunctions additional_funcs;
  additional_funcs.Add("inline_include", TBuiltinFunctions::rtVoid, &function_inline_include);
  additional_funcs.Add("pi", TBuiltinFunctions::rtReal, &function_pi);
  additional_funcs.Add("increment", TBuiltinFunctions::rtList, &function_increment);

  string filename= (argc>1)?argv[1]:"(file does not exist)";
  cout<<"loading "<<filename<<"..."<<endl;

  TTest test;
  TVariable var(test);

  TLiteralTable literal_table;

  if (LoadFromFile (filename,var,literal_table,&additional_funcs))
  {
    var.WriteToStream(cout,true);
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
