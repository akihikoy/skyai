//-------------------------------------------------------------------------------------------
/*! \file    variable_binexec.cpp
    \brief   Test program of variable binary executor
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.04, 2012

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

int main(int argc,char**argv)
{
  using namespace var_space;
  string filename= (argc>1)?argv[1]:"(file does not exist)";
  cout<<"loading "<<filename<<"..."<<endl;

  TTest test;
  TVariable var(test);

  TLiteralTable literal_table;

  if (LoadFromFile (filename,var,literal_table))
  {
    var.WriteToStream(cout,true);

    cout<<"--------"<<endl;
    TBinaryStack bin_stack2;
    var.WriteToBinary(bin_stack2);
    var_space::PrintToStream(bin_stack2);
    TTest test2;
    TVariable var2(test2);
    bin_stack2.GoFirst();
    ExecuteBinary(bin_stack2,var2,literal_table);
    var2.WriteToStream(cout,true);
  }

  return 0;
}
//-------------------------------------------------------------------------------------------
