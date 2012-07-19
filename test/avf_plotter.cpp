//-------------------------------------------------------------------------------------------
/*! \file    avf_plotter.cpp
    \brief   skyai - certain application
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jul.19, 2012

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
#include <skyai/skyai.h>
#include <skyai/interfaces/action_value_func.h>
#include <skyai/utility.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{

//===========================================================================================
class TAVFPlotterConfigurations
//===========================================================================================
{
public:

  TRealVector RangeMinS;
  TRealVector RangeMaxS;
  TRealVector RangeMinA;
  TRealVector RangeMaxA;

  TInt NumDivisionS;
  TInt NumDivisionA;

  TAVFPlotterConfigurations (var_space::TVariableMap &mmap) :
      NumDivisionS (1),
      NumDivisionA (1)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( RangeMinS );
      ADD( RangeMaxS );
      ADD( RangeMinA );
      ADD( RangeMaxA );
      ADD( NumDivisionS );
      ADD( NumDivisionA );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief AVF (Action Value Function) plotter module
class MAVFPlotterModule
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MAVFPlotterModule   TThis;
  typedef TRealVector         TState;
  typedef TRealVector         TAction;
  typedef TRealVector         TFeature;
  SKYAI_MODULE_NAMES(MAVFPlotterModule)

  MAVFPlotterModule (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      slot_start            (*this),
      signal_initialization (*this),
      in_evaluate           (*this),
      in_feature            (*this)
    {
      add_slot_port   (slot_start            );
      add_signal_port (signal_initialization );
      add_in_port     (in_evaluate           );
      add_in_port     (in_feature            );
    }

  void Start()  {slot_start.Exec();}

protected:

  TAVFPlotterConfigurations  conf_;

  MAKE_SLOT_PORT(slot_start, void, (void), (), TThis);

  MAKE_SIGNAL_PORT(signal_initialization, void (void), TThis);

  MAKE_IN_PORT(in_evaluate, void (const TState &x, const TAction &a, TStateActionAttribute attrib), TThis);

  MAKE_IN_PORT(in_feature, const TFeature& (const TState &x), TThis);

  virtual void slot_start_exec (void);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)   \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(evaluate, void, (const TState &x, const TAction &a, TStateActionAttribute attrib), (x,a,attrib))
  GET_FROM_IN_PORT(feature, const TFeature&, (const TState &x), (x))

  #undef GET_FROM_IN_PORT

};  // end of MAVFPlotterModule
//-------------------------------------------------------------------------------------------

/*virtual*/void MAVFPlotterModule::slot_start_exec (void)
{
  signal_initialization.ExecAll();

  TReal ndivs(static_cast<TReal>((conf_.NumDivisionS==1)?1:conf_.NumDivisionS-1));
  TReal ndiva(static_cast<TReal>((conf_.NumDivisionA==1)?1:conf_.NumDivisionA-1));
  TState  steps((conf_.RangeMaxS-conf_.RangeMinS)/ndivs);
  TAction stepa((conf_.RangeMaxA-conf_.RangeMinA)/ndiva);
  TState  s(conf_.RangeMinS);
  TAction a;
  TValue Q;
  TStateActionAttribute attrib;
  attrib.ActionValue= &Q;
  for(int is(conf_.NumDivisionS); is>0; --is)
  {
    a= conf_.RangeMinA;
    for(int ia(conf_.NumDivisionA); ia>0; --ia)
    {
      get_evaluate(get_feature(s),a,attrib);
      std::cout<< s.transpose() << "  " << a.transpose() << "  " << Q <<std::endl;
      a+= stepa;
    }
    std::cout<<std::endl;
    s+= steps;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MAVFPlotterModule)
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  TOptionParser option(argc,argv);
  TAgent  agent;
  if (!ParseCmdLineOption (agent, option))  return 0;

  MAVFPlotterModule *p_plotter = dynamic_cast<MAVFPlotterModule*>(agent.SearchModule("plotter"));
  if(p_plotter==NULL)  {LERROR("module `plotter' is not defined as an instance of MAVFPlotterModule"); return 1;}

  p_plotter->Start();

  return 0;
}
//-------------------------------------------------------------------------------------------
