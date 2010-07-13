//-------------------------------------------------------------------------------------------
/*! \file    base.cpp
    \brief   libskyai - base unit (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.14, 2009-

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
#include <skyai/base.h>
#include <lora/string.h>
#include <lora/small_classes.h>
#include <boost/bind.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;


template <typename t_port>
void ShowAllPortNames (const typename TModuleInterface::TPortSet<t_port>::type &ports,
                        std::ostream &os,
                        bool show_connection,
                        bool show_max_connection,
                        const std::string &prefix)
{
  for (typename TModuleInterface::TPortSet<t_port>::type::const_iterator itr(ports.begin()); itr!=ports.end(); ++itr)
  {
    os<<prefix<<itr->second->Name();
    if (show_connection||show_max_connection) os<<"  cnct: ";
    if (show_connection) os<<itr->second->ConnectionSize()<<"(cur)";
    if (show_connection&&show_max_connection) os<<"/";
    if (show_max_connection) os<<itr->second->MaxConnectionSize()<<"(max)";
    os<<endl;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TPortInterface
//===========================================================================================

const std::string  TPortInterface::UniqueCode() const
{
  return outer_base_.InstanceName()+"."+name_;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TModuleInterface
//===========================================================================================

TPortInterface* TModuleInterface::PortPtr (const std::string &v_name)
{
#define X_SEARCH_PORT(x_type,x_set_name)                                    \
  {                                                                         \
    TPortSet<x_type*>::type::const_iterator item= x_set_name.find(v_name);  \
    if(item!=x_set_name.end())  return  (item->second);                     \
  }
  X_SEARCH_PORT(TOutPortInterface        , out_ports_    )
  X_SEARCH_PORT(TInPortInterface         , in_ports_     )
  X_SEARCH_PORT(TSignalPortInterface     , signal_ports_ )
  X_SEARCH_PORT(TSlotPortInterface       , slot_ports_   )
#undef X_SEARCH_PORT
  return NULL;
}
//-------------------------------------------------------------------------------------------

TPortInterface&  TModuleInterface::Port (const std::string &v_name)
{
  TPortInterface *p= PortPtr(v_name);
  if(p)  return *p;
  LERROR("module "<<ModuleUniqueCode()<<" does not have the specified port "<<v_name);
  lexit(df); return dummy_return<TPortInterface>::value();
}
//-------------------------------------------------------------------------------------------

#define X_SEARCH_PORT(x_type,x_set_name,x_error_msg)                               \
    TPortSet<x_type*>::type::const_iterator item= x_set_name.find(v_name);         \
    if(item!=x_set_name.end())  return *(item->second);                            \
    LERROR(x_error_msg);                                                           \
    lexit(df); return dummy_return<x_type>::value();

//!\brief access an out-port by its name
TOutPortInterface&  TModuleInterface::OutPort (const std::string &v_name)
{
  X_SEARCH_PORT(TOutPortInterface, out_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified out-port "<<v_name)
}
//!\brief access an out-port (const) by its name
const TOutPortInterface&  TModuleInterface::OutPort (const std::string &v_name) const
{
  X_SEARCH_PORT(TOutPortInterface, out_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified out-port "<<v_name)
}

//!\brief access an in-port by its name
TInPortInterface&  TModuleInterface::InPort (const std::string &v_name)
{
  X_SEARCH_PORT(TInPortInterface, in_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified in-port "<<v_name)
}
//!\brief access an in-port (const) by its name
const TInPortInterface&  TModuleInterface::InPort (const std::string &v_name) const
{
  X_SEARCH_PORT(TInPortInterface, in_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified in-port "<<v_name)
}

//!\brief access a signal-port by its name
TSignalPortInterface&  TModuleInterface::SignalPort (const std::string &v_name)
{
  X_SEARCH_PORT(TSignalPortInterface, signal_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified signal-port "<<v_name)
}
//!\brief access a signal-port (const) by its name
const TSignalPortInterface&  TModuleInterface::SignalPort (const std::string &v_name) const
{
  X_SEARCH_PORT(TSignalPortInterface, signal_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified signal-port "<<v_name)
}

//!\brief access a slot-port by its name
TSlotPortInterface&  TModuleInterface::SlotPort (const std::string &v_name)
{
  X_SEARCH_PORT(TSlotPortInterface, slot_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified slot-port "<<v_name)
}
//!\brief access a slot-port (const) by its name
const TSlotPortInterface&  TModuleInterface::SlotPort (const std::string &v_name) const
{
  X_SEARCH_PORT(TSlotPortInterface, slot_ports_,
      "module "<<ModuleUniqueCode()<<" does not have the specified slot-port "<<v_name)
}

#undef X_SEARCH_PORT
//-------------------------------------------------------------------------------------------

/*static*/void TModuleInterface::ParseShowConfOption (const std::string &option, TModuleInterface::TShowConf &conf)
{
  TOptionParser poption(option);
  #define PARSER(x_member,x_opt)  \
    if(poption(x_opt) !="") conf.x_member = ConvertFromStr<bool>(poption(x_opt))
  PARSER(ShowInstanceName       , "in" );
  PARSER(ShowUniqueCode         , "uc" );
  PARSER(ShowParamsConfig       , "prc");
  PARSER(ShowParamsMemory       , "prm");
  PARSER(ShowPorts              , "p"  );
  PARSER(ShowPortsConnection    , "pc" );
  PARSER(ShowPortsMaxConnection , "pmc");
  #undef PARSER
}
//-------------------------------------------------------------------------------------------

void TModuleInterface::ShowModule (const TModuleInterface::TShowConf &conf, std::ostream &os) const
{
  const string delim(" - ");
  const string prefix("  ");
  os<<ioscc::blue<<InheritedModuleName();
  if (conf.ShowInstanceName) os<<delim<<InstanceName();
  if (conf.ShowUniqueCode)   os<<" ("<<ModuleUniqueCode()<<")";
  os<<ioscc::none<<endl;
  if (conf.ShowParamsConfig)
  {
    os<<ioscc::green<<prefix<<"param_box_config:"<<endl;
    param_box_config_.WriteToStream(os, true, prefix*2);
  }
  if (conf.ShowParamsMemory)
  {
    os<<ioscc::green<<prefix<<"param_box_memory:"<<endl;
    param_box_memory_.WriteToStream(os, true, prefix*2);
  }
  if (conf.ShowPorts)
  {
    os<<ioscc::green<<prefix<<"out_ports:"<<endl;
    ShowAllPortNames<TOutPortInterface*>    (out_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
    os<<ioscc::green<<prefix<<"in_ports:"<<endl;
    ShowAllPortNames<TInPortInterface*>     (in_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
    os<<ioscc::green<<prefix<<"signal_ports:"<<endl;
    ShowAllPortNames<TSignalPortInterface*> (signal_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
    os<<ioscc::green<<prefix<<"slot_ports:"<<endl;
    ShowAllPortNames<TSlotPortInterface*>   (slot_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
  }
}
//-------------------------------------------------------------------------------------------

void TModuleInterface::ShowModule (const std::string &option, std::ostream &os) const
{
  TShowConf show_conf;
  ParseShowConfOption (option, show_conf);
  ShowModule (show_conf, os);
}
//-------------------------------------------------------------------------------------------

#define PORT_EXISTING_CHECK(x_port) \
    do{if (PortPtr(x_port.Name())!=NULL)  \
      {LERROR("in module "<<ModuleUniqueCode()<<", port "<<x_port.Name()<<" already exists"); lexit(df);}}while(0)
/*protected*/void TModuleInterface::add_out_port (TOutPortInterface &v_port)
{
  PORT_EXISTING_CHECK(v_port);
  out_ports_[v_port.Name()]= &v_port;
}
/*protected*/void TModuleInterface::add_in_port (TInPortInterface &v_port)
{
  PORT_EXISTING_CHECK(v_port);
  in_ports_[v_port.Name()]= &v_port;
}

/*protected*/void TModuleInterface::add_signal_port (TSignalPortInterface &v_port)
{
  PORT_EXISTING_CHECK(v_port);
  signal_ports_[v_port.Name()]= &v_port;
}
/*protected*/void TModuleInterface::add_slot_port (TSlotPortInterface &v_port)
{
  PORT_EXISTING_CHECK(v_port);
  slot_ports_[v_port.Name()]= &v_port;
  // [-- for signal forwarding
  if (v_port.ForwardingSinalPortBase().Name() != SKYAI_DISABLED_FWD_PORT_NAME)
  {
    PORT_EXISTING_CHECK(v_port.ForwardingSinalPortBase());
    signal_ports_[v_port.ForwardingSinalPortBase().Name()]= &(v_port.ForwardingSinalPortBase());
  }
  // for signal forwarding  --]
}
#undef PORT_EXISTING_CHECK
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TAgent
//===========================================================================================

//! clear all modules and path_list_ (memories are freed)
void TAgent::Clear()
{
  for(TModuleSet::iterator itr(modules_.begin()), ilast(modules_.end()); itr!=ilast; ++itr)
  {
    if(itr->second)  {delete itr->second;}
    itr->second=NULL;
  }
  modules_.clear();

  if(path_list_)  delete path_list_;
  path_list_= NULL;
}
//-------------------------------------------------------------------------------------------

TModuleInterface& TAgent::Module (const std::string &module_name)
{
  TModuleInterface *m (find_module(module_name,/*error=*/true));
  if (m)  return *m;
  lexit(df); return dummy_return<TModuleInterface&>::value();
}
//-------------------------------------------------------------------------------------------

const TModuleInterface& TAgent::Module (const std::string &module_name) const
{
  TModuleInterface *m (find_module(module_name,/*error=*/true));
  if (m)  return *m;
  lexit(df); return dummy_return<TModuleInterface&>::value();
}
//-------------------------------------------------------------------------------------------

//! create a module whose type is specified by the string v_module_class
TModuleInterface&  TAgent::AddModule (const std::string &v_module_class, const std::string &v_instance_name)
{
  if (modules_.find(v_instance_name)!=modules_.end())
  {
    LERROR("module "<<v_instance_name<<" is already registered");
    lexit(df);
  }
  TModuleManager::TGenerator  module_generator (TModuleManager::Generator(v_module_class));
  if (module_generator==NULL)
  {
    LERROR("cannot generate an instance of "<<v_module_class);
    lexit(df);
  }
  TModuleInterface *p= module_generator(v_instance_name);
  // TModuleInterface *p= TModuleManager::Generator(v_module_class)(v_instance_name);
  modules_[v_instance_name]= p;
  p->SetAgent(*this);
  return *p;
}
//-------------------------------------------------------------------------------------------

bool TAgent::ConnectIO(
    TModuleInterface &start_module, const std::string &out_port_name,
    TModuleInterface &end_module,   const std::string &in_port_name)
{
  TOutPortInterface &start (start_module.OutPort(out_port_name));
  TInPortInterface  &end (end_module.InPort(in_port_name));

  if(start.Connect(end) && end.Connect(start))
    return true;
  return false;
}
//-------------------------------------------------------------------------------------------

bool TAgent::ConnectEvent(
    TModuleInterface &start_module, const std::string &signal_port_name,
    TModuleInterface &end_module,   const std::string &slot_port_name)
{
  TSignalPortInterface &start (start_module.SignalPort(signal_port_name));
  TSlotPortInterface &end (end_module.SlotPort(slot_port_name));

  if(start.Connect(end) && end.Connect(start))
    return true;
  return false;
}
//-------------------------------------------------------------------------------------------

/*! add edge.  the port types are automatically determined */
bool TAgent::Connect(
    TModuleInterface &start_module, const std::string &start_port_name,
    TModuleInterface &end_module,   const std::string &end_port_name)
{
  TPortInterface &start (start_module.Port(start_port_name));
  TPortInterface &end (end_module.Port(end_port_name));

  if(start.Connect(end) && end.Connect(start))
    return true;
  return false;
}
//-------------------------------------------------------------------------------------------

/*! add edge. the modules are indicated by names. the port types are automatically determined */
bool TAgent::Connect(
    const std::string &start_module_name, const std::string &start_port_name,
    const std::string &end_module_name,   const std::string &end_port_name)
{
  TModuleSet::iterator itr_start_module (modules_.find(start_module_name));
  if (itr_start_module==modules_.end())  {LERROR("there is no module named "<<start_module_name);}
  TModuleSet::iterator itr_end_module (modules_.find(end_module_name));
  if (itr_end_module==modules_.end())  {LERROR("there is no module named "<<end_module_name);}
  TPortInterface &start (itr_start_module->second->Port(start_port_name));
  TPortInterface &end (itr_end_module->second->Port(end_port_name));

  if(start.Connect(end) && end.Connect(start))
    return true;
  return false;
}
//-------------------------------------------------------------------------------------------

//! \todo implement Disconnect
void TAgent::Disconnect(
    TModuleInterface &start_module, const std::string &start_port_name,
    TModuleInterface &end_module,   const std::string &end_port_name)
{
  FIXME("TAgent::Disconnect is not implemented yet..orz..");
}
//-------------------------------------------------------------------------------------------


/*!\brief for each module, apply the function f */
void TAgent::ForEachModule (boost::function<bool(TModuleInterface* module)> f)
{
  // for each module in modules_ :
  for(TModuleSet::iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
    if (!f(mod_itr->second))  break;
}
//-------------------------------------------------------------------------------------------

/*!\brief for each module, apply the function f */
void TAgent::ForEachModule (boost::function<bool(const TModuleInterface* module)> f) const
{
  // for each module in modules_ :
  for(TModuleSet::const_iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
    if (!f(mod_itr->second))  break;
}
//-------------------------------------------------------------------------------------------

/*!\brief for each connected port, apply the function f */
void TAgent::ForEachConnection (boost::function<bool(TPortInterface* from_port_ptr, TPortInterface* to_port_ptr)> f)
{
  // for each module in modules_ :
  for(TModuleSet::iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::iterator
                op_itr(mod_itr->second->OutPortBegin()), opiend(mod_itr->second->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      // for each port connected to op_itr :
      // NOTE :  from_port_ptr is op_itr->second,  to_port_ptr is _1
      op_itr->second->ForEachConnectedPort (boost::bind(f, op_itr->second, _1));
    }

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::iterator
                sp_itr(mod_itr->second->SlotPortBegin()), spiend(mod_itr->second->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      // for each port connected to sp_itr :
      // NOTE :  from_port_ptr is _1,  to_port_ptr is sp_itr->second
      sp_itr->second->ForEachConnectedPort (boost::bind(f, _1, sp_itr->second));
    }
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief for each connected port, apply the function f */
void TAgent::ForEachConnection (boost::function<bool(const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)> f) const
{
  // for each module in modules_ :
  for(TModuleSet::const_iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::const_iterator
                op_itr(mod_itr->second->OutPortBegin()), opiend(mod_itr->second->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      // for each port connected to op_itr :
      // NOTE :  from_port_ptr is op_itr->second,  to_port_ptr is _1
      op_itr->second->ForEachConnectedPort (boost::bind(f, op_itr->second, _1));
    }

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->second->SlotPortBegin()), spiend(mod_itr->second->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      // for each port connected to sp_itr :
      // NOTE :  from_port_ptr is _1,  to_port_ptr is sp_itr->second
      sp_itr->second->ForEachConnectedPort (boost::bind(f, _1, sp_itr->second));
    }
  }
}
//-------------------------------------------------------------------------------------------

// NOTE: the following member functions are defined in parser.cpp
// bool TAgent::LoadFromFile (const std::string &filename, bool *is_last, std::list<std::string> *included_list);
// bool TAgent::SaveToFile (const std::string &filename) const;

/*!\brief add dir_name (native format path) to the path-list */
void TAgent::AddPath (const std::string &dir_name)
{
  using namespace boost::filesystem;
  if (path_list_==NULL)  path_list_= new std::list<path>;

  path_list_->push_back (complete(path(dir_name,native)));
}
//-------------------------------------------------------------------------------------------

/*!\brief add dir_list (list of native format path) to the path-list */
void TAgent::AddPathList (const std::list<std::string> &dir_list)
{
  using namespace boost::filesystem;
  if (path_list_==NULL)  path_list_= new std::list<path>;

  for (std::list<std::string>::const_iterator itr(dir_list.begin()),last(dir_list.end()); itr!=last; ++itr)
    path_list_->push_back (complete(path(*itr,native)));
}
//-------------------------------------------------------------------------------------------

/*!\brief search filename from the path-list, return the native path */
std::string TAgent::SearchFileName (const std::string &filename) const
{
  using namespace boost::filesystem;
  path  file_path(filename,native), complete_path;
  if (file_path.is_complete())
  {
    if (exists(file_path))  return file_path.file_string();
    return "";
  }

  if (exists(complete_path= complete(file_path)))  return complete_path.file_string();

  if (path_list_==NULL)  return "";

  for (std::list<path>::const_iterator itr(path_list_->begin()),last(path_list_->end()); itr!=last; ++itr)
    if (exists(complete_path= (*itr)/file_path))
      return complete_path.file_string();
  return "";
}
//-------------------------------------------------------------------------------------------

/*!\brief return a complete native path to filename which is a relative path from conf_.DataDir */
std::string TAgent::GetDataFileName (const std::string &filename) const
{
  using namespace boost::filesystem;
  if (filename=="")  return complete(path(conf_.DataDir,native)).file_string();

  path  file_path(filename,native), data_dir_path(conf_.DataDir,native);
  return complete(data_dir_path/file_path).file_string();
}
//-------------------------------------------------------------------------------------------


void TAgent::SetAllModuleMode (const TModuleInterface::TModuleMode &mm)
{
  for(TModuleSet::iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
    mod_itr->second->SetModuleMode (mm);
}
//-------------------------------------------------------------------------------------------

void TAgent::SetDebugStream (std::ostream &os)
{
  for(TModuleSet::iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
    mod_itr->second->SetDebugStream (os);
}
//-------------------------------------------------------------------------------------------


void TAgent::ShowAllModules (const std::string &option, std::ostream &os) const
{
  TModuleInterface::TShowConf  show_conf;
  TModuleInterface::ParseShowConfOption (option, show_conf);
  for(TModuleSet::const_iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // os<< (*mod_itr)->ModuleUniqueCode() <<std::endl;
    mod_itr->second->ShowModule (show_conf, os);
  }
}
//-------------------------------------------------------------------------------------------

//! DEBUG FUNCTION
static bool print_connection (std::ostream *os, const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)
{
  if (from_port_ptr && to_port_ptr)
    *os<<"connect "<<from_port_ptr->UniqueCode()
        <<"  ---->  "<<to_port_ptr->UniqueCode()<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

//! DEBUG FUNCTION
void TAgent::ShowAllConnections (std::ostream &os) const
{
  ForEachConnection (boost::bind(&print_connection,&os,_1,_2));
  // ForEachConnection (print_connection);
}
//-------------------------------------------------------------------------------------------

static bool export_connection_to_dot (
    std::ostream *os, const std::string &indent,
    const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)
{
  if (from_port_ptr && to_port_ptr)
  {
    if (const TSlotPortInterface *slot_port_ptr= dynamic_cast<const TSlotPortInterface *>(to_port_ptr))
    {
      *os<<indent<< "cluster_"<<from_port_ptr->OuterBase().InstanceName()<<"_"<<from_port_ptr->Name()
          <<" -> "<<"cluster_"<<slot_port_ptr->OuterBase().InstanceName()<<"_"<<slot_port_ptr->Name()//<<":s"
          <<" [style=dashed,color=blue];"<<std::endl;
    }
    else
    {
      *os<<indent<< "cluster_"<<from_port_ptr->OuterBase().InstanceName()<<"_"<<from_port_ptr->Name()
          <<" -> "<<"cluster_"<<to_port_ptr->OuterBase().InstanceName()<<"_"<<to_port_ptr->Name()
          <<" [style=solid,color=red];"<<std::endl;
    }
  }
  return true;
}
//-------------------------------------------------------------------------------------------

static inline std::string port_name_to_disp_style (const std::string &name)
{
  // replace '_' by "\\n"
  std::stringstream ss;
  for(std::string::const_iterator itr(name.begin()),end(name.end()); itr!=end; ++itr)
    if(*itr!='_')  ss<<*itr; else ss<<"_\\n";
  return ss.str();
}
//-------------------------------------------------------------------------------------------

/*! TEST: export module structure to a graph description language.
      Draw by graphviz - fdp (e.g. fdp -Tsvg FOO.dot -o BAR.svg) */
void TAgent::ExportToDOT (std::ostream &os) const
{
  const std::string indent("  ");
  std::string cluster_name, port_name;
  os<<"digraph hoge"<<std::endl;
  os<<"{"<<std::endl;

  os<<indent<<"bgcolor=\"transparent\";"<<std::endl;
  os<<indent<<"fontname=\"Trebuchet MS\";"<<std::endl;
  os<<indent<<"// concentrate=true;"<<std::endl;
  os<<indent<<"// rankdir=LR;"<<std::endl;
  os<<indent<<"splines=compound;"<<std::endl;
  os<<indent<<"overlap=prism;"<<std::endl;
  os<<indent<<"smoothing=spring;"<<std::endl;
  os<<indent<<"K=0.5;"<<std::endl;
  os<<std::endl;

  os<<indent<<"edge [arrowhead=vee,penwidth=2,arrowsize=2,weight=1.5];"<<std::endl;
  os<<indent<<"node [fontsize=10, fontname=\"Trebuchet MS\", penwidth=2, sep=100, fixedsize=1,margin=0];"<<std::endl;
  os<<std::endl;

  // for each module in modules_ :
  for(TModuleSet::const_iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    cluster_name= "cluster_"+mod_itr->second->InstanceName();
    os<<indent<<"subgraph "<<cluster_name<<std::endl;
    os<<indent<<"{"<<std::endl;

    os<<indent*2<<"label=\""<<mod_itr->second->InheritedModuleName()
                            <<"\\n  "<<mod_itr->second->InstanceName()<<"\";"<<std::endl;
    os<<indent*2<<"fontsize=16;"<<std::endl;
    // os<<indent*2<<"sep=0.0;"<<std::endl;
    // os<<indent*2<<"margin=0.0;"<<std::endl;

    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::const_iterator
                op_itr(mod_itr->second->OutPortBegin()), opiend(mod_itr->second->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      port_name= cluster_name+"_"+op_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(op_itr->first)<<"\", shape=circle, color=red, width=0.5, height=0.5];"<<std::endl;
    }

    // for each in-port :
    for (TModuleInterface::TPortSet<TInPortInterface*>::type::const_iterator
                ip_itr(mod_itr->second->InPortBegin()), ipiend(mod_itr->second->InPortEnd());
            ip_itr!=ipiend;  ++ip_itr)
    {
      port_name= cluster_name+"_"+ip_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(ip_itr->first)<<"\", shape=rect, color=red, width=0.5, height=0.5];"<<std::endl;
    }

    // for each signal-port :
    for (TModuleInterface::TPortSet<TSignalPortInterface*>::type::const_iterator
                sp_itr(mod_itr->second->SignalPortBegin()), spiend(mod_itr->second->SignalPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      port_name= cluster_name+"_"+sp_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(sp_itr->first)<<"\", shape=doublecircle, color=blue, width=0.3, height=0.3];"<<std::endl;
    }

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->second->SlotPortBegin()), spiend(mod_itr->second->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      port_name= cluster_name+"_"+sp_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(sp_itr->first)<<"\", shape=triangle, color=blue, width=0.5, height=0.5];"<<std::endl;
    }

    os<<indent<<"}"<<std::endl;
  }
  os<<std::endl;

  // for edge between a slot port and its forwarding sinal port
  // for each module in modules_ :
  for(TModuleSet::const_iterator mod_itr(modules_.begin()), miend(modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->second->SlotPortBegin()), spiend(mod_itr->second->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      if (sp_itr->second->ForwardingSinalPortBase().Name()==SKYAI_DISABLED_FWD_PORT_NAME)  continue;
      TSignalPortInterface *fwds_port_ptr (&(sp_itr->second->ForwardingSinalPortBase()));
      os<<indent<< "cluster_"<<sp_itr->second->OuterBase().InstanceName()<<"_"<<sp_itr->second->Name()
         <<" -> "<<"cluster_"<<fwds_port_ptr->OuterBase().InstanceName()<<"_"<<fwds_port_ptr->Name()//<<":s"
         <<" [style=dotted,color=green];"<<std::endl;
    }
  }
  os<<std::endl;

  // for each edge:
  ForEachConnection (boost::bind(&export_connection_to_dot,&os,indent,_1,_2));
  os<<std::endl;

  os<<"}"<<std::endl;

}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

