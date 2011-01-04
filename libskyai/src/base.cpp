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
static void show_all_port_names (
    const typename TModuleInterface::TPortSet<t_port>::type &ports,
    std::ostream &os,
    bool show_connection,
    bool show_max_connection,
    const std::string &prefix )
{
  for (typename TModuleInterface::TPortSet<t_port>::type::const_iterator itr(ports.begin()); itr!=ports.end(); ++itr)
  {
    os<<prefix<<itr->first;
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



//===========================================================================================
// class TModuleInterface
//===========================================================================================

bool TModuleInterface::DecomposePortUniqueCode(const std::string &unique_code, std::string &module_name, std::string &port_name)
{
  TTokenizer token(unique_code);
  module_name= token.ReadIdentifier();
  std::string delim= token.ReadSymbols();
  TrimBoth(delim);
  port_name= token.ReadIdentifier();
  if(module_name=="" || port_name=="" || delim!=SKYAI_MODULE_PORT_DELIMITER || !token.EOL())
    {LERROR("invalid port unique-code: "<<unique_code); return false;}
  return true;
}
//-------------------------------------------------------------------------------------------

/*virtual*/TModuleInterface::~TModuleInterface(void)
{
  LASSERT(!HasActivePorts());
}
//-------------------------------------------------------------------------------------------

#define X_SEARCH_PORT(x_type,x_set_name)                                  \
  TPortSet<x_type*>::type::const_iterator item= x_set_name.find(v_name);  \
  if(item!=x_set_name.end())  return  (item->second);                     \
  return NULL;

TOutPortInterface* TModuleInterface::OutPortPtr (const std::string &v_name)
{
  X_SEARCH_PORT(TOutPortInterface        , out_ports_    )
}
const TOutPortInterface* TModuleInterface::OutPortPtr (const std::string &v_name) const
{
  X_SEARCH_PORT(TOutPortInterface        , out_ports_    )
}
TInPortInterface* TModuleInterface::InPortPtr (const std::string &v_name)
{
  X_SEARCH_PORT(TInPortInterface         , in_ports_     )
}
const TInPortInterface* TModuleInterface::InPortPtr (const std::string &v_name) const
{
  X_SEARCH_PORT(TInPortInterface         , in_ports_     )
}
TSignalPortInterface* TModuleInterface::SignalPortPtr (const std::string &v_name)
{
  X_SEARCH_PORT(TSignalPortInterface     , signal_ports_ )
}
const TSignalPortInterface* TModuleInterface::SignalPortPtr (const std::string &v_name) const
{
  X_SEARCH_PORT(TSignalPortInterface     , signal_ports_ )
}
TSlotPortInterface* TModuleInterface::SlotPortPtr (const std::string &v_name)
{
  X_SEARCH_PORT(TSlotPortInterface       , slot_ports_   )
}
const TSlotPortInterface* TModuleInterface::SlotPortPtr (const std::string &v_name) const
{
  X_SEARCH_PORT(TSlotPortInterface       , slot_ports_   )
}

#undef X_SEARCH_PORT
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

TPortInterface* TModuleInterface::PortPtr (const std::string &v_name)
{
  TPortInterface *p(NULL);
  if ((p= OutPortPtr    (v_name))!=NULL)  return p;
  if ((p= InPortPtr     (v_name))!=NULL)  return p;
  if ((p= SignalPortPtr (v_name))!=NULL)  return p;
  if ((p= SlotPortPtr   (v_name))!=NULL)  return p;
  return NULL;
}
//-------------------------------------------------------------------------------------------

const TPortInterface* TModuleInterface::PortPtr (const std::string &v_name) const
{
  const TPortInterface *p(NULL);
  if ((p= OutPortPtr    (v_name))!=NULL)  return p;
  if ((p= InPortPtr     (v_name))!=NULL)  return p;
  if ((p= SignalPortPtr (v_name))!=NULL)  return p;
  if ((p= SlotPortPtr   (v_name))!=NULL)  return p;
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

const TPortInterface&  TModuleInterface::Port (const std::string &v_name) const
{
  const TPortInterface *p= PortPtr(v_name);
  if(p)  return *p;
  LERROR("module "<<ModuleUniqueCode()<<" does not have the specified port "<<v_name);
  lexit(df); return dummy_return<TPortInterface>::value();
}
//-------------------------------------------------------------------------------------------

bool TModuleInterface::SearchPortByPtr (const TPortInterface *port_ptr, TConstPortInfo &info) const
{
  info.Kind= pkUnknown;
  info.Name= "";
  info.Ptr= NULL;
  info.OuterModule= NULL;
#define X_SEARCH_PORT(x_type,x_port)  \
  for (TPortSet<T##x_type##PortInterface*>::type::const_iterator itr(x_port.begin()),last(x_port.end()); itr!=last; ++itr)  \
    if (itr->second==port_ptr)        \
    {                                 \
      info.Kind= pk##x_type;          \
      info.Name= itr->first;          \
      info.Ptr= port_ptr;             \
      info.OuterModule= this;         \
      return true;                    \
    }
  X_SEARCH_PORT(Out    , out_ports_   );
  X_SEARCH_PORT(In     , in_ports_    );
  X_SEARCH_PORT(Signal , signal_ports_);
  X_SEARCH_PORT(Slot   , slot_ports_  );
#undef X_SEARCH_PORT
  return false;
}
//-------------------------------------------------------------------------------------------
bool TModuleInterface::SearchPortByPtr (TPortInterface *port_ptr, TPortInfo &info)
{
  TConstPortInfo i;
  if(!SearchPortByPtr(port_ptr,i))
  {
    info.Kind= pkUnknown;
    info.Name= "";
    info.Ptr= NULL;
    info.OuterModule= NULL;
    return false;
  }
  info.Kind = i.Kind;
  info.Name = i.Name;
  info.Ptr  = const_cast<TPortInterface*>(i.Ptr);
  info.OuterModule = const_cast<TModuleInterface*>(i.OuterModule);
  return true;
}
//-------------------------------------------------------------------------------------------

//!\brief return true if this module has some active ports
bool TModuleInterface::HasActivePorts() const
{
#define X_SCAN_PORT(x_type,x_port)  \
  for (TPortSet<T##x_type##PortInterface*>::type::const_iterator itr(x_port.begin()),last(x_port.end()); itr!=last; ++itr)  \
    if (itr->second->IsActive())  return true;
  X_SCAN_PORT(Out    , out_ports_   );
  X_SCAN_PORT(In     , in_ports_    );
  X_SCAN_PORT(Signal , signal_ports_);
  X_SCAN_PORT(Slot   , slot_ports_  );
#undef X_SCAN_PORT
  return false;
}
//-------------------------------------------------------------------------------------------

bool TModuleInterface::ExecuteFunction(const std::string &func_name, const std::list<var_space::TLiteral> &argv, bool no_export)
{
  LASSERT(pagent_);
  LASSERT(parent_cmodule_);
  return pagent_->ExecuteFunction(func_name, argv, *parent_cmodule_, no_export);
}
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
  const string prefix("  ");
  os<<ioscc::blue<<InheritedModuleName();
  if (conf.ShowInstanceName) os<<SKYAI_MODULE_NAME_DELIMITER<<InstanceName();
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
    show_all_port_names<TOutPortInterface*>    (out_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
    os<<ioscc::green<<prefix<<"in_ports:"<<endl;
    show_all_port_names<TInPortInterface*>     (in_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
    os<<ioscc::green<<prefix<<"signal_ports:"<<endl;
    show_all_port_names<TSignalPortInterface*> (signal_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
    os<<ioscc::green<<prefix<<"slot_ports:"<<endl;
    show_all_port_names<TSlotPortInterface*>   (slot_ports_, os, conf.ShowPortsConnection, conf.ShowPortsMaxConnection, prefix*2);
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

#define PORT_EXISTING_CHECK(x_port_name) \
    do{if (PortPtr(x_port_name)!=NULL)   \
      {LERROR("in module "<<ModuleUniqueCode()<<", port "<<x_port_name<<" already exists"); lexit(df);}}while(0)
/*protected*/void TModuleInterface::add_out_port    (TOutPortInterface    &v_port, const std::string &v_name)
{
  PORT_EXISTING_CHECK(v_name);
  out_ports_[v_name]= &v_port;
}
/*protected*/void TModuleInterface::add_in_port     (TInPortInterface     &v_port, const std::string &v_name)
{
  PORT_EXISTING_CHECK(v_name);
  in_ports_[v_name]= &v_port;
}
/*protected*/void TModuleInterface::add_signal_port (TSignalPortInterface &v_port, const std::string &v_name)
{
  PORT_EXISTING_CHECK(v_name);
  signal_ports_[v_name]= &v_port;
}
/*protected*/void TModuleInterface::add_slot_port   (TSlotPortInterface   &v_port, const std::string &v_name)
{
  PORT_EXISTING_CHECK(v_name);
  slot_ports_[v_name]= &v_port;
}
#undef PORT_EXISTING_CHECK
//-------------------------------------------------------------------------------------------

/*protected*/void TModuleInterface::add_out_port    (TOutPortInterface    &v_port)
{
  add_out_port(v_port,v_port.OriginalName());
}
/*protected*/void TModuleInterface::add_in_port     (TInPortInterface     &v_port)
{
  add_in_port(v_port,v_port.OriginalName());
}
/*protected*/void TModuleInterface::add_signal_port (TSignalPortInterface &v_port)
{
  add_signal_port(v_port,v_port.OriginalName());
}
/*protected*/void TModuleInterface::add_slot_port   (TSlotPortInterface   &v_port)
{
  add_slot_port(v_port,v_port.OriginalName());
  // [-- for signal forwarding
  if (v_port.ForwardingSinalPortBase().OriginalName() != SKYAI_DISABLED_FWD_PORT_NAME)
  {
    add_signal_port(v_port.ForwardingSinalPortBase(), v_port.ForwardingSinalPortBase().OriginalName());
  }
  // for signal forwarding  --]
}
//-------------------------------------------------------------------------------------------

//! remove port v_name. return true if removed
bool TModuleInterface::remove_port (const std::string &v_name)
{
#define X_REMOVE_PORT(x_type,x_set_name)  \
  {TPortSet<x_type*>::type::iterator item= x_set_name.find(v_name);    \
  if(item!=x_set_name.end())  {x_set_name.erase(item); return true;} }
  X_REMOVE_PORT(TOutPortInterface    , out_ports_   );
  X_REMOVE_PORT(TInPortInterface     , in_ports_    );
  X_REMOVE_PORT(TSignalPortInterface , signal_ports_);
  X_REMOVE_PORT(TSlotPortInterface   , slot_ports_  );
#undef X_REMOVE_PORT
  LWARNING("in module "<<ModuleUniqueCode()<<": failed to remove port "<<v_name);
  return false;
}
//-------------------------------------------------------------------------------------------

void TModuleInterface::clear_ports()
{
  out_ports_   .clear();
  in_ports_    .clear();
  signal_ports_.clear();
  slot_ports_  .clear();
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TCompositeModule
//===========================================================================================

//! clear all modules (memories are freed)
void TCompositeModule::Clear()
{
  clear_ports();

  for(TModuleSet::iterator itr(sub_modules_.begin()), ilast(sub_modules_.end()); itr!=ilast; ++itr)
  {
    if(itr->Ptr!=NULL && itr->Managed)  {delete itr->Ptr;}
    const_cast<TModuleCell&>(*itr).Ptr=NULL;
  }

  sub_modules_.clear();
}
//-------------------------------------------------------------------------------------------

TModuleInterface& TCompositeModule::SubModule (const std::string &module_name)
{
  TModuleInterface *m (find_sub_module(module_name,/*error=*/true));
  if (m)  return *m;
  lexit(df); return dummy_return<TModuleInterface&>::value();
}
//-------------------------------------------------------------------------------------------

const TModuleInterface& TCompositeModule::SubModule (const std::string &module_name) const
{
  TModuleInterface *m (find_sub_module(module_name,/*error=*/true));
  if (m)  return *m;
  lexit(df); return dummy_return<TModuleInterface&>::value();
}
//-------------------------------------------------------------------------------------------

/*! search a module named module_name.
    if a sub-module is a composite one and max_depth>0, module_name is searched recursively */
TModuleInterface* TCompositeModule::SearchSubModule (const std::string &module_name, int max_depth)
{
  list<TCompositeModule*> buf1, buf2, *current_depth(&buf1), *next_depth(&buf2);
  current_depth->push_back(this);
  TCompositeModule *tmp(NULL);

  for(;max_depth>=0;--max_depth)
  {
    for(list<TCompositeModule*>::const_iterator depth_itr(current_depth->begin()),depth_last(current_depth->end()); depth_itr!=depth_last; ++depth_itr)
    {
      for(TModuleSet::iterator mod_itr((*depth_itr)->sub_modules_.begin()), mod_last((*depth_itr)->sub_modules_.end()); mod_itr!=mod_last; ++mod_itr)
      {
        if(mod_itr->Ptr->IsZombie())  continue;
        if(mod_itr->Name==module_name)  return mod_itr->Ptr;
        tmp= dynamic_cast<TCompositeModule*>(mod_itr->Ptr);
        if(tmp)  next_depth->push_back(tmp);
      }
    }
    swap(current_depth,next_depth);
    next_depth->clear();
  }
  return NULL;
}
//-------------------------------------------------------------------------------------------

/*! search a module named module_name.
    if a sub-module is a composite one and max_depth>0, module_name is searched recursively */
const TModuleInterface* TCompositeModule::SearchSubModule (const std::string &module_name, int max_depth) const
{
  list<const TCompositeModule*> buf1, buf2, *current_depth(&buf1), *next_depth(&buf2);
  current_depth->push_back(this);
  const TCompositeModule *tmp(NULL);

  for(;max_depth>=0;--max_depth)
  {
    for(list<const TCompositeModule*>::const_iterator depth_itr(current_depth->begin()),depth_last(current_depth->end()); depth_itr!=depth_last; ++depth_itr)
    {
      for(TModuleSet::const_iterator mod_itr((*depth_itr)->sub_modules_.begin()), mod_last((*depth_itr)->sub_modules_.end()); mod_itr!=mod_last; ++mod_itr)
      {
        if(mod_itr->Ptr->IsZombie())  continue;
        if(mod_itr->Name==module_name)  return mod_itr->Ptr;
        tmp= dynamic_cast<const TCompositeModule*>(mod_itr->Ptr);
        if(tmp)  next_depth->push_back(tmp);
      }
    }
    swap(current_depth,next_depth);
    next_depth->clear();
  }
  return NULL;
}
//-------------------------------------------------------------------------------------------

bool TCompositeModule::SearchSubPort (TPortInterface *port_ptr, TPortInfo &info)
{
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if(mod_itr->Ptr->IsZombie())  continue;
    if(mod_itr->Ptr->SearchPortByPtr(port_ptr,info))  return true;
  }
  info= TPortInfo();
  return false;
}
//-------------------------------------------------------------------------------------------
bool TCompositeModule::SearchSubPort (const TPortInterface *port_ptr, TConstPortInfo &info) const
{
  // copy SearchSubPort; replace iterator by const_iterator
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if(mod_itr->Ptr->IsZombie())  continue;
    if(mod_itr->Ptr->SearchPortByPtr(port_ptr,info))  return true;
  }
  info= TConstPortInfo();
  return false;
}
//-------------------------------------------------------------------------------------------

TPortInterface*  TCompositeModule::SearchSubPort (const std::string &unique_code)
{
  std::string module_name, port_name;
  if (!DecomposePortUniqueCode(unique_code, module_name, port_name))  return NULL;
  if (TModuleInterface *module= SubModulePtr(module_name))
    return module->PortPtr(port_name);
  return NULL;
}
//-------------------------------------------------------------------------------------------
const TPortInterface*  TCompositeModule::SearchSubPort (const std::string &unique_code) const
{
  std::string module_name, port_name;
  if (!DecomposePortUniqueCode(unique_code, module_name, port_name))  return NULL;
  if (const TModuleInterface *module= SubModulePtr(module_name))
    return module->PortPtr(port_name);
  return NULL;
}
//-------------------------------------------------------------------------------------------

std::string  TCompositeModule::SearchSubPortUniqueCode (const TPortInterface *port_ptr) const
{
  TConstPortInfo  port_info;
  SearchSubPort(port_ptr,port_info);
  return port_info.OuterModule->PortUniqueCode(port_info.Name);
}
//-------------------------------------------------------------------------------------------

//! create a module whose type is specified by the string v_module_class
TModuleInterface&  TCompositeModule::AddSubModule (const std::string &v_module_class, const std::string &v_instance_name)
{
  if (sub_modules_.find(TModuleCell(v_instance_name))!=sub_modules_.end())
  {
    LERROR("module "<<v_instance_name<<" is already registered");
    lexit(df);
  }

  TModuleInterface *p(NULL);

  if (Agent().CompositeModuleGenerator().GeneratorExists(v_module_class))
  {
    TCompositeModule *cp = new TCompositeModule(v_module_class, v_instance_name);
    cp->SetAgent(Agent());
    cp->SetParentCModule(this);
    if (!Agent().CompositeModuleGenerator().Create(*cp, v_module_class, v_instance_name))
    {
      delete cp; cp=NULL;
      lexit(df);
    }
    p= cp;
  }
  else
  {
    TModuleManager::TGenerator  module_generator (TModuleManager::Generator(v_module_class));
    if (module_generator==NULL)
    {
      LERROR("cannot generate an instance of "<<v_module_class);
      lexit(df);
    }
    p= module_generator(v_instance_name);
    p->SetAgent(Agent());
    p->SetParentCModule(this);
  }

  sub_modules_.insert(TModuleCell(v_instance_name,p,true));

  return *p;
}
//-------------------------------------------------------------------------------------------

/*! add p into the sub-module set.  this pointer is not managed by this class,
    i.e. the memory is not freed in Clear(), and parameter is not saved (only connection is saved) */
void TCompositeModule::AddUnmanagedSubModule (TModuleInterface *p, const std::string &v_instance_name)
{
  if (sub_modules_.find(TModuleCell(v_instance_name))!=sub_modules_.end())
  {
    LERROR("module "<<v_instance_name<<" is already registered");
    lexit(df);
  }

  sub_modules_.insert(TModuleCell(v_instance_name,p,false));
}
//-------------------------------------------------------------------------------------------

typedef std::list<std::pair<TPortInterface*,TPortInterface*> > TRemoveCList;
bool add_to_remove_list (TPortInterface *first, TPortInterface *second, TRemoveCList *remove_list)
{
  remove_list->push_back(TRemoveCList::value_type(first,second));
  return true;
}
//-------------------------------------------------------------------------------------------

/*! remove the module specified by mod_itr. all connections are disconnected */
/*protected*/bool TCompositeModule::remove_sub_module (TModuleSet::iterator mod_itr)
{
  LASSERT(mod_itr!=sub_modules_.end());

  // NOTE: if the module has some active ports, the module is not removed, but is set the zombie flag
  if (mod_itr->Ptr->HasActivePorts())
  {
    LERROR("failed to remove: "<<mod_itr->Name<<" has some active ports");
    mod_itr->Ptr->SetZombie(true);
    return false;
  }

  // remove all connections connected to mod_itr
  TRemoveCList remove_list;
  #define ADD_TO_REMOVE_LIST(x_type)  \
    for (TModuleInterface::TPortSet<T##x_type##PortInterface*>::type::iterator  \
                port_itr(mod_itr->Ptr->x_type##PortBegin()), port_last(mod_itr->Ptr->x_type##PortEnd()); port_itr!=port_last;  ++port_itr)  \
      port_itr->second->ForEachConnectedPort (boost::bind(&add_to_remove_list, port_itr->second, _1, &remove_list));
  ADD_TO_REMOVE_LIST(Out)
  ADD_TO_REMOVE_LIST(In)
  ADD_TO_REMOVE_LIST(Signal)
  ADD_TO_REMOVE_LIST(Slot)
  #undef ADD_TO_REMOVE_LIST
  for(TRemoveCList::iterator rm_itr(remove_list.begin()),rm_last(remove_list.end()); rm_itr!=rm_last; ++rm_itr)
    if (!rm_itr->first->Disconnect(rm_itr->second) || !rm_itr->second->Disconnect(rm_itr->first))
      {LERROR("fatal!"); lexit(df);}

  // remove exported items...
  for (std::list<TExportItem>::iterator itr(export_list_.begin()),last(export_list_.end()); itr!=last; )
  {
    if (mod_itr->Name==itr->ModuleName)
    {
      switch (itr->Kind)
      {
      case ekPort   : remove_port(itr->ExportName);  break;
      case ekConfig : ParamBoxConfig().RemoveMemberVariable(itr->ExportName); break;
      case ekMemory : ParamBoxMemory().RemoveMemberVariable(itr->ExportName); break;
      default : LERROR("fatal!"); lexit(df);
      }
      itr= export_list_.erase(itr);
    }
    else
      ++itr;
  }

  // free memory of the module
  if (mod_itr->Ptr!=NULL && mod_itr->Managed)  {delete mod_itr->Ptr;}
  const_cast<TModuleCell&>(*mod_itr).Ptr=NULL;

  // erase mod_itr from sub_modules_
  sub_modules_.erase(mod_itr);

  return true;
}
//-------------------------------------------------------------------------------------------

/*! remove a module of the instance name. all connections are disconnected */
bool TCompositeModule::RemoveSubModule (const std::string &v_instance_name)
{
  TModuleSet::iterator mod_itr (sub_modules_.find(TModuleCell(v_instance_name)));
  if (mod_itr==sub_modules_.end())  {LERROR(v_instance_name<<" does not exist"); return false;}

  return remove_sub_module(mod_itr);
}
//-------------------------------------------------------------------------------------------

/*! remove all zombies */
bool TCompositeModule::ClearZombies ()
{
  bool removed_all(true);
  std::list<TModuleSet::iterator>  remove_list;
  for (TModuleSet::iterator itr(sub_modules_.begin()),last(sub_modules_.end()); itr!=last; ++itr)
    if (itr->Ptr->IsZombie())  remove_list.push_back(itr);
  for (std::list<TModuleSet::iterator>::iterator itr(remove_list.begin()),last(remove_list.end()); itr!=last; ++itr)
    if (!remove_sub_module(*itr))  removed_all= false;
  return removed_all;
}
//-------------------------------------------------------------------------------------------

/*! add edge.  the port types are automatically determined */
bool TCompositeModule::SubConnect(
    TModuleInterface &start_module, const std::string &start_port_name,
    TModuleInterface &end_module,   const std::string &end_port_name)
{
  TPortInterface &start (start_module.Port(start_port_name));
  TPortInterface &end (end_module.Port(end_port_name));

  if (start.Connect(end) && end.Connect(start))
    return true;
  std::cerr<<"  failed to connect ports:"<<std::endl;
  std::cerr<<"    from port: "<<&start<<", "<<start_module.InstanceName()<<SKYAI_MODULE_PORT_DELIMITER<<start_port_name<<std::endl;
  std::cerr<<"    to port:   "<<&end<<", "<<end_module.InstanceName()<<SKYAI_MODULE_PORT_DELIMITER<<end_port_name<<std::endl;
  return false;
}
//-------------------------------------------------------------------------------------------

/*! add edge. the modules are indicated by names. the port types are automatically determined */
bool TCompositeModule::SubConnect(
    const std::string &start_module_name, const std::string &start_port_name,
    const std::string &end_module_name,   const std::string &end_port_name)
{
  TModuleSet::iterator itr_start_module (sub_modules_.find(TModuleCell(start_module_name)));
  if (itr_start_module==sub_modules_.end() || itr_start_module->Ptr->IsZombie())  {LERROR(start_module_name<<" does not exist"); return false;}
  TModuleSet::iterator itr_end_module (sub_modules_.find(TModuleCell(end_module_name)));
  if (itr_end_module==sub_modules_.end() || itr_end_module->Ptr->IsZombie())  {LERROR(end_module_name<<" does not exist"); return false;}
  TPortInterface &start (itr_start_module->Ptr->Port(start_port_name));
  TPortInterface &end (itr_end_module->Ptr->Port(end_port_name));

  if (start.Connect(end) && end.Connect(start))
    return true;
  std::cerr<<"  failed to connect ports:"<<std::endl;
  std::cerr<<"    from port: "<<&start<<", "<<start_module_name<<SKYAI_MODULE_PORT_DELIMITER<<start_port_name<<std::endl;
  std::cerr<<"    to port:   "<<&end<<", "<<end_module_name<<SKYAI_MODULE_PORT_DELIMITER<<end_port_name<<std::endl;
  return false;
}
//-------------------------------------------------------------------------------------------

/*! disconnect the two ports.  the port types are automatically determined */
bool TCompositeModule::SubDisconnect(
    TModuleInterface &start_module, const std::string &start_port_name,
    TModuleInterface &end_module,   const std::string &end_port_name)
{
  TPortInterface &start (start_module.Port(start_port_name));
  TPortInterface &end (end_module.Port(end_port_name));

  if (start.Disconnect(&end) && end.Disconnect(&start))
    return true;
  std::cerr<<"  failed to disconnect ports:"<<std::endl;
  std::cerr<<"    from port: "<<&start<<", "<<start_module.InstanceName()<<SKYAI_MODULE_PORT_DELIMITER<<start_port_name<<std::endl;
  std::cerr<<"    to port:   "<<&end<<", "<<end_module.InstanceName()<<SKYAI_MODULE_PORT_DELIMITER<<end_port_name<<std::endl;
  return false;
}
//-------------------------------------------------------------------------------------------

/*! disconnect the two ports. the modules are indicated by names. the port types are automatically determined */
bool TCompositeModule::SubDisconnect(
    const std::string &start_module_name, const std::string &start_port_name,
    const std::string &end_module_name,   const std::string &end_port_name)
{
  TModuleSet::iterator itr_start_module (sub_modules_.find(TModuleCell(start_module_name)));
  if (itr_start_module==sub_modules_.end() || itr_start_module->Ptr->IsZombie())  {LERROR(start_module_name<<" does not exist"); return false;}
  TModuleSet::iterator itr_end_module (sub_modules_.find(TModuleCell(end_module_name)));
  if (itr_end_module==sub_modules_.end() || itr_end_module->Ptr->IsZombie())  {LERROR(end_module_name<<" does not exist"); return false;}
  TPortInterface &start (itr_start_module->Ptr->Port(start_port_name));
  TPortInterface &end (itr_end_module->Ptr->Port(end_port_name));

  if (start.Disconnect(&end) && end.Disconnect(&start))
    return true;
  std::cerr<<"  failed to disconnect ports:"<<std::endl;
  std::cerr<<"    from port: "<<&start<<", "<<start_module_name<<SKYAI_MODULE_PORT_DELIMITER<<start_port_name<<std::endl;
  std::cerr<<"    to port:   "<<&end<<", "<<end_module_name<<SKYAI_MODULE_PORT_DELIMITER<<end_port_name<<std::endl;
  return false;
}
//-------------------------------------------------------------------------------------------


/*!\brief for each module, apply the function f */
void TCompositeModule::ForEachSubModule (boost::function<bool(TModuleInterface* module)> f)
{
  // for each module in sub_modules_ :
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;
    if (!f(mod_itr->Ptr))  break;
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief for each module, apply the function f */
void TCompositeModule::ForEachSubModule (boost::function<bool(const TModuleInterface* module)> f) const
{
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;
    if (!f(mod_itr->Ptr))  break;
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief for each module, apply the function f */
void TCompositeModule::ForEachSubModuleCell (boost::function<bool(const TModuleCell &mcell)> f) const
{
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;
    if (!f(*mod_itr))  break;
  }
}
//-------------------------------------------------------------------------------------------

/*protected*/bool TCompositeModule::apply_f_if_to_port_exists (const TPortInfo *from_port, TPortInterface *to_port_ptr, TConnectionManipulator f)
{
  TPortInfo to_port;
  if (SearchSubPort(to_port_ptr,to_port))
  {
    LASSERT(to_port.Kind==pkIn||to_port.Kind==pkSlot);
    f(from_port,&to_port);
  }
  return true;
}
/*protected*/bool TCompositeModule::apply_f_if_to_port_exists_c (const TConstPortInfo *from_port, const TPortInterface *to_port_ptr, TConstConnectionManipulator f) const
{
  TConstPortInfo to_port;
  if (SearchSubPort(to_port_ptr,to_port))
  {
    LASSERT(to_port.Kind==pkIn||to_port.Kind==pkSlot);
    f(from_port,&to_port);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

/*!\brief for each connected port, apply the function f */
void TCompositeModule::ForEachSubConnection (TConnectionManipulator f)
{
  TPortInfo from_port;
  // for each module in sub_modules_ :
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;

    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::iterator
                op_itr(mod_itr->Ptr->OutPortBegin()), opiend(mod_itr->Ptr->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      from_port.Kind = pkOut;
      from_port.Name = op_itr->first;
      from_port.Ptr  = op_itr->second;
      from_port.OuterModule = mod_itr->Ptr;
      // for each port connected to op_itr :
      op_itr->second->ForEachConnectedPort (boost::bind(&TCompositeModule::apply_f_if_to_port_exists, this, &from_port, _1, f));
    }

    // for each signal-port :
    for (TModuleInterface::TPortSet<TSignalPortInterface*>::type::iterator
                sp_itr(mod_itr->Ptr->SignalPortBegin()), spiend(mod_itr->Ptr->SignalPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      from_port.Kind = pkSignal;
      from_port.Name = sp_itr->first;
      from_port.Ptr  = sp_itr->second;
      from_port.OuterModule = mod_itr->Ptr;
      // for each port connected to sp_itr :
      sp_itr->second->ForEachConnectedPort (boost::bind(&TCompositeModule::apply_f_if_to_port_exists, this, &from_port, _1, f));
    }
  }
}
//-------------------------------------------------------------------------------------------

/*!\brief for each connected port, apply the function f */
void TCompositeModule::ForEachSubConnection (TConstConnectionManipulator f) const
{
  // copy ForEachSubConnection; iterator --> const_iterator: apply_f_if_to_port_exists --> apply_f_if_to_port_exists_c;
  // TPortInfo --> TConstPortInfo; op_itr->second --> const_cast<const TOutPortInterface*>(op_itr->second)
  // sp_itr->second --> const_cast<const TSignalPortInterface*>(sp_itr->second)
  TConstPortInfo from_port;
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;

    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::const_iterator
                op_itr(mod_itr->Ptr->OutPortBegin()), opiend(mod_itr->Ptr->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      from_port.Kind = pkOut;
      from_port.Name = op_itr->first;
      from_port.Ptr  = op_itr->second;
      from_port.OuterModule = mod_itr->Ptr;
      // for each port connected to op_itr :
      const_cast<const TOutPortInterface*>(op_itr->second)->ForEachConnectedPort (boost::bind(&TCompositeModule::apply_f_if_to_port_exists_c, this, &from_port, _1, f));
    }

    // for each signal-port :
    for (TModuleInterface::TPortSet<TSignalPortInterface*>::type::const_iterator
                sp_itr(mod_itr->Ptr->SignalPortBegin()), spiend(mod_itr->Ptr->SignalPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      from_port.Kind = pkSignal;
      from_port.Name = sp_itr->first;
      from_port.Ptr  = sp_itr->second;
      from_port.OuterModule = mod_itr->Ptr;
      // for each port connected to sp_itr :
      const_cast<const TSignalPortInterface*>(sp_itr->second)->ForEachConnectedPort (boost::bind(&TCompositeModule::apply_f_if_to_port_exists_c, this, &from_port, _1, f));
    }
  }
}
//-------------------------------------------------------------------------------------------


/*!\brief export a port sub_module_name.port_name as export_name */
bool TCompositeModule::ExportPort (const std::string &sub_module_name, const std::string &port_name, const std::string &export_name)
{
  export_list_.push_back (TExportItem(ekPort, sub_module_name, port_name, export_name));

  TModuleInterface  &sub_module(SubModule(sub_module_name));
  if (TOutPortInterface    *p= sub_module.OutPortPtr    (port_name))  {add_out_port    (*p,export_name); return true;}
  if (TInPortInterface     *p= sub_module.InPortPtr     (port_name))  {add_in_port     (*p,export_name); return true;}
  if (TSignalPortInterface *p= sub_module.SignalPortPtr (port_name))  {add_signal_port (*p,export_name); return true;}
  if (TSlotPortInterface   *p= sub_module.SlotPortPtr   (port_name))  {add_slot_port   (*p,export_name); return true;}
  LERROR(sub_module_name<<" does not have a port named "<<port_name);
  lexit(df);
  return false;
}
//-------------------------------------------------------------------------------------------

/*!\brief export a config-parameter sub_module_name.config.param_name as export_name */
bool TCompositeModule::ExportConfig (const std::string &sub_module_name, const std::string &param_name, const std::string &export_name)
{
  export_list_.push_back (TExportItem(ekConfig, sub_module_name, param_name, export_name));

  std::string id(param_name);
  if (!ParamBoxConfig().AddMemberVariable (export_name, SubModule(sub_module_name).ParamBoxConfig().GetMember(var_space::TVariable(id))))
  {
    LERROR("failed to export "<<sub_module_name<<".config."<<param_name<<" as "<<export_name);
    lexit(df);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

/*!\brief export a memory-parameter sub_module_name.memory.param_name as export_name */
bool TCompositeModule::ExportMemory (const std::string &sub_module_name, const std::string &param_name, const std::string &export_name)
{
  export_list_.push_back (TExportItem(ekMemory, sub_module_name, param_name, export_name));

  std::string id(param_name);
  if (!ParamBoxMemory().AddMemberVariable (export_name, SubModule(sub_module_name).ParamBoxMemory().GetMember(var_space::TVariable(id))))
  {
    LERROR("failed to export "<<sub_module_name<<".memory."<<param_name<<" as "<<export_name);
    lexit(df);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------


// NOTE: the following member function is defined in parser.cpp
// bool TCompositeModule::WriteToStream (std::ostream &os, const std::string &indent) const;


void TCompositeModule::SetAllSubModuleMode (const TModuleInterface::TModuleMode &mm)
{
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
    mod_itr->Ptr->SetModuleMode (mm);
}
//-------------------------------------------------------------------------------------------

void TCompositeModule::SetAllSubDebugStream (std::ostream &os)
{
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
    mod_itr->Ptr->SetDebugStream (os);
}
//-------------------------------------------------------------------------------------------

void TCompositeModule::ShowAllSubModules (const std::string &option, std::ostream &os) const
{
  TModuleInterface::TShowConf  show_conf;
  TModuleInterface::ParseShowConfOption (option, show_conf);
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;
    mod_itr->Ptr->ShowModule (show_conf, os);
  }
}
//-------------------------------------------------------------------------------------------

static bool print_connection (std::ostream *os, const TConstPortInfo *from_port, const TConstPortInfo *to_port)
{
  if (from_port && to_port)
    *os<<"connect "<<from_port->OuterModule->PortUniqueCode(from_port->Name)
        <<"  ---->  "<<to_port->OuterModule->PortUniqueCode(to_port->Name)<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

void TCompositeModule::ShowAllSubConnections (std::ostream &os) const
{
  ForEachSubConnection (boost::bind(&print_connection,&os,_1,_2));
}
//-------------------------------------------------------------------------------------------

static bool export_connection_to_dot (
    std::ostream *os, const std::string &indent,
    const TConstPortInfo *from_port, const TConstPortInfo *to_port)
{
  if (from_port && to_port)
  {
    if (to_port->Kind==pkSlot)
    {
      *os<<indent<< "cluster_"<<from_port->OuterModule->InstanceName()<<"_"<<from_port->Name
          <<" -> "<<"cluster_"<<to_port->OuterModule->InstanceName()<<"_"<<to_port->Name//<<":s"
          <<" [style=dashed,color=blue];"<<std::endl;
    }
    else if (to_port->Kind==pkIn)
    {
      *os<<indent<< "cluster_"<<from_port->OuterModule->InstanceName()<<"_"<<from_port->Name
          <<" -> "<<"cluster_"<<to_port->OuterModule->InstanceName()<<"_"<<to_port->Name
          <<" [style=solid,color=red];"<<std::endl;
    }
    else {LERROR("fatal!"); lexit(df);}
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
void TCompositeModule::ExportToDOT (std::ostream &os) const
{
  const std::string indent("  ");
  std::string cluster_name, port_name;
  os<<"digraph agent"<<std::endl;
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

  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;

    cluster_name= "cluster_"+mod_itr->Ptr->InstanceName();
    os<<indent<<"subgraph "<<cluster_name<<std::endl;
    os<<indent<<"{"<<std::endl;

    os<<indent*2<<"label=\""<<mod_itr->Ptr->InheritedModuleName()
                            <<"\\n  "<<mod_itr->Ptr->InstanceName()<<"\";"<<std::endl;
    os<<indent*2<<"fontsize=16;"<<std::endl;
    // os<<indent*2<<"sep=0.0;"<<std::endl;
    // os<<indent*2<<"margin=0.0;"<<std::endl;

    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::const_iterator
                op_itr(mod_itr->Ptr->OutPortBegin()), opiend(mod_itr->Ptr->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      port_name= cluster_name+"_"+op_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(op_itr->first)<<"\", shape=circle, color=red, width=0.5, height=0.5];"<<std::endl;
    }

    // for each in-port :
    for (TModuleInterface::TPortSet<TInPortInterface*>::type::const_iterator
                ip_itr(mod_itr->Ptr->InPortBegin()), ipiend(mod_itr->Ptr->InPortEnd());
            ip_itr!=ipiend;  ++ip_itr)
    {
      port_name= cluster_name+"_"+ip_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(ip_itr->first)<<"\", shape=rect, color=red, width=0.5, height=0.5];"<<std::endl;
    }

    // for each signal-port :
    for (TModuleInterface::TPortSet<TSignalPortInterface*>::type::const_iterator
                sp_itr(mod_itr->Ptr->SignalPortBegin()), spiend(mod_itr->Ptr->SignalPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      port_name= cluster_name+"_"+sp_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(sp_itr->first)<<"\", shape=doublecircle, color=blue, width=0.3, height=0.3];"<<std::endl;
    }

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->Ptr->SlotPortBegin()), spiend(mod_itr->Ptr->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      port_name= cluster_name+"_"+sp_itr->first;
      os<<indent*2<<port_name<<" [label=\""<<port_name_to_disp_style(sp_itr->first)<<"\", shape=triangle, color=blue, width=0.5, height=0.5];"<<std::endl;
    }

    os<<indent<<"}"<<std::endl;
  }
  os<<std::endl;

  // for edge between a slot port and its forwarding sinal port
#if 0
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    if (mod_itr->Ptr->IsZombie())  continue;

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->Ptr->SlotPortBegin()), spiend(mod_itr->Ptr->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      if (sp_itr->second->ForwardingSinalPortBase().OriginalName()==SKYAI_DISABLED_FWD_PORT_NAME)  continue;
      TConstPortInfo fwds_port_info;
      LASSERT(mod_itr->Ptr->SearchPortByPtr(&(sp_itr->second->ForwardingSinalPortBase()),fwds_port_info));
      os<<indent<< "cluster_"<<mod_itr->Name<<"_"<<sp_itr->first
         <<" -> "<<"cluster_"<<mod_itr->Name<<"_"<<fwds_port_info.Name//<<":s"
         <<" [style=dotted,color=green];"<<std::endl;
    }
  }
  os<<std::endl;
#endif

  // for each edge:
  ForEachSubConnection (boost::bind(&export_connection_to_dot,&os,indent,_1,_2));
  os<<std::endl;

  os<<"}"<<std::endl;

}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TCompositeModuleGenerator
//===========================================================================================

//! Added: return true, failed: return false
bool TCompositeModuleGenerator::AddGenerator(const std::string &cmodule_name, const TGeneratorInfo &generator)
{
  if(GeneratorExists(cmodule_name))  {LERROR(cmodule_name<<" already exists");  return false;}
  generators_[cmodule_name]= generator;
  cmodule_name_list_.push_back(cmodule_name);
  return true;
}
//-------------------------------------------------------------------------------------------

//! Removed: return true, failed: return false
bool TCompositeModuleGenerator::RemoveGenerator(const std::string &cmodule_name)
{
  std::map<std::string, TGeneratorInfo>::iterator  g_itr(generators_.find(cmodule_name));
  std::list<std::string>::iterator   cnl_itr(std::find(cmodule_name_list_.begin(),cmodule_name_list_.end(),cmodule_name));
  if (g_itr==generators_.end() && cnl_itr==cmodule_name_list_.end())
    {LERROR(cmodule_name<<": module (composite) not found"); return false;}
  if (g_itr!=generators_.end())  generators_.erase(g_itr);
  if (cnl_itr!=cmodule_name_list_.end())  cmodule_name_list_.erase(cnl_itr);
  return true;
}
//-------------------------------------------------------------------------------------------

bool TCompositeModuleGenerator::GeneratorExists(const std::string &cmodule_name) const
{
  std::map<std::string, TGeneratorInfo>::const_iterator  itr(generators_.find(cmodule_name));
  return itr!=generators_.end();
}
//-------------------------------------------------------------------------------------------

const TCompositeModuleGenerator::TGeneratorInfo* TCompositeModuleGenerator::Generator(const std::string &cmodule_name) const
{
  std::map<std::string, TGeneratorInfo>::const_iterator  itr(generators_.find(cmodule_name));
  if (itr==generators_.end())
  {
    LERROR(cmodule_name<<": module (composite) not found");
    return NULL;
  }
  return &(itr->second);
}

// NOTE: the following member functions are defined in parser.cpp
// bool TCompositeModuleGenerator::Create(TCompositeModule &instance, const std::string &cmodule_name, const std::string &instance_name, bool no_export) const;
// bool TCompositeModuleGenerator::WriteToStream (std::ostream &os, const std::string &indent) const;


//===========================================================================================
// class TFunctionManager
//===========================================================================================
//! Added: return true, failed: return false
bool TFunctionManager::AddFunction(const std::string &func_name, const TFunctionInfo &function)
{
  if(FunctionExists(func_name))  {LERROR(func_name<<" already exists");  return false;}
  functions_[func_name]= function;
// std::stringstream plist;PrintContainer(functions_[func_name].ParamList.begin(),functions_[func_name].ParamList.end(),plist, ",");
// LDEBUG("#####ADDED FUNCTION: "<<func_name
// <<"("<<plist.str()<<"); defined in "<<functions_[func_name].FileName<<":"<<functions_[func_name].LineNum<<endl
// <<functions_[func_name].Script<<"#####");
  return true;
}
//-------------------------------------------------------------------------------------------

//! Removed: return true, failed: return false
bool TFunctionManager::RemoveFunction(const std::string &func_name)
{
  std::map<std::string, TFunctionInfo>::iterator  itr(functions_.find(func_name));
  if (itr==functions_.end())
    {LERROR(func_name<<": function not found");  return false;}
  else
    functions_.erase(itr);
  return true;
}
//-------------------------------------------------------------------------------------------

bool TFunctionManager::FunctionExists(const std::string &func_name) const
{
  std::map<std::string, TFunctionInfo>::const_iterator  itr(functions_.find(func_name));
  return itr!=functions_.end();
}
//-------------------------------------------------------------------------------------------

const TFunctionManager::TFunctionInfo* TFunctionManager::Function(const std::string &func_name) const
{
  std::map<std::string, TFunctionInfo>::const_iterator  itr(functions_.find(func_name));
  if (itr==functions_.end())
  {
    LERROR(func_name<<": function not found");
    return NULL;
  }
  return &(itr->second);
}
//-------------------------------------------------------------------------------------------

// NOTE: the following member function is defined in parser.cpp
// bool TFunctionManager::WriteToStream (std::ostream &os, const std::string &indent) const;


//===========================================================================================
// class TAgent
//===========================================================================================

//! clear all modules and path_list_ (memories are freed)
void TAgent::Clear()
{
  modules_.Clear();

  if(path_list_)  delete path_list_;
  path_list_= NULL;
}
//-------------------------------------------------------------------------------------------

// NOTE: the following member functions are defined in parser.cpp
// bool TAgent::LoadFromFile (const std::string &filename, std::list<std::string> *included_list);
// bool TAgent::SaveToFile (const std::string &filename) const;

/*!\brief add dir_name (native format path) to the path-list */
void TAgent::AddPath (const std::string &dir_name)
{
  using namespace boost::filesystem;
  SetPathList().push_back (complete(path(dir_name,native)));
}
//-------------------------------------------------------------------------------------------

/*!\brief add dir_list (list of native format path) to the path-list */
void TAgent::AddPathList (const std::list<std::string> &dir_list)
{
  using namespace boost::filesystem;
  SetPathList();

  for (std::list<std::string>::const_iterator itr(dir_list.begin()),last(dir_list.end()); itr!=last; ++itr)
    path_list_->push_back (complete(path(*itr,native)));
}
//-------------------------------------------------------------------------------------------

const std::list<boost::filesystem::path>&  TAgent::PathList()
{
  return SetPathList();
}
//-------------------------------------------------------------------------------------------
const std::list<boost::filesystem::path>&  TAgent::PathList() const
{
  return *path_list_;
}
//-------------------------------------------------------------------------------------------
std::list<boost::filesystem::path>&  TAgent::SetPathList()
{
  using namespace boost::filesystem;
  if (path_list_==NULL)  path_list_= new std::list<path>;
  return *path_list_;
}
//-------------------------------------------------------------------------------------------

bool TAgent::ExecuteFunction(
        const std::string &func_name, const std::list<var_space::TLiteral> &argv,
        TCompositeModule &context_cmodule,  bool no_export)
{
  return function_manager_.ExecuteFunction(func_name, argv, context_cmodule,
            boost::filesystem::initial_path(),
            /*path_list=*/NULL, /*included_list=*/NULL,
            &cmp_module_generator_, no_export);
}
//-------------------------------------------------------------------------------------------

// NOTE: the following member function is defined in parser.cpp
// bool TAgent::ExecuteScript(
        // const std::string &exec_script, TCompositeModule &context_cmodule, std::list<std::string> *included_list,
        // const std::string &file_name, int start_line_num, bool no_export);

/*!\brief search filename from the path-list, return the native path
    \param [in]omissible_extension  :  indicate an extension with dot, such as ".agent" */
std::string TAgent::SearchFileName (const std::string &filename, const std::string &omissible_extension) const
{
  using namespace boost::filesystem;
  path  file_path(filename,native), complete_path;
  if (file_path.is_complete())
  {
    if (exists(file_path))  return file_path.file_string();
    if (exists((complete_path= file_path.parent_path()/(file_path.filename()+omissible_extension))))  return complete_path.file_string();
    return "";
  }

  if (exists(complete_path= complete(file_path)))  return complete_path.file_string();
  if (exists(complete_path= complete(file_path.string()+omissible_extension)))  return complete_path.file_string();

  if (path_list_==NULL)  return "";

  path  tmp_path(file_path);
  for (std::list<path>::const_iterator itr(path_list_->begin()),last(path_list_->end()); itr!=last; ++itr)
    if (exists(complete_path= (*itr)/tmp_path))
      return complete_path.file_string();
  tmp_path= file_path.string()+omissible_extension;
  for (std::list<path>::const_iterator itr(path_list_->begin()),last(path_list_->end()); itr!=last; ++itr)
    if (exists(complete_path= (*itr)/tmp_path))
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


struct TCompletePortInfo
{
  TPortKind Kind;
  std::list<std::pair<std::string, std::string> >  UniqueCodes;
};

static bool dump_port_info (const TCompositeModule::TModuleCell &mcell, std::map<TPortInterface*,TCompletePortInfo> *port_info)
{
  typedef TModuleInterface::TPortSet<TOutPortInterface*>::type::const_iterator    t_Out_itr;
  typedef TModuleInterface::TPortSet<TInPortInterface*>::type::const_iterator     t_In_itr;
  typedef TModuleInterface::TPortSet<TSignalPortInterface*>::type::const_iterator t_Signal_itr;
  typedef TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator   t_Slot_itr;

  #define DUMP_EVERY_PORT_INFO(x_type) \
    for (t_##x_type##_itr itr(mcell.Ptr->x_type##PortBegin()),                          \
                          last(mcell.Ptr->x_type##PortEnd()); itr!=last; ++itr)         \
    {                                                                                   \
      std::pair<std::string, std::string> unique_code(mcell.Name,itr->first);           \
      std::map<TPortInterface*,TCompletePortInfo>::iterator                             \
                                                pi_itr(port_info->find(itr->second));   \
      if(pi_itr!=port_info->end())                                                      \
      {                                                                                 \
        LASSERT1op1(pi_itr->second.Kind,==,pk##x_type);                                 \
        pi_itr->second.UniqueCodes.push_front(unique_code);                             \
      }                                                                                 \
      else                                                                              \
      {                                                                                 \
        TCompletePortInfo tmp;                                                          \
        tmp.Kind= pk##x_type;                                                           \
        tmp.UniqueCodes.push_front(unique_code);                                        \
        port_info->insert(                                                              \
            std::map<TPortInterface*,TCompletePortInfo>::value_type(itr->second,tmp));  \
      }                                                                                 \
    }
  DUMP_EVERY_PORT_INFO(Out);
  DUMP_EVERY_PORT_INFO(In);
  DUMP_EVERY_PORT_INFO(Signal);
  DUMP_EVERY_PORT_INFO(Slot);
  #undef DUMP_EVERY_PORT_INFO

  if (const TCompositeModule *cmod= dynamic_cast<const TCompositeModule*>(mcell.Ptr))
    cmod->ForEachSubModuleCell(boost::bind(&dump_port_info,_1,port_info));

  return true;
}
//-------------------------------------------------------------------------------------------

/*!\brief write all port information (port kind, port pointer, every outer module name and port name) to os */
void TAgent::DumpPortInfo (std::ostream &os) const
{
  std::map<TPortInterface*,TCompletePortInfo>  port_info;
  modules_.ForEachSubModuleCell(boost::bind(&dump_port_info,_1,&port_info));
  for (std::map<TPortInterface*,TCompletePortInfo>::const_iterator itr(port_info.begin()),last(port_info.end()); itr!=last; ++itr)
  {
    os<<"port "<<itr->first<<" ("<<ConvertToStr(itr->second.Kind)<<"):"<<std::endl;
    for (std::list<std::pair<std::string, std::string> >::const_iterator uc_itr(itr->second.UniqueCodes.begin()),uc_last(itr->second.UniqueCodes.end()); uc_itr!=uc_last; ++uc_itr)
      os<<"  "<<uc_itr->first<<SKYAI_MODULE_PORT_DELIMITER<<uc_itr->second<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

