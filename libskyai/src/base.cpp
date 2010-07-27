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

std::string TModuleInterface::SearchPortByPtr(const TPortInterface *port_ptr) const
{
#define X_SEARCH_PORT(x_type,x_port)  \
  for (TPortSet<x_type*>::type::const_iterator itr(x_port.begin()),last(x_port.end()); itr!=last; ++itr)  \
    if (itr->second==port_ptr)  return itr->first;
  X_SEARCH_PORT(TOutPortInterface    , out_ports_   );
  X_SEARCH_PORT(TInPortInterface     , in_ports_    );
  X_SEARCH_PORT(TSignalPortInterface , signal_ports_);
  X_SEARCH_PORT(TSlotPortInterface   , slot_ports_  );
#undef X_SEARCH_PORT
  return "";
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
  add_out_port(v_port,v_port.Name());
}
/*protected*/void TModuleInterface::add_in_port     (TInPortInterface     &v_port)
{
  add_in_port(v_port,v_port.Name());
}
/*protected*/void TModuleInterface::add_signal_port (TSignalPortInterface &v_port)
{
  add_signal_port(v_port,v_port.Name());
}
/*protected*/void TModuleInterface::add_slot_port   (TSlotPortInterface   &v_port)
{
  add_slot_port(v_port,v_port.Name());
  // [-- for signal forwarding
  if (v_port.ForwardingSinalPortBase().Name() != SKYAI_DISABLED_FWD_PORT_NAME)
  {
    add_signal_port(v_port.ForwardingSinalPortBase(), v_port.ForwardingSinalPortBase().Name());
  }
  // for signal forwarding  --]
}

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
TModuleInterface* TCompositeModule::SearchModule (const std::string &module_name, int max_depth)
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
const TModuleInterface* TCompositeModule::SearchModule (const std::string &module_name, int max_depth) const
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

bool TCompositeModule::SubConnectIO(
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

bool TCompositeModule::SubConnectEvent(
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
bool TCompositeModule::SubConnect(
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
bool TCompositeModule::SubConnect(
    const std::string &start_module_name, const std::string &start_port_name,
    const std::string &end_module_name,   const std::string &end_port_name)
{
  TModuleSet::iterator itr_start_module (sub_modules_.find(TModuleCell(start_module_name)));
  if (itr_start_module==sub_modules_.end())  {LERROR(start_module_name<<" does not exist"); lexit(df);}
  TModuleSet::iterator itr_end_module (sub_modules_.find(TModuleCell(end_module_name)));
  if (itr_end_module==sub_modules_.end())  {LERROR(end_module_name<<" does not exist"); lexit(df);}
  TPortInterface &start (itr_start_module->Ptr->Port(start_port_name));
  TPortInterface &end (itr_end_module->Ptr->Port(end_port_name));

  if(start.Connect(end) && end.Connect(start))
    return true;
  return false;
}
//-------------------------------------------------------------------------------------------

//! \todo implement Disconnect
void TCompositeModule::SubDisconnect(
    TModuleInterface &start_module, const std::string &start_port_name,
    TModuleInterface &end_module,   const std::string &end_port_name)
{
  FIXME("TCompositeModule::SubDisconnect is not implemented yet..orz..");
}
//-------------------------------------------------------------------------------------------

//! \todo implement Disconnect
void TCompositeModule::SubDisconnect(
    const std::string &start_module_name, const std::string &start_port_name,
    const std::string &end_module_name,   const std::string &end_port_name)
{
  FIXME("TCompositeModule::SubDisconnect is not implemented yet..orz..");
}
//-------------------------------------------------------------------------------------------


/*!\brief for each module, apply the function f */
void TCompositeModule::ForEachSubModule (boost::function<bool(TModuleInterface* module)> f)
{
  // for each module in sub_modules_ :
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
    if (!f(mod_itr->Ptr))  break;
}
//-------------------------------------------------------------------------------------------

/*!\brief for each module, apply the function f */
void TCompositeModule::ForEachSubModule (boost::function<bool(const TModuleInterface* module)> f) const
{
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
    if (!f(mod_itr->Ptr))  break;
}
//-------------------------------------------------------------------------------------------

/*!\brief for each module, apply the function f */
void TCompositeModule::ForEachSubModuleCell (boost::function<bool(const TModuleCell &mcell)> f) const
{
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
    if (!f(*mod_itr))  break;
}
//-------------------------------------------------------------------------------------------

/*!\brief for each connected port, apply the function f */
void TCompositeModule::ForEachSubConnection (boost::function<bool(TPortInterface* from_port_ptr, TPortInterface* to_port_ptr)> f)
{
  // for each module in sub_modules_ :
  for(TModuleSet::iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::iterator
                op_itr(mod_itr->Ptr->OutPortBegin()), opiend(mod_itr->Ptr->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      // for each port connected to op_itr :
      // NOTE :  from_port_ptr is op_itr->second,  to_port_ptr is _1
      op_itr->second->ForEachConnectedPort (boost::bind(f, op_itr->second, _1));
    }

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::iterator
                sp_itr(mod_itr->Ptr->SlotPortBegin()), spiend(mod_itr->Ptr->SlotPortEnd());
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
void TCompositeModule::ForEachSubConnection (boost::function<bool(const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)> f) const
{
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // for each out-port :
    for (TModuleInterface::TPortSet<TOutPortInterface*>::type::const_iterator
                op_itr(mod_itr->Ptr->OutPortBegin()), opiend(mod_itr->Ptr->OutPortEnd());
            op_itr!=opiend;  ++op_itr)
    {
      // for each port connected to op_itr :
      // NOTE :  from_port_ptr is op_itr->second,  to_port_ptr is _1
      op_itr->second->ForEachConnectedPort (boost::bind(f, op_itr->second, _1));
    }

    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->Ptr->SlotPortBegin()), spiend(mod_itr->Ptr->SlotPortEnd());
            sp_itr!=spiend;  ++sp_itr)
    {
      // for each port connected to sp_itr :
      // NOTE :  from_port_ptr is _1,  to_port_ptr is sp_itr->second
      sp_itr->second->ForEachConnectedPort (boost::bind(f, _1, sp_itr->second));
    }
  }
}
//-------------------------------------------------------------------------------------------


/*!\brief export a port sub_module_name.port_name as export_name */
bool TCompositeModule::ExportPort (const std::string &sub_module_name, const std::string &port_name, const std::string &export_name)
{
  export_list_.push_back (std::pair<std::string,std::string>(sub_module_name+"."+port_name, export_name));

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
  export_list_.push_back (std::pair<std::string,std::string>(sub_module_name+".config."+param_name, export_name));

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
  export_list_.push_back (std::pair<std::string,std::string>(sub_module_name+".memory."+param_name, export_name));

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

void TCompositeModule::SetDebugStream (std::ostream &os)
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
    mod_itr->Ptr->ShowModule (show_conf, os);
}
//-------------------------------------------------------------------------------------------

static bool print_connection (std::ostream *os, const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)
{
  if (from_port_ptr && to_port_ptr)
    *os<<"connect "<<from_port_ptr->UniqueCode()
        <<"  ---->  "<<to_port_ptr->UniqueCode()<<std::endl;
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
void TCompositeModule::ExportToDOT (std::ostream &os) const
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

  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
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
  // for each module in sub_modules_ :
  for(TModuleSet::const_iterator mod_itr(sub_modules_.begin()), miend(sub_modules_.end()); mod_itr!=miend; ++mod_itr)
  {
    // for each slot-port :
    for (TModuleInterface::TPortSet<TSlotPortInterface*>::type::const_iterator
                sp_itr(mod_itr->Ptr->SlotPortBegin()), spiend(mod_itr->Ptr->SlotPortEnd());
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
  if(GeneratorExists(cmodule_name))  {LERROR(cmodule_name<<" already exists.");  return false;}
  generators_[cmodule_name]= generator;
  cmodule_name_list_.push_back(cmodule_name);
// LDEBUG("#####ADDED GENERATOR: "<<cmodule_name
// <<"("<<generators_[cmodule_name].FileName<<":"<<generators_[cmodule_name].LineNum<<")"<<endl
// <<generators_[cmodule_name].Script<<"#####");
  return true;
}
//-------------------------------------------------------------------------------------------

bool TCompositeModuleGenerator::GeneratorExists(const std::string &cmodule_name) const
{
  std::map<std::string, TGeneratorInfo>::const_iterator  itr(generators_.find(cmodule_name));
  return itr!=generators_.end();
}
//-------------------------------------------------------------------------------------------

// NOTE: the following member function is defined in parser.cpp
// bool TCompositeModuleGenerator::Create(TCompositeModule &instance, const std::string &cmodule_name, const std::string &instance_name, bool no_export) const;
// bool TCompositeModuleGenerator::WriteToStream (std::ostream &os, const std::string &indent) const;


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
// bool TAgent::LoadFromFile (const std::string &filename, bool *is_last, std::list<std::string> *included_list);
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



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

