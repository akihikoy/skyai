//-------------------------------------------------------------------------------------------
/*! \file    base.h
    \brief   libskyai - base unit (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.24, 2009-

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
#ifndef skyai_base_h
#define skyai_base_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
#include <lora/variable_space.h>
#include <climits>
#include <string>
#include <list>
#include <map>
#include <boost/function.hpp>
#include <skyai/module_manager.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
static const int SKYAI_CONNECTION_SIZE_MAX (INT_MAX);
//-------------------------------------------------------------------------------------------
static const char *SKYAI_DISABLED_FWD_PORT_NAME ("-");  //!< used to be a disabled forwarding signal port name
//-------------------------------------------------------------------------------------------
static const char *SKYAI_MODULE_NAME_DELIMITER ("-");
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief define the interface of ports */
class TPortInterface
//===========================================================================================
{
public:

  TPortInterface (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size)
    : outer_base_              (v_outer_base),
      name_                    (v_name),
      max_connection_size_     (v_max_connection_size)
    {}

  virtual ~TPortInterface() {}

  /*!\brief Connect v_port to this port (return true if successful) */
  virtual bool Connect (TPortInterface &v_port) = 0;

  /*!\brief Disconnect the connection with the port specified by port_ptr (disconnected:true) */
  virtual bool Disconnect (const TPortInterface *port_ptr) = 0;

  virtual int ConnectionSize() const = 0;

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  virtual void ForEachConnectedPort (boost::function<bool(TPortInterface*)> f) = 0;

  /*!\brief for each connected port, apply the function f
      \note if the return of the function f is false, the iteration is finished immidiately,
            else the iteration is continued.  */
  virtual void ForEachConnectedPort (boost::function<bool(const TPortInterface*)> f) const = 0;

  const TModuleInterface& OuterBase() const {return outer_base_;}
  const std::string& Name() const {return name_;}
  const std::string  UniqueCode() const;
  const int&         MaxConnectionSize() const {return max_connection_size_;}

protected:

  const TModuleInterface &outer_base_;

  const std::string  name_;
  int max_connection_size_;

};
//-------------------------------------------------------------------------------------------

class TOutPortInterface : public TPortInterface
{
public:

  TOutPortInterface (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX) :
      TPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}

protected:
};
//-------------------------------------------------------------------------------------------

class TInPortInterface : public TPortInterface
{
public:

  TInPortInterface (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=1) :
      TPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}

protected:
};
//-------------------------------------------------------------------------------------------

class TSignalPortInterface : public TPortInterface
{
public:

  TSignalPortInterface (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX) :
      TPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}

protected:
};
//-------------------------------------------------------------------------------------------

class TSlotPortInterface : public TPortInterface
{
public:

  TSlotPortInterface (const TModuleInterface &v_outer_base, const std::string v_name, int v_max_connection_size=SKYAI_CONNECTION_SIZE_MAX)
      : TPortInterface (v_outer_base, v_name, v_max_connection_size)
    {}

  /*!\brief return the reference to the forwarding-signal-port
      that is emitted after the slot (exec_) is executed */
  virtual TSignalPortInterface&  ForwardingSinalPortBase() = 0;

protected:
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
/*!\brief This macro generates basic member functions about module name.
  Use at the public section of a module class.
  \note  This module-name string (ModuleName()) is referred from SKYAI_ADD_MODULE macro,
    and registered to TModuleManager with this name.
    TModuleManager generates an instance of the module from a module-name string.
    Thus, the module-name string should be unique.
    Basically, use string stringized from the module-class name.

  \note  <b>For a template module case</b>, because we cannot directoly register the template to TModuleManager,
    we (1) explicitly instantiate the template module with some types, (2) typedef them as unique names,
    and (3) register them into TModuleManager with their unique name given by typedef.
    In such a case, we have to specialize ModuleName() for template instances.
    Actually, use SKYAI_INSTANTIATE_TEMPLATE_MODULE_N and SKYAI_SPECIALIZE_TEMPLATE_MODULE_N macros
    whith automatically do the instantiating and specializing processes. */
#define SKYAI_MODULE_NAMES(x_module_name)                                           \
  static std::string    ModuleName()  {return std::string(#x_module_name);}         \
  virtual std::string   InheritedModuleName() const {return ModuleName();}
//-------------------------------------------------------------------------------------------

//!\TODO  Automate to generate the following macros (up to about 10)

/*!\brief This macro specializes and instantiates a template module (for 1 template argument)
    \note Use in the namespace loco_rabbits.
    \note This macro gives the instance an unique name (typedef), specializes ModuleName(),
          and explicitly instantiates the template module.
    \note The unique name of a module Module1 instantiated with Type1 is Module1_Type1,
          and ModuleName() gives "Module1_Type1"  */
#define SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(x_module_name,x_type_1)     \
  typedef x_module_name<x_type_1> x_module_name##_##x_type_1;           \
  template<> std::string x_module_name<x_type_1>::ModuleName()          \
      {return std::string(#x_module_name "_" #x_type_1);}               \
  template class x_module_name<x_type_1>;
/*!\brief This macro specializes and instantiates a template module (for 2 template argument) */
#define SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(x_module_name,x_type_1,x_type_2)         \
  typedef x_module_name<x_type_1,x_type_2> x_module_name##_##x_type_1##_##x_type_2;  \
  template<> std::string x_module_name<x_type_1,x_type_2>::ModuleName()              \
      {return std::string(#x_module_name "_" #x_type_1 "_" #x_type_2);}              \
  template class x_module_name<x_type_1,x_type_2>;
/*!\brief This macro specializes and instantiates a template module (for 3 template argument) */
#define SKYAI_INSTANTIATE_TEMPLATE_MODULE_3(x_module_name,x_type_1,x_type_2,x_type_3)                 \
  typedef x_module_name<x_type_1,x_type_2,x_type_3> x_module_name##_##x_type_1##x_type_2##x_type_3;   \
  template<> std::string x_module_name<x_type_1,x_type_2,x_type_3>::ModuleName()                      \
      {return std::string(#x_module_name "_" #x_type_1 "_" #x_type_2 "_" #x_type_3);}                 \
  template class x_module_name<x_type_1,x_type_2,x_type_3>;

/*!\brief This macro generates a declaration of specializing a template module (for 1 template argument)
    \note Use in the namespace loco_rabbits.
          Use in a header file.
    \warning Even if you do not use these macros, you can compile the code.
          However, the compiler fails in specializing a module that is inherited from
          the template module.  */
#define SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(x_module_name,x_type_1)  \
  template<> std::string x_module_name<x_type_1>::ModuleName();
/*!\brief This macro generates a declaration of specializing a template module (for 2 template argument)  */
#define SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(x_module_name,x_type_1,x_type_2)  \
  template<> std::string x_module_name<x_type_1,x_type_2>::ModuleName();
/*!\brief This macro generates a declaration of specializing a template module (for 3 template argument)  */
#define SKYAI_SPECIALIZE_TEMPLATE_MODULE_3(x_module_name,x_type_1,x_type_2,x_type_3)  \
  template<> std::string x_module_name<x_type_1,x_type_2,x_type_3>::ModuleName();

//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Base class of every module (every skyai module is required to be inherited from this class) */
class TModuleInterface
//===========================================================================================
{
public:

  template <typename t_port>
  struct TPortSet
    {
      typedef std::map<std::string, t_port> type;
    };

  enum TModuleMode {mmNormal=0, mmDebug};

  static std::string   ModuleName()  {return std::string("TModuleInterface");}
  virtual std::string   InheritedModuleName() const {return ModuleName();}

  std::string InstanceName() const
    {return instance_name_;}
  std::string ModuleUniqueCode() const
    {return InheritedModuleName() + SKYAI_MODULE_NAME_DELIMITER + instance_name_;}

  var_space::TVariable& ParamBoxConfig()  {return param_box_config_;}
  const var_space::TVariable& ParamBoxConfig() const {return param_box_config_;}

  var_space::TVariable& ParamBoxMemory()  {return param_box_memory_;}
  const var_space::TVariable& ParamBoxMemory() const {return param_box_memory_;}

  // virtual void SaveConfigTo (TSettingFile &sf) const {param_box_config_.SaveTo(sf);}
  // virtual void SaveConfigTo (const std::string &filename, bool add_suffix=false) const {FIXME("!");}
  // virtual void LoadConfigFrom (TSettingFile &sf)  {param_box_config_.LoadFrom(sf);}
  // virtual void LoadConfigFrom (const std::string &filename, bool add_suffix=false) {FIXME("!");}

  // virtual void SaveMemoryTo (TSettingFile &sf) const {param_box_memory_.SaveTo(sf);}
  // virtual void SaveMemoryTo (const std::string &filename, bool add_suffix=false) const {FIXME("!");}
  // virtual void LoadMemoryFrom (TSettingFile &sf)  {param_box_memory_.LoadFrom(sf);}
  // virtual void LoadMemoryFrom (const std::string &filename, bool add_suffix=false) {FIXME("!");}

  /*!\brief a constructor of TModuleInterface class.
      \note any subclasses of the TModuleInterface that are registered to TModuleManager
          should have the same constructor form as TModuleInterface */
  TModuleInterface (const std::string &v_instance_name)
      : instance_name_         (v_instance_name),
        param_box_config_      (var_space::VariableSpace()),
        param_box_memory_      (var_space::VariableSpace()),
        module_mode_           (mmNormal),
        debug_stream_          (NULL)
    {}

  virtual ~TModuleInterface(void) {}


  // const std::string GetConfigStr (const std::string &param_name)        {return param_box_config_.GetStr(param_name);}
  // const TStringListEx GetConfigStrList (const std::string &param_name)  {return param_box_config_.GetStrList(param_name);}
  // void SetConfigStr (const std::string &param_name, const std::string &str)             {param_box_config_.SetStr(param_name,str);}
  // void SetConfigStrList (const std::string &param_name, const TStringListEx &str_list)  {param_box_config_.SetStrList(param_name,str_list);}

  // const std::string GetMemoryStr (const std::string &param_name)        {return param_box_memory_.GetStr(param_name);}
  // const TStringListEx GetMemoryStrList (const std::string &param_name)  {return param_box_memory_.GetStrList(param_name);}
  // void SetMemoryStr (const std::string &param_name, const std::string &str)             {param_box_memory_.SetStr(param_name,str);}
  // void SetMemoryStrList (const std::string &param_name, const TStringListEx &str_list)  {param_box_memory_.SetStrList(param_name,str_list);}


  const TModuleMode&  ModuleMode () const {return module_mode_;}
  void  SetModuleMode (const TModuleMode &mm)  {module_mode_= mm;}

  std::ostream& DebugStream () const {return (debug_stream_==NULL) ? std::cerr : *debug_stream_;}
  void SetDebugStream (std::ostream &os) {debug_stream_= &os;}

  TPortInterface* PortPtr (const std::string &v_name)
    {
#define X_SEARCH_PORT(x_type,x_set_name)                                              \
    {                                                                                 \
      TPortSet<x_type*>::type::const_iterator item= x_set_name.find(v_name);          \
      if(item!=x_set_name.end())  return  (item->second);                             \
    }
      X_SEARCH_PORT(TOutPortInterface        , out_ports_    )
      X_SEARCH_PORT(TInPortInterface         , in_ports_     )
      X_SEARCH_PORT(TSignalPortInterface     , signal_ports_ )
      X_SEARCH_PORT(TSlotPortInterface       , slot_ports_   )
#undef X_SEARCH_PORT
      return NULL;
    }

  TPortInterface& Port (const std::string &v_name)
    {
      TPortInterface *p= PortPtr(v_name);
      if(p)  return *p;
      LERROR("module "<<ModuleUniqueCode()<<" does not have the specified port "<<v_name);
      lexit(df); return dummy_return<TPortInterface>::value();
    }

#define X_SEARCH_PORT(x_type,x_set_name,x_error_msg)                               \
    TPortSet<x_type*>::type::const_iterator item= x_set_name.find(v_name);         \
    if(item!=x_set_name.end())  return *(item->second);                            \
    LERROR(x_error_msg);                                                           \
    lexit(df); return dummy_return<x_type>::value();

  //!\brief access an out-port by its name
  TOutPortInterface& OutPort (const std::string &v_name)
    {
      X_SEARCH_PORT(TOutPortInterface, out_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified out-port "<<v_name)
    }
  //!\brief access an out-port (const) by its name
  const TOutPortInterface& OutPort (const std::string &v_name) const
    {
      X_SEARCH_PORT(TOutPortInterface, out_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified out-port "<<v_name)
    }

  //!\brief access an in-port by its name
  TInPortInterface& InPort (const std::string &v_name)
    {
      X_SEARCH_PORT(TInPortInterface, in_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified in-port "<<v_name)
    }
  //!\brief access an in-port (const) by its name
  const TInPortInterface& InPort (const std::string &v_name) const
    {
      X_SEARCH_PORT(TInPortInterface, in_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified in-port "<<v_name)
    }

  //!\brief access a signal-port by its name
  TSignalPortInterface& SignalPort (const std::string &v_name)
    {
      X_SEARCH_PORT(TSignalPortInterface, signal_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified signal-port "<<v_name)
    }
  //!\brief access a signal-port (const) by its name
  const TSignalPortInterface& SignalPort (const std::string &v_name) const
    {
      X_SEARCH_PORT(TSignalPortInterface, signal_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified signal-port "<<v_name)
    }

  //!\brief access a slot-port by its name
  TSlotPortInterface& SlotPort (const std::string &v_name)
    {
      X_SEARCH_PORT(TSlotPortInterface, slot_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified slot-port "<<v_name)
    }
  //!\brief access a slot-port (const) by its name
  const TSlotPortInterface& SlotPort (const std::string &v_name) const
    {
      X_SEARCH_PORT(TSlotPortInterface, slot_ports_,
          "module "<<ModuleUniqueCode()<<" does not have the specified slot-port "<<v_name)
    }

#undef X_SEARCH_PORT


  // accessors to iterators

  TPortSet<TOutPortInterface*>::type::iterator       OutPortBegin() {return out_ports_.begin();}
  TPortSet<TOutPortInterface*>::type::iterator       OutPortEnd()   {return out_ports_.end();}
  TPortSet<TOutPortInterface*>::type::const_iterator OutPortBegin() const {return out_ports_.begin();}
  TPortSet<TOutPortInterface*>::type::const_iterator OutPortEnd()   const {return out_ports_.end();}

  TPortSet<TInPortInterface*>::type::iterator       InPortBegin() {return in_ports_.begin();}
  TPortSet<TInPortInterface*>::type::iterator       InPortEnd()   {return in_ports_.end();}
  TPortSet<TInPortInterface*>::type::const_iterator InPortBegin() const {return in_ports_.begin();}
  TPortSet<TInPortInterface*>::type::const_iterator InPortEnd()   const {return in_ports_.end();}

  TPortSet<TSignalPortInterface*>::type::iterator       SignalPortBegin() {return signal_ports_.begin();}
  TPortSet<TSignalPortInterface*>::type::iterator       SignalPortEnd()   {return signal_ports_.end();}
  TPortSet<TSignalPortInterface*>::type::const_iterator SignalPortBegin() const {return signal_ports_.begin();}
  TPortSet<TSignalPortInterface*>::type::const_iterator SignalPortEnd()   const {return signal_ports_.end();}

  TPortSet<TSlotPortInterface*>::type::iterator       SlotPortBegin() {return slot_ports_.begin();}
  TPortSet<TSlotPortInterface*>::type::iterator       SlotPortEnd()   {return slot_ports_.end();}
  TPortSet<TSlotPortInterface*>::type::const_iterator SlotPortBegin() const {return slot_ports_.begin();}
  TPortSet<TSlotPortInterface*>::type::const_iterator SlotPortEnd()   const {return slot_ports_.end();}


  struct TShowConf
    {
      bool ShowInstanceName       ;
      bool ShowUniqueCode         ;
      bool ShowParamsConfig       ;
      bool ShowParamsMemory       ;
      bool ShowPorts              ;
      bool ShowPortsConnection    ;
      bool ShowPortsMaxConnection ;
      TShowConf()
        :
          ShowInstanceName        (true),
          ShowUniqueCode          (false),
          ShowParamsConfig        (true),
          ShowParamsMemory        (false),
          ShowPorts               (true),
          ShowPortsConnection     (true),
          ShowPortsMaxConnection  (true)
        {}
    };
  static void ParseShowConfOption (const std::string &option, TShowConf &conf);
  void ShowModule (const TShowConf &conf=TShowConf(), std::ostream &os=std::cout) const;
  void ShowModule (const std::string &option="", std::ostream &os=std::cout) const;

protected:

  /*!\brief parameter box which has links to the configuration parameters of this module */
  var_space::TVariableMap&  param_box_config_map ()  {return param_box_config_.SetMemberMap();}

  /*!\brief parameter box which has links to the learning parameters of this module */
  var_space::TVariableMap&  param_box_memory_map ()  {return param_box_memory_.SetMemberMap();}

#define PORT_EXISTING_CHECK(x_port) \
    do{if (PortPtr(x_port.Name())!=NULL)  \
      {LERROR("in module "<<ModuleUniqueCode()<<", port "<<x_port.Name()<<" already exists"); lexit(df);}}while(0)
  void add_out_port (TOutPortInterface &v_port)
    {
      PORT_EXISTING_CHECK(v_port);
      out_ports_[v_port.Name()]= &v_port;
    }
  void add_in_port (TInPortInterface &v_port)
    {
      PORT_EXISTING_CHECK(v_port);
      in_ports_[v_port.Name()]= &v_port;
    }

  void add_signal_port (TSignalPortInterface &v_port)
    {
      PORT_EXISTING_CHECK(v_port);
      signal_ports_[v_port.Name()]= &v_port;
    }
  void add_slot_port (TSlotPortInterface &v_port)
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

private:

  /*! forbid to copy (every ports and parameter boxes should not be copied) */
  const TModuleInterface& operator= (const TModuleInterface &);


  const std::string instance_name_;

  /*!\brief parameter box which has links to the configuration parameters of this module */
  var_space::TVariable param_box_config_;

  /*!\brief parameter box which has links to the learning parameters of this module */
  var_space::TVariable param_box_memory_;

  TModuleMode   module_mode_;
  std::ostream  *debug_stream_;

  /*! the set of out-ports. */
  TPortSet<TOutPortInterface*>::type out_ports_;

  /*! the set of in-ports. */
  TPortSet<TInPortInterface*>::type  in_ports_;

  /*! the set of signal-ports. */
  TPortSet<TSignalPortInterface*>::type  signal_ports_;

  /*! the set of slot-ports. */
  TPortSet<TSlotPortInterface*>::type  slot_ports_;

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*!\brief agent class that holds all instances of the modules */
class TAgent
//===========================================================================================
{
public:

  /*! Type of module set (map) whose key is module.InstanceName() */
  typedef std::map<std::string, TModuleInterface*>  TModuleSet;

  //! clear all modules (memories are freed)
  void Clear();

  TAgent () {}

  virtual ~TAgent()
    {
      Clear();
    }

  TModuleInterface& Module (const std::string &module_name);
  const TModuleInterface& Module (const std::string &module_name) const;

  template <typename t_module>
  t_module& ModuleAs (const std::string &module_name);

  template <typename t_module>
  const t_module& ModuleAs (const std::string &module_name) const;

  //! create a module of the specified type
  template <typename t_module>
  t_module&  AddModule (const std::string &v_instance_name);

  //! create a module whose type is specified by the string v_module_class
  TModuleInterface&  AddModule (const std::string &v_module_class, const std::string &v_instance_name);


  bool ConnectIO(
      TModuleInterface &start_module, const std::string &out_port_name,
      TModuleInterface &end_module,   const std::string &in_port_name);
  bool ConnectEvent(
      TModuleInterface &start_module, const std::string &signal_port_name,
      TModuleInterface &end_module,   const std::string &slot_port_name);

  /*! add edge.  the port types are automatically determined */
  bool Connect(
      TModuleInterface &start_module, const std::string &start_port_name,
      TModuleInterface &end_module,   const std::string &end_port_name);

  /*! add edge. the modules are indicated by names. the port types are automatically determined */
  bool Connect(
      const std::string &start_module_name, const std::string &start_port_name,
      const std::string &end_module_name,   const std::string &end_port_name);

  /*! \todo implement Disconnect */
  void Disconnect(
      TModuleInterface &start_module, const std::string &start_port_name,
      TModuleInterface &end_module,   const std::string &end_port_name);

  void SetAllModuleMode (const TModuleInterface::TModuleMode &mm);
  void SetDebugStream (std::ostream &os);


  /*!\brief for each module, apply the function f */
  void ForEachModule (boost::function<bool(TModuleInterface* module)> f);

  /*!\brief for each module, apply the function f */
  void ForEachModule (boost::function<bool(const TModuleInterface* module)> f) const;

  /*!\brief for each connected port, apply the function f */
  void ForEachConnection (boost::function<bool(TPortInterface* from_port_ptr, TPortInterface* to_port_ptr)> f);

  /*!\brief for each connected port, apply the function f */
  void ForEachConnection (boost::function<bool(const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)> f) const;

  void ShowAllModules (const std::string &format="", std::ostream &os=std::cerr) const;

  //! DEBUG FUNCTION
  void ShowAllConnections (std::ostream &os=std::cerr) const;

  //! TEST (export to a graph description language)
  void ExportToDOT (std::ostream &os=std::cout) const;


protected:

  //! forbit to call a copy constructor
  TAgent (const TAgent&);
  const TAgent& operator=(const TAgent&);

  TModuleSet    modules_;

  inline TModuleInterface* find_module (const std::string &module_name, bool error=false) const;

};
//-------------------------------------------------------------------------------------------

template <typename t_module>
t_module& TAgent::ModuleAs (const std::string &module_name)
{
  TModuleInterface *m (find_module(module_name,/*error=*/true));
  if (m==NULL)  {lexit(df); return dummy_return<t_module&>::value();}
  if (t_module *m2= dynamic_cast<t_module*>(m))  return *m2;
  LERROR("module "<<module_name<<" is not an instance of "<<t_module::ModuleName()<<".");
  lexit(df); return dummy_return<t_module&>::value();
}
//-------------------------------------------------------------------------------------------

template <typename t_module>
const t_module& TAgent::ModuleAs (const std::string &module_name) const
{
  TModuleInterface *m (find_module(module_name,/*error=*/true));
  if (m==NULL)  {lexit(df); return dummy_return<t_module&>::value();}
  if (t_module *m2= dynamic_cast<t_module*>(m))  return *m2;
  LERROR("module "<<module_name<<" is not an instance of "<<t_module::ModuleName()<<".");
  lexit(df); return dummy_return<t_module&>::value();
}
//-------------------------------------------------------------------------------------------

//! create a module of the specified type
template <typename t_module>
t_module&  TAgent::AddModule (const std::string &v_instance_name)
{
  if (modules_.find(v_instance_name)!=modules_.end())
  {
    LERROR("module "<<v_instance_name<<" is already registered");
    lexit(df);
  }
  t_module *p= new t_module (v_instance_name);
  modules_[v_instance_name]= p;
  return *p;
}
//-------------------------------------------------------------------------------------------

inline TModuleInterface* TAgent::find_module (const std::string &module_name, bool error) const
{
  TModuleSet::const_iterator itr= modules_.find(module_name);
  if(itr==modules_.end())
  {
    LERROR("module "<<module_name<<" is not found.");
    return NULL;
  }
  return itr->second;
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_base_h
//-------------------------------------------------------------------------------------------
