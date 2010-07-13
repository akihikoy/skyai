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
// forward declaration:
namespace boost {namespace filesystem {
  struct path_traits;
  template<class String, class Traits> class basic_path;
  typedef basic_path< std::string, path_traits > path;
}}
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
static const int SKYAI_CONNECTION_SIZE_MAX (INT_MAX);
//-------------------------------------------------------------------------------------------
static const char * const SKYAI_DISABLED_FWD_PORT_NAME ("-");  //!< used to be a disabled forwarding signal port name
//-------------------------------------------------------------------------------------------
static const char * const SKYAI_MODULE_NAME_DELIMITER ("-");
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


class  TAgent;


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

  /*!\brief a constructor of TModuleInterface class.
      \note any subclasses of the TModuleInterface that are registered to TModuleManager
          should have the same constructor form as TModuleInterface */
  TModuleInterface (const std::string &v_instance_name)
      : instance_name_         (v_instance_name),
        pagent_                (NULL),
        param_box_config_      (var_space::VariableSpace()),
        param_box_memory_      (var_space::VariableSpace()),
        module_mode_           (mmNormal),
        debug_stream_          (NULL)
    {}

  virtual ~TModuleInterface(void) {}

  const TModuleMode&  ModuleMode () const {return module_mode_;}
  void  SetModuleMode (const TModuleMode &mm)  {module_mode_= mm;}

  std::ostream& DebugStream () const {return (debug_stream_==NULL) ? std::cerr : *debug_stream_;}
  void SetDebugStream (std::ostream &os) {debug_stream_= &os;}

  TPortInterface* PortPtr (const std::string &v_name);
  TPortInterface& Port (const std::string &v_name);

  TOutPortInterface& OutPort (const std::string &v_name);                   //!<\brief access an out-port by its name
  const TOutPortInterface& OutPort (const std::string &v_name) const;       //!<\brief access an out-port (const) by its name
  TInPortInterface& InPort (const std::string &v_name);                     //!<\brief access an in-port by its name
  const TInPortInterface& InPort (const std::string &v_name) const;         //!<\brief access an in-port (const) by its name
  TSignalPortInterface& SignalPort (const std::string &v_name);             //!<\brief access a signal-port by its name
  const TSignalPortInterface& SignalPort (const std::string &v_name) const; //!<\brief access a signal-port (const) by its name
  TSlotPortInterface& SlotPort (const std::string &v_name);                 //!<\brief access a slot-port by its name
  const TSlotPortInterface& SlotPort (const std::string &v_name) const;     //!<\brief access a slot-port (const) by its name


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


  //!\brief return the reference to the host agent
  const TAgent&  Agent() const {LASSERT(pagent_); return *pagent_;}
  void  SetAgent(const TAgent &agent)  {pagent_= &agent;}

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

  void add_out_port (TOutPortInterface &v_port);
  void add_in_port (TInPortInterface &v_port);
  void add_signal_port (TSignalPortInterface &v_port);
  void add_slot_port (TSlotPortInterface &v_port);

private:

  /*! forbid to copy (every ports and parameter boxes should not be copied) */
  const TModuleInterface& operator= (const TModuleInterface &);


  const std::string instance_name_;
  const TAgent *pagent_;  //!< pointer to the host agent

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
/*!\brief TAgent's Configurations  */
struct TAgentConfigurations
//===========================================================================================
{
  std::string               DataDir;   //!< directoly path to save data (boost::filesystem's portable file-path format)

  TAgentConfigurations(var_space::TVariableMap &mmap)
      : DataDir ("nonexistent_dir")
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( DataDir   );
      #undef ADD
    }
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

  //! clear all modules and path_list_ (memories are freed)
  void Clear();

  TAgent ()
      : param_box_config_  (var_space::VariableSpace()),
        conf_              (param_box_config_.SetMemberMap()),
        path_list_         (NULL)
    {}

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


  const TAgentConfigurations&  Config() const {return conf_;}
  TAgentConfigurations&  SetConfig()  {return conf_;}

  var_space::TVariable& ParamBoxConfig()  {return param_box_config_;}
  const var_space::TVariable& ParamBoxConfig() const {return param_box_config_;}


  /*!\brief for each module, apply the function f */
  void ForEachModule (boost::function<bool(TModuleInterface* module)> f);

  /*!\brief for each module, apply the function f */
  void ForEachModule (boost::function<bool(const TModuleInterface* module)> f) const;

  /*!\brief for each connected port, apply the function f */
  void ForEachConnection (boost::function<bool(TPortInterface* from_port_ptr, TPortInterface* to_port_ptr)> f);

  /*!\brief for each connected port, apply the function f */
  void ForEachConnection (boost::function<bool(const TPortInterface* from_port_ptr, const TPortInterface* to_port_ptr)> f) const;


  /*!\brief load modules, connections, configurations from the file [filename] (native path format)
      \param [in,out]included_list  :  included full-path (native) list
      \note  If you use include_once for multiple LoadFromFile, the same included_list should be specified */
  bool LoadFromFile (const std::string &filename, bool *is_last=NULL, std::list<std::string> *included_list=NULL);

  /*!\brief save modules, connections, configurations to the file [filename] (native path format) */
  bool SaveToFile (const std::string &filename) const;


  /*!\brief add dir_name (native format path) to the path-list */
  void AddPath (const std::string &dir_name);

  /*!\brief add dir_list (list of native format path) to the path-list */
  void AddPathList (const std::list<std::string> &dir_list);


  /*!\brief search filename from the path-list, return the native path */
  std::string SearchFileName (const std::string &filename) const;

  /*!\brief return a complete native path to filename which is a relative path from conf_.DataDir */
  std::string GetDataFileName (const std::string &filename) const;


  void SetAllModuleMode (const TModuleInterface::TModuleMode &mm);
  void SetDebugStream (std::ostream &os);

  void ShowAllModules (const std::string &format="", std::ostream &os=std::cerr) const;

  //! DEBUG FUNCTION
  void ShowAllConnections (std::ostream &os=std::cerr) const;

  /*! TEST: export module structure to a graph description language.
        Draw by graphviz - fdp (e.g. fdp -Tsvg FOO.dot -o BAR.svg) */
  void ExportToDOT (std::ostream &os=std::cout) const;


protected:

  //! forbit to call a copy constructor
  TAgent (const TAgent&);
  const TAgent& operator=(const TAgent&);

  TModuleSet    modules_;

  /*!\brief parameter box which has links to the configuration parameters of the agent */
  var_space::TVariable param_box_config_;
  TAgentConfigurations conf_;

  std::list<boost::filesystem::path>  *path_list_;

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
