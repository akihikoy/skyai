//-------------------------------------------------------------------------------------------
/*! \file    base.h
    \brief   libskyai - base unit (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Aug.24, 2009-
\todo FIXME: reduce the dependencies: separate TCompositeModule,TAgent from base.h

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
#include <skyai/module_manager.h>
#include <lora/common.h>
#include <lora/string.h>
#include <lora/variable_space.h>
#include <lora/binary.h>
#include <climits>
#include <string>
#include <list>
#include <map>
#include <set>
#include <boost/function.hpp>
//-------------------------------------------------------------------------------------------
// forward declarations:
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
static const char * const SKYAI_MODULE_PORT_DELIMITER (".");
//-------------------------------------------------------------------------------------------
static const char * const SKYAI_EXT_STORAGE_DIR ("ext_sto");
//-------------------------------------------------------------------------------------------
/*!\note we define SKYAI_DEFAULT_AGENT_SCRIPT_EXT as a macro rather than a const variable
    so that "."SKYAI_DEFAULT_AGENT_SCRIPT_EXT can indicate ".agent" */
#define SKYAI_DEFAULT_AGENT_SCRIPT_EXT  "agent"
//-------------------------------------------------------------------------------------------
#ifndef SKYAI_DEFAULT_LIBRARY_EXT
#define SKYAI_DEFAULT_LIBRARY_EXT "so"
#endif
//-------------------------------------------------------------------------------------------

class  TPortInterface;
class  TModuleInterface;
class  TCompositeModule;
class  TAgent;

enum TPortKind {pkUnknown=0,pkOut,pkIn,pkSignal,pkSlot};

ENUM_STR_MAP_BEGIN(TPortKind)
  ENUM_STR_MAP_ADD(pkUnknown )
  ENUM_STR_MAP_ADD(pkOut     )
  ENUM_STR_MAP_ADD(pkIn      )
  ENUM_STR_MAP_ADD(pkSignal  )
  ENUM_STR_MAP_ADD(pkSlot    )
ENUM_STR_MAP_END  (TPortKind)
//-------------------------------------------------------------------------------------------

struct TPortInfo
{
  TPortKind         Kind;
  std::string       Name;
  TPortInterface    *Ptr;
  TModuleInterface  *OuterModule;
  TPortInfo() : Kind(pkUnknown), Name(""), Ptr(NULL), OuterModule(NULL) {}
};
struct TConstPortInfo
{
  TPortKind               Kind;
  std::string             Name;
  const TPortInterface    *Ptr;
  const TModuleInterface  *OuterModule;
  TConstPortInfo() : Kind(pkUnknown), Name(""), Ptr(NULL), OuterModule(NULL) {}
};
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
      max_connection_size_     (v_max_connection_size),
      active_counter_          (0),
      never_called_            (true)
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

  /*!\brief get the original name of the port
      \note a port of a module can be exported as a port of the module's parent_cmodule_.
            in this case, the name of the exported port is arbitrary, i.e. not equal to the OriginalName().  */
  const std::string& OriginalName() const {return name_;}

  const int& MaxConnectionSize() const {return max_connection_size_;}

  bool IsActive() const {LASSERT1op1(active_counter_,>=,0); return active_counter_>0;}

  bool NeverCalled() const {return never_called_;}
  void SetNeverCalled (bool nc)  {never_called_= nc;}

protected:

  const TModuleInterface &outer_base_;

  const std::string  name_;
  int max_connection_size_;

  mutable int active_counter_;  //!< if positive, the outer module should not be removed

  struct TActivate
    {
      const TPortInterface &Outer;
      TActivate(const TPortInterface &o) : Outer(o) {++Outer.active_counter_;}
      ~TActivate()  {--Outer.active_counter_;}
    };

  mutable bool never_called_;  //!< if true, this port has not been executed after generated

  inline void first_call_check() const;

private:

  TPortInterface(const TPortInterface&);
  const TPortInterface& operator=(const TPortInterface&);

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

  std::string GlobalUniqueCode() const;

  static std::string PortUniqueCode(const std::string &module_name, const std::string &port_name)
    {return module_name+SKYAI_MODULE_PORT_DELIMITER+port_name;}
  std::string PortUniqueCode(const std::string &port_name) const {return PortUniqueCode(InstanceName(), port_name);}

  std::string PortUniqueCode(const TPortInterface &port) const
    {TConstPortInfo pi; if(!SearchPortByPtr(&port,pi)) return ""; return PortUniqueCode(pi.Name);}

  static bool DecomposePortUniqueCode(const std::string &unique_code, std::string &module_name, std::string &port_name);

  var_space::TVariable& ParamBoxConfig()  {first_access_check(); return param_box_config_;}
  const var_space::TVariable& ParamBoxConfig() const {first_access_check(); return param_box_config_;}

  var_space::TVariable& ParamBoxMemory()  {first_access_check(); return param_box_memory_;}
  const var_space::TVariable& ParamBoxMemory() const {first_access_check(); return param_box_memory_;}

  /*!\brief a constructor of TModuleInterface class.
      \note any subclasses of the TModuleInterface that are registered to TModuleManager
          should have the same constructor form as TModuleInterface */
  TModuleInterface (const std::string &v_instance_name)
      : instance_name_         (v_instance_name),
        pagent_                (NULL),
        parent_cmodule_        (NULL),
        param_box_config_      (var_space::VariableSpace()),
        param_box_memory_      (var_space::VariableSpace()),
        module_mode_           (mmNormal),
        debug_stream_          (NULL),
        is_zombie_             (false),
        never_accessed_        (true)
    {}

  virtual ~TModuleInterface(void);

  const TModuleMode&  ModuleMode () const {return module_mode_;}
  virtual void  SetModuleMode (const TModuleMode &mm)  {module_mode_= mm;}

  std::ostream& DebugStream () const {return (debug_stream_==NULL) ? std::cerr : *debug_stream_;}
  virtual void SetDebugStream (std::ostream &os) {debug_stream_= &os;}


  TOutPortInterface*          OutPortPtr (const std::string &v_name);          //!<\brief access an out-port by its name
  const TOutPortInterface*    OutPortPtr (const std::string &v_name) const;    //!<\brief access an out-port (const) by its name
  TInPortInterface*           InPortPtr (const std::string &v_name);           //!<\brief access an in-port by its name
  const TInPortInterface*     InPortPtr (const std::string &v_name) const;     //!<\brief access an in-port (const) by its name
  TSignalPortInterface*       SignalPortPtr (const std::string &v_name);       //!<\brief access a signal-port by its name
  const TSignalPortInterface* SignalPortPtr (const std::string &v_name) const; //!<\brief access a signal-port (const) by its name
  TSlotPortInterface*         SlotPortPtr (const std::string &v_name);         //!<\brief access a slot-port by its name
  const TSlotPortInterface*   SlotPortPtr (const std::string &v_name) const;   //!<\brief access a slot-port (const) by its name

  TOutPortInterface&          OutPort (const std::string &v_name);          //!<\brief access an out-port by its name
  const TOutPortInterface&    OutPort (const std::string &v_name) const;    //!<\brief access an out-port (const) by its name
  TInPortInterface&           InPort (const std::string &v_name);           //!<\brief access an in-port by its name
  const TInPortInterface&     InPort (const std::string &v_name) const;     //!<\brief access an in-port (const) by its name
  TSignalPortInterface&       SignalPort (const std::string &v_name);       //!<\brief access a signal-port by its name
  const TSignalPortInterface& SignalPort (const std::string &v_name) const; //!<\brief access a signal-port (const) by its name
  TSlotPortInterface&         SlotPort (const std::string &v_name);         //!<\brief access a slot-port by its name
  const TSlotPortInterface&   SlotPort (const std::string &v_name) const;   //!<\brief access a slot-port (const) by its name

  TPortInterface*             PortPtr (const std::string &v_name);          //!<\brief access a port by its name
  const TPortInterface*       PortPtr (const std::string &v_name) const;    //!<\brief access a port by its name
  TPortInterface&             Port (const std::string &v_name);             //!<\brief access a port by its name
  const TPortInterface&       Port (const std::string &v_name) const;       //!<\brief access a port by its name

  bool  SearchPortByPtr (const TPortInterface *port_ptr, TConstPortInfo &info) const;
  bool  SearchPortByPtr (TPortInterface *port_ptr, TPortInfo &info);


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


  //!\brief return true if this module has some active ports
  bool HasActivePorts() const;

  //!\brief set true: the module should be removed, but impossible because some ports are active
  void SetZombie (bool z)  {is_zombie_=z;}

  //!\brief true: the module should be removed, but impossible because some ports are active
  bool IsZombie()  {return is_zombie_;}


  //!\brief return the reference to the host agent
  TAgent&        Agent()       {LASSERT(pagent_); return *pagent_;}
  const TAgent&  Agent() const {LASSERT(pagent_); return *pagent_;}
  void  SetAgent (TAgent &agent)  {pagent_= &agent;}

  //!\brief return the pointer to the host composite module
  TCompositeModule&        ParentCModule()       {LASSERT(parent_cmodule_); return *parent_cmodule_;}
  const TCompositeModule&  ParentCModule() const {LASSERT(parent_cmodule_); return *parent_cmodule_;}
  TCompositeModule*        ParentCModulePtr()       {return parent_cmodule_;}
  const TCompositeModule*  ParentCModulePtr() const {return parent_cmodule_;}
  void SetParentCModule (TCompositeModule *parent)  {parent_cmodule_= parent;}


  virtual void SetNeverAccessed (bool na);
  void SetLazyLoadFile (const std::string &file_name);

  bool ExecuteFunction (const std::string &func_name, const std::list<var_space::TLiteral> &argv, var_space::TLiteral *ret_val=NULL, bool ignore_export=false);


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

  std::string lazy_include_filename_;  //!< if not empty, the file is included in on_first_access


  /*!\brief parameter box which has links to the configuration parameters of this module */
  var_space::TVariableMap&  param_box_config_map ()  {return param_box_config_.SetMemberMap();}

  /*!\brief parameter box which has links to the learning parameters of this module */
  var_space::TVariableMap&  param_box_memory_map ()  {return param_box_memory_.SetMemberMap();}


  void add_out_port    (TOutPortInterface    &v_port, const std::string &v_name);
  void add_in_port     (TInPortInterface     &v_port, const std::string &v_name);
  void add_signal_port (TSignalPortInterface &v_port, const std::string &v_name);
  void add_slot_port   (TSlotPortInterface   &v_port, const std::string &v_name);  //!<\note this method does not add forwarding-sinal-port

  void add_out_port    (TOutPortInterface    &v_port);
  void add_in_port     (TInPortInterface     &v_port);
  void add_signal_port (TSignalPortInterface &v_port);
  void add_slot_port   (TSlotPortInterface   &v_port);  //!<\note this method also adds forwarding-sinal-port

  bool remove_port (const std::string &v_name);  //!< remove port v_name. return true if removed

  void clear_ports();

  inline void first_access_check() const;

  virtual void on_first_access();

private:

  /*! forbid to copy (every ports and parameter boxes should not be copied) */
  const TModuleInterface& operator= (const TModuleInterface &);


  const std::string instance_name_;
  TAgent *pagent_;    //!< pointer to the host agent
  TCompositeModule  *parent_cmodule_;  //!< pointer to the host composite module
    /*!\note We want to let pagent_ and parent_cmodule_ a constant object for safety.
        But, they should be variable to execute a script's function.
        Thus, I removed a const from them (@Aug.16,2010) */

  /*!\brief parameter box which has links to the configuration parameters of this module */
  var_space::TVariable param_box_config_;

  /*!\brief parameter box which has links to the learning parameters of this module */
  var_space::TVariable param_box_memory_;

  TModuleMode   module_mode_;
  std::ostream  *debug_stream_;

  bool  is_zombie_;  //!< true: the module should be removed, but impossible because some ports are active

  mutable bool never_accessed_;  //!< if true, this module have not been accessed after generated or SetLazyLoadFile

  /*! the set of out-ports. */
  TPortSet<TOutPortInterface*>::type out_ports_;

  /*! the set of in-ports. */
  TPortSet<TInPortInterface*>::type  in_ports_;

  /*! the set of signal-ports. */
  TPortSet<TSignalPortInterface*>::type  signal_ports_;

  /*! the set of slot-ports. */
  TPortSet<TSlotPortInterface*>::type  slot_ports_;


  friend class TPortInterface;  // this class is friend to access first_access_check()

  // following functions are friend(s) to directly access param_box_* (not to execute first_access_check)
  friend inline var_space::TVariable& ll_param_box_config(TModuleInterface &m);
  friend inline var_space::TVariable& ll_param_box_memory(TModuleInterface &m);

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Composite module class */
class TCompositeModule : public TModuleInterface
//===========================================================================================
{
public:

  struct TModuleCell
    {
      std::string       Name;  //! == Ptr->InstanceName
      TModuleInterface  *Ptr;
      bool              Managed;  //! if true, Ptr is freed in Clear() if not NULL
      TModuleCell(void) : Name(""), Ptr(NULL), Managed(true) {}
      TModuleCell(const std::string &id) : Name(id), Ptr(NULL), Managed(true) {}
      TModuleCell(const std::string &id, TModuleInterface *m, bool mngd) : Name(id), Ptr(m), Managed(mngd) {}
    };

  typedef boost::function<bool(const TPortInfo *from_port, const TPortInfo *to_port)> TConnectionManipulator;
  typedef boost::function<bool(const TConstPortInfo *from_port, const TConstPortInfo *to_port)> TConstConnectionManipulator;


  //! clear all modules (memories are freed)
  void Clear();

  override std::string  InheritedModuleName() const {return cmodule_name_;}

  TCompositeModule(const std::string &v_cmodule_name, const std::string &v_instance_name)
    : TModuleInterface (v_instance_name),
      cmodule_name_    (v_cmodule_name)
    {}

  virtual ~TCompositeModule()
    {
      Clear();
    }

  TModuleInterface* SubModulePtr (const std::string &module_name)  {return find_sub_module(module_name,/*error=*/false);}
  const TModuleInterface* SubModulePtr (const std::string &module_name) const  {return find_sub_module(module_name,/*error=*/false);}

  TModuleInterface& SubModule (const std::string &module_name);
  const TModuleInterface& SubModule (const std::string &module_name) const;

  /*! search a module named module_name.
      if a sub-module is a composite one and max_depth>0, module_name is searched recursively */
  TModuleInterface* SearchSubModule (const std::string &module_name, int max_depth=5);
  /*! search a module named module_name.
      if a sub-module is a composite one and max_depth>0, module_name is searched recursively */
  const TModuleInterface* SearchSubModule (const std::string &module_name, int max_depth=5) const;

  bool  SearchSubPort (TPortInterface *port_ptr, TPortInfo &info);
  bool  SearchSubPort (const TPortInterface *port_ptr, TConstPortInfo &info) const;

  TPortInterface*  SearchSubPort (const std::string &unique_code);
  const TPortInterface*  SearchSubPort (const std::string &unique_code) const;

  std::string  SearchSubPortUniqueCode (const TPortInterface *port_ptr) const;

  template <typename t_module>
  t_module& SubModuleAs (const std::string &module_name);

  template <typename t_module>
  const t_module& SubModuleAs (const std::string &module_name) const;

  //! create a module of the specified type
  template <typename t_module>
  t_module&  AddSubModule (const std::string &v_instance_name);

  //! create a module whose type is specified by the string v_module_class
  TModuleInterface&  AddSubModule (const std::string &v_module_class, const std::string &v_instance_name);

  /*! add p into the sub-module set.  this pointer is not managed by this class,
        i.e. the memory is not freed in Clear(), and parameter is not saved (only connection is saved) */
  void AddUnmanagedSubModule (TModuleInterface *p, const std::string &v_instance_name);

  /*! remove a module of the instance name. all connections are disconnected */
  bool RemoveSubModule (const std::string &v_instance_name);

  /*! remove all zombies */
  bool ClearZombies ();

  /*! connect the two ports.  the port types are automatically determined */
  bool SubConnect(
      TModuleInterface &start_module, const std::string &start_port_name,
      TModuleInterface &end_module,   const std::string &end_port_name);

  /*! connect the two ports. the modules are indicated by names. the port types are automatically determined */
  bool SubConnect(
      const std::string &start_module_name, const std::string &start_port_name,
      const std::string &end_module_name,   const std::string &end_port_name);

  /*! disconnect the two ports.  the port types are automatically determined */
  bool SubDisconnect(
      TModuleInterface &start_module, const std::string &start_port_name,
      TModuleInterface &end_module,   const std::string &end_port_name);

  /*! disconnect the two ports. the modules are indicated by names. the port types are automatically determined */
  bool SubDisconnect(
      const std::string &start_module_name, const std::string &start_port_name,
      const std::string &end_module_name,   const std::string &end_port_name);


  /*!\brief for each module, apply the function f */
  void ForEachSubModule (boost::function<bool(TModuleInterface* module)> f);

  /*!\brief for each module, apply the function f */
  void ForEachSubModule (boost::function<bool(const TModuleInterface* module)> f) const;

  /*!\brief for each module, apply the function f */
  void ForEachSubModuleCell (boost::function<bool(const TModuleCell &mcell)> f) const;

  /*!\brief for each connected port, apply the function f */
  void ForEachSubConnection (TConnectionManipulator f);

  /*!\brief for each connected port, apply the function f */
  void ForEachSubConnection (TConstConnectionManipulator f) const;


  /*!\brief export a port sub_module_name.port_name as export_name */
  bool ExportPort (const std::string &sub_module_name, const std::string &port_name, const std::string &export_name);

  /*!\brief export a config-parameter sub_module_name.config.param_name as export_name */
  bool ExportConfig (const std::string &sub_module_name, const std::string &param_name, const std::string &export_name);

  /*!\brief export a memory-parameter sub_module_name.memory.param_name as export_name */
  bool ExportMemory (const std::string &sub_module_name, const std::string &param_name, const std::string &export_name);


  /*!\brief save modules, connections, configurations to a stream */
  bool WriteToStream (std::ostream &os, const std::string &indent="", bool ext_sto_available=false) const;
    // defined in agent_writer.cpp

  /*!\brief save modules, connections, configurations to binary */
  bool WriteToBinary (TBinaryStack &bstack, bool ext_sto_available=false) const;
    // defined in agent_writer.cpp

  /*!\brief load modules, connections, configurations from the file [file_name] (native path format) */
  bool LoadFromFile (const std::string &file_name);
    // defined in agent_binexec.cpp


  override void SetNeverAccessed (bool na);


  void SetAllSubModuleMode (const TModuleMode &mm);
  void SetAllSubDebugStream (std::ostream &os);
  override void SetModuleMode (const TModuleMode &mm)  {TModuleInterface::SetModuleMode(mm); SetAllSubModuleMode(mm);}
  override void SetDebugStream (std::ostream &os)  {TModuleInterface::SetDebugStream(os); SetAllSubDebugStream(os);}

  void ShowAllSubModules (const std::string &format="", std::ostream &os=std::cerr) const;

  void ShowAllSubConnections (std::ostream &os=std::cerr) const;

  /*! TEST: export module structure to a graph description language.
        Draw by graphviz - fdp (e.g. fdp -Tsvg FOO.dot -o BAR.svg) */
  void ExportToDOT (std::ostream &os=std::cout) const;

protected:

  /*! Type of module set (map) whose key is module.InstanceName() */
  typedef std::set<TModuleCell>  TModuleSet;

  enum TExportKind {ekPort=0, ekConfig, ekMemory};
  struct TExportItem
    {
      TExportKind  Kind;
      std::string  ModuleName;
      std::string  ElemName;
      std::string  ExportName;
      TExportItem(TExportKind k, const std::string &mname, const std::string &ename, const std::string &xname)
        : Kind(k), ModuleName(mname), ElemName(ename), ExportName(xname)  {}
    };

  std::string cmodule_name_;

  TModuleSet  sub_modules_;

  std::list<TExportItem>  export_list_;  //!< stored in Export{Port,Memory,Config} to be used in WriteToStream


  inline TModuleInterface* find_sub_module (const std::string &module_name, bool error=false) const;

  /*! remove the module specified by mod_itr. all connections are disconnected */
  bool remove_sub_module (TModuleSet::iterator mod_itr);

  bool apply_f_if_to_port_exists (const TPortInfo *from_port, TPortInterface *to_port_ptr, TConnectionManipulator f);
  bool apply_f_if_to_port_exists_c (const TConstPortInfo *from_port, const TPortInterface *to_port_ptr, TConstConnectionManipulator f) const;

};
//-------------------------------------------------------------------------------------------

inline bool operator<(const TCompositeModule::TModuleCell &lhs, const TCompositeModule::TModuleCell &rhs)
{
  return lhs.Name < rhs.Name;
}
//-------------------------------------------------------------------------------------------

template <typename t_module>
t_module& TCompositeModule::SubModuleAs (const std::string &module_name)
{
  TModuleInterface *m (find_sub_module(module_name,/*error=*/true));
  if (m==NULL)  {lexit(df); return dummy_return<t_module&>::value();}
  if (t_module *m2= dynamic_cast<t_module*>(m))  return *m2;
  LERROR("module "<<module_name<<" is not an instance of "<<t_module::ModuleName()<<".");
  lexit(df); return dummy_return<t_module&>::value();
}
//-------------------------------------------------------------------------------------------

template <typename t_module>
const t_module& TCompositeModule::SubModuleAs (const std::string &module_name) const
{
  TModuleInterface *m (find_sub_module(module_name,/*error=*/true));
  if (m==NULL)  {lexit(df); return dummy_return<t_module&>::value();}
  if (t_module *m2= dynamic_cast<t_module*>(m))  return *m2;
  LERROR("module "<<module_name<<" is not an instance of "<<t_module::ModuleName()<<".");
  lexit(df); return dummy_return<t_module&>::value();
}
//-------------------------------------------------------------------------------------------

//! create a module of the specified type
template <typename t_module>
t_module&  TCompositeModule::AddSubModule (const std::string &v_instance_name)
{
  if (sub_modules_.find(TModuleCell(v_instance_name))!=sub_modules_.end())
  {
    LERROR("module "<<v_instance_name<<" is already registered");
    lexit(df);
  }
  t_module *p= new t_module (v_instance_name);
  sub_modules_.insert(TModuleCell(v_instance_name,p,true));
  p->SetAgent(Agent());
  p->SetParentCModule(this);
  return *p;
}
//-------------------------------------------------------------------------------------------

inline TModuleInterface* TCompositeModule::find_sub_module (const std::string &module_name, bool error) const
{
  TModuleSet::const_iterator itr= sub_modules_.find(module_name);
  if(itr==sub_modules_.end() || itr->Ptr->IsZombie())
  {
    if(error)  LERROR("module "<<module_name<<" is not found.");
    return NULL;
  }
  return itr->Ptr;
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Generator of TCompositeModule  */
class TCompositeModuleGenerator
//===========================================================================================
{
public:

  struct TGeneratorInfo
    {
      TBinaryStack  Binary;
      std::string   FileName;  //!< filename where Binary is defined
      int           LineNum;   //!< line number where Binary is defined
    };

  //! Added: return true, failed: return false
  bool AddGenerator(const std::string &cmodule_name, const TGeneratorInfo &generator);

  //! Removed: return true, failed: return false
  bool RemoveGenerator(const std::string &cmodule_name);

  bool GeneratorExists(const std::string &cmodule_name) const;

  const TGeneratorInfo* Generator(const std::string &cmodule_name) const;

  //! Create an instance of cmodule_name; return true for success
  bool Create(TCompositeModule &instance, const std::string &cmodule_name, bool ignore_export=false) const;
    // defined in agent_binexec.cpp

  //! Write all composite module definitions to a stream
  bool WriteToStream (std::ostream &os, const std::string &indent="") const;
    // defined in agent_writer.cpp

  //! Write all composite module definitions to binary
  bool WriteToBinary (TBinaryStack &bstack) const;
    // defined in agent_writer.cpp

private:

  std::map<std::string, TGeneratorInfo>  generators_;
  std::list<std::string> cmodule_name_list_;  //!< list of composite module names (used in WriteToStream/WriteToBinary)

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief Function manager  */
class TFunctionManager
//===========================================================================================
{
public:

  struct TFunctionInfo
    {
      TBinaryStack Binary;
      std::list<std::string>  ParamList;
      std::string  FileName;  //!< filename where Binary is defined
      int          LineNum;   //!< line number where Binary is defined
    };

  //! Added: return true, failed: return false
  bool AddFunction(const std::string &func_name, const TFunctionInfo &function);

  //! Removed: return true, failed: return false
  bool RemoveFunction(const std::string &func_name);

  bool FunctionExists(const std::string &func_name) const;

  const TFunctionInfo* Function(const std::string &func_name) const;

  bool ExecuteFunction(
          const std::string &func_name, const std::list<var_space::TLiteral> &argv,
          TCompositeModule &context_cmodule, var_space::TLiteral *ret_val=NULL, bool ignore_export=false) const;
    // defined in agent_binexec.cpp

  //! Write all function definitions to a stream
  bool WriteToStream (std::ostream &os, const std::string &indent="") const;
    // defined in agent_writer.cpp

  //! Write all function definitions to binary
  bool WriteToBinary (TBinaryStack &bstack) const;
    // defined in agent_writer.cpp

private:

  std::map<std::string, TFunctionInfo>  functions_;

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief TAgent's Configurations  */
struct TAgentConfigurations
//===========================================================================================
{
  std::string     DataDir;   //!< directoly path to save data (boost::filesystem's portable file-path format)

  std::string     LazyLoadModulePattern;   /*!< regular expression pattern. if a behavior matches with this,
                                                its parameters are saved into the other file,
                                                and will loaded when first required */

  std::string     ExtFilePrefix;  //!< used in SaveToFile and dump* command to save data in an external file

  TAgentConfigurations(var_space::TVariableMap &mmap)
      :
        DataDir                ("nonexistent_dir"),
        LazyLoadModulePattern  (""),
        ExtFilePrefix          ("")
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( DataDir                );
      ADD( LazyLoadModulePattern  );
      ADD( ExtFilePrefix          );
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

  TAgent();
  virtual ~TAgent();

  //! clear all modules and path_list_ (memories are freed)
  void Clear();

  const boost::filesystem::path& CurrentDir() const {return *current_dir_;}


  TCompositeModule&  Modules ()  {return modules_;}
  const TCompositeModule&  Modules () const {return modules_;}

  TModuleInterface& Module (const std::string &module_name)  {return modules_.SubModule(module_name);}
  const TModuleInterface& Module (const std::string &module_name) const {return modules_.SubModule(module_name);}

  /*! search a module named module_name.
      if a sub-module is a composite one and max_depth>0, module_name is searched recursively */
  TModuleInterface* SearchModule (const std::string &module_name, int max_depth=5)  {return modules_.SearchSubModule(module_name,max_depth);}
  /*! search a module named module_name.
      if a sub-module is a composite one and max_depth>0, module_name is searched recursively */
  const TModuleInterface* SearchModule (const std::string &module_name, int max_depth=5) const {return modules_.SearchSubModule(module_name,max_depth);}

  template <typename t_module>
  t_module& ModuleAs (const std::string &module_name)  {return modules_.SubModuleAs<t_module>(module_name);}

  template <typename t_module>
  const t_module& ModuleAs (const std::string &module_name) const {return modules_.SubModuleAs<t_module>(module_name);}

  //! create a module of the specified type
  template <typename t_module>
  t_module&  AddModule (const std::string &v_instance_name)  {return modules_.AddSubModule<t_module>(v_instance_name);}

  //! create a module whose type is specified by the string v_module_class
  TModuleInterface&  AddModule (const std::string &v_module_class, const std::string &v_instance_name)
    {return modules_.AddSubModule(v_module_class, v_instance_name);}


  /*! connect the two ports.  the port types are automatically determined */
  bool Connect(
      TModuleInterface &start_module, const std::string &start_port_name,
      TModuleInterface &end_module,   const std::string &end_port_name)
    {return modules_.SubConnect(start_module, start_port_name, end_module, end_port_name);}

  /*! connect the two ports. the modules are indicated by names. the port types are automatically determined */
  bool Connect(
      const std::string &start_module_name, const std::string &start_port_name,
      const std::string &end_module_name,   const std::string &end_port_name)
    {return modules_.SubConnect(start_module_name, start_port_name, end_module_name, end_port_name);}

  /*! disconnect the ports */
  bool Disconnect(
      TModuleInterface &start_module, const std::string &start_port_name,
      TModuleInterface &end_module,   const std::string &end_port_name)
    {return modules_.SubDisconnect(start_module, start_port_name, end_module, end_port_name);}

  /*! disconnect the ports */
  bool Disconnect(
      const std::string &start_module_name, const std::string &start_port_name,
      const std::string &end_module_name,   const std::string &end_port_name)
    {return modules_.SubDisconnect(start_module_name, start_port_name, end_module_name, end_port_name);}


  const TAgentConfigurations&  Config() const {return conf_;}
  TAgentConfigurations&  SetConfig()  {return conf_;}

  var_space::TVariable& ParamBoxConfig()  {return modules_.ParamBoxConfig();}
  const var_space::TVariable& ParamBoxConfig() const {return modules_.ParamBoxConfig();}



  /*!\brief for each module, apply the function f */
  void ForEachModule (boost::function<bool(TModuleInterface* module)> f)
    {modules_.ForEachSubModule(f);}

  /*!\brief for each module, apply the function f */
  void ForEachModule (boost::function<bool(const TModuleInterface* module)> f) const
    {modules_.ForEachSubModule(f);}

  /*!\brief for each connected port, apply the function f */
  void ForEachConnection (TCompositeModule::TConnectionManipulator f)
    {modules_.ForEachSubConnection(f);}

  /*!\brief for each connected port, apply the function f */
  void ForEachConnection (TCompositeModule::TConstConnectionManipulator f) const
    {modules_.ForEachSubConnection(f);}


  /*!\brief load modules, connections, configurations from the file [filename] (native path format) */
  bool LoadFromFile (const std::string &file_name);
    // defined in agent_binexec.cpp

  /*!\brief save modules, connections, configurations to the file [filename] (native path format) */
  bool SaveToFile (const std::string &filename, const std::string &ext_file_prefix="") const;
    // defined in agent_writer.cpp


  /*!\brief add dir_name (native format path) to the path-list */
  void AddPath (const std::string &dir_name);

  /*!\brief add dir_list (list of native format path) to the path-list */
  void AddPathList (const std::list<std::string> &dir_list);

  std::list<boost::filesystem::path>*  PathListPtr()  {return path_list_;}
  const std::list<boost::filesystem::path>*  PathListPtr() const {return path_list_;}
  std::list<boost::filesystem::path>&  PathList()  {return SetPathList();}
  std::list<boost::filesystem::path>&  SetPathList();

  std::list<std::string>&  LibList()  {return lib_list_;}
  const std::list<std::string>&  LibList() const {return lib_list_;}

  std::list<std::string>&  IncludedList()  {return included_list_;}
  const std::list<std::string>&  IncludedList() const {return included_list_;}


  TCompositeModuleGenerator& CompositeModuleGenerator()  {return cmp_module_generator_;}
  const TCompositeModuleGenerator& CompositeModuleGenerator() const {return cmp_module_generator_;}
  TFunctionManager& FunctionManager()  {return function_manager_;}
  const TFunctionManager& FunctionManager() const {return function_manager_;}

  bool ExecuteFunction(
          const std::string &func_name, const std::list<var_space::TLiteral> &argv,
          TCompositeModule &context_cmodule, var_space::TLiteral *ret_val=NULL, bool ignore_export=false);

  bool ExecuteScript(
          const std::string &script, TCompositeModule &context_cmodule,
          bool ignore_export=false, const std::string &file_name="-");
    // defined in agent_binexec.cpp

  /*!\brief search filename from the path-list, return the native path
      \param [in]omissible_extension  :  indicate an extension with dot, such as ".agent" */
  std::string SearchFileName (const std::string &filename, const std::string &omissible_extension="."SKYAI_DEFAULT_AGENT_SCRIPT_EXT) const;

  /*!\brief return a complete native path to filename which is a relative path from conf_.DataDir */
  std::string GetDataFileName (const std::string &filename) const;


  /*!\brief chech module_name matches with the conf_.LazyLoadModulePattern */
  bool IsLazyLoadModule (const std::string &module_name) const;


  /*!\brief write all port information (port kind, port pointer, every outer module name and port name) to os */
  void DumpPortInfo (std::ostream &os=std::cerr) const;

  void SetAllModuleMode (const TModuleInterface::TModuleMode &mm)
    {modules_.SetModuleMode(mm);}
  void SetAllDebugStream (std::ostream &os)
    {modules_.SetDebugStream(os);}

  void ShowAllModules (const std::string &format="", std::ostream &os=std::cerr) const
    {modules_.ShowAllSubModules(format,os);}

  void ShowAllConnections (std::ostream &os=std::cerr) const
    {modules_.ShowAllSubConnections(os);}

  /*! TEST: export module structure to a graph description language.
        Draw by graphviz - fdp (e.g. fdp -Tsvg FOO.dot -o BAR.svg) */
  void ExportToDOT (std::ostream &os=std::cout) const
    {modules_.ExportToDOT(os);}


protected:

  //! forbit to call a copy constructor
  TAgent (const TAgent&);
  const TAgent& operator=(const TAgent&);

  TCompositeModule  modules_;

  TAgentConfigurations conf_;

  TCompositeModuleGenerator  cmp_module_generator_;
  TFunctionManager           function_manager_;

  boost::filesystem::path             *current_dir_;
  std::list<boost::filesystem::path>  *path_list_;
  std::list<std::string>              lib_list_;  //!< loaded libraries
  std::list<std::string>              included_list_;  //!< included agent scripts

};
//-------------------------------------------------------------------------------------------


//===========================================================================================
// implementation of inline member functions
//===========================================================================================

inline void TPortInterface::first_call_check() const
{
  if (never_called_)
  {
    outer_base_.first_access_check();
    never_called_= false;
  }
}
//-------------------------------------------------------------------------------------------

inline void TModuleInterface::first_access_check() const
{
  if (!never_accessed_)  return;
  never_accessed_= false;
  if (parent_cmodule_)  parent_cmodule_->first_access_check();

  const_cast<TModuleInterface*>(this)->on_first_access();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_base_h
//-------------------------------------------------------------------------------------------
