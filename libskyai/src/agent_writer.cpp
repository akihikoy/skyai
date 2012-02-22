//-------------------------------------------------------------------------------------------
/*! \file    agent_writer.cpp
    \brief   libskyai - certain program (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.07, 2012

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
#include <skyai/agent_writer.h>
#include <skyai/agent_bindef.h>
#include <skyai/base.h>
#include <skyai/types.h>
//-------------------------------------------------------------------------------------------
#include <lora/stl_ext.h>
#include <lora/file.h>
#include <lora/variable_bindef.h>
#include <fstream>
#include <list>
#include <boost/bind.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;


static bool save_module_to_stream (const TCompositeModule::TModuleCell &module, ostream *os, const std::string &indent)
{
  if (module.Managed)
    (*os)<<indent<< "module  "<< module.Ptr->InheritedModuleName() << "  " << module.Ptr->InstanceName() << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_connection_to_stream (const TConstPortInfo *from_port, const TConstPortInfo *to_port, ostream *os, const std::string &indent)
{
  (*os)<<indent<< "connect  "<< from_port->OuterModule->InstanceName() << "." << from_port->Name
                    << " ,   "<< to_port->OuterModule->InstanceName() << "." << to_port->Name << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_agent_config_to_stream (const TAgent *agent, ostream *os, const std::string &indent)
{
  (*os) <<indent<< "config ={" << endl;
  agent->ParamBoxConfig().WriteToStream (*os, true, indent+"    ");
  (*os) <<indent<< "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_config_to_stream (const TModuleInterface *module, ostream *os, const std::string &indent)
{
  if(module->ParamBoxConfig().NoMember())  return true;
  (*os) <<indent<< module->InstanceName() << ".config ={" << endl;
  module->ParamBoxConfig().WriteToStream (*os, true, indent+"    ");
  (*os) <<indent<< "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_memory_to_stream (const TModuleInterface *module, ostream *os, const std::string &indent)
{
  if(module->ParamBoxMemory().NoMember())  return true;
  (*os) <<indent<< module->InstanceName() << ".memory ={" << endl;
  module->ParamBoxMemory().WriteToStream (*os, true, indent+"    ");
  (*os) <<indent<< "  }" << endl;
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_cmp_params_to_stream (const TCompositeModule::TModuleCell &module, ostream *os, const std::string &indent,
              bool save_conf, bool save_mem, bool ext_sto_available);

static bool save_cmp_params_to_stream_base (const TModuleInterface *module, ostream *os, const std::string &indent,
              bool save_conf, bool save_mem, bool ext_sto_available)
{
  using namespace boost::filesystem;
  ostream *s_os(os);
  ofstream ex_ofs;
  if(ext_sto_available && module->Agent().IsLazyLoadModule(module->InstanceName()))
  {
    string ext_filename
      = string(SKYAI_EXT_STORAGE_DIR)
        +"/"+module->Agent().Config().ExtFilePrefix+module->GlobalUniqueCode()+"."SKYAI_DEFAULT_AGENT_SCRIPT_EXT;
    string ext_file_path
      = complete(path(module->Agent().Config().DataDir,native)
        / path(ext_filename)).file_string();
    if(CanOpenFile(ext_file_path,fopAsk))
    {
      ex_ofs.open(ext_file_path.c_str());
      if(ex_ofs)
      {
        s_os= &ex_ofs;
        (*os) <<indent<< "linclude  "<< module->InstanceName()<<" \""<<ext_filename<<"\""<< endl;
      }
    }
  }

  if(const TCompositeModule *cmodule= dynamic_cast<const TCompositeModule*>(module))
  {
    (*s_os) <<indent<< "edit  " << module->InstanceName() << endl;
    (*s_os) <<indent<< "{" << endl;
    cmodule->ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,s_os,indent+"  ",save_conf,save_mem,false));
    (*s_os) <<indent<< "}" << endl;
  }
  else
  {
    if(save_conf)  save_config_to_stream(module,s_os,indent);
    if(save_mem)   save_memory_to_stream(module,s_os,indent);
  }
  if(ex_ofs)  ex_ofs.close();
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_cmp_params_to_stream (const TCompositeModule::TModuleCell &module, ostream *os, const std::string &indent,
              bool save_conf, bool save_mem, bool ext_sto_available)
{
  if(!module.Managed)  return true;

  return save_cmp_params_to_stream_base(module.Ptr, os, indent, save_conf, save_mem, ext_sto_available);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to a stream */
bool TCompositeModule::WriteToStream (std::ostream &os, const std::string &indent, bool ext_sto_available) const
//===========================================================================================
{
  ForEachSubModuleCell (boost::bind(save_module_to_stream,_1,&os,indent));

  os<<endl;
  ForEachSubConnection (boost::bind(save_connection_to_stream,_1,_2,&os,indent));

  if (!export_list_.empty())
  {
    os<<endl;
    for (std::list<TExportItem>::const_iterator itr(export_list_.begin()),last(export_list_.end()); itr!=last; ++itr)
    {
      os<<indent<< "export  "<<itr->ModuleName;
      switch (itr->Kind)
      {
      case ekPort   : break;
      case ekConfig : os<<".config"; break;
      case ekMemory : os<<".memory"; break;
      default : LERROR("fatal!"); lexit(df);
      }
      os<<"."<<itr->ElemName<<"  as  "<<itr->ExportName<<endl;
    }
  }

  os<<endl;
  ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,&os,indent,true,true,ext_sto_available));
  return true;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
//===========================================================================================



static bool save_module_to_binary (const TCompositeModule::TModuleCell &module, TBinaryStack *bstack)
{
  if (module.Managed)
  {
    var_space::AddPushID(*bstack, module.Ptr->InheritedModuleName());
    var_space::AddPushID(*bstack, module.Ptr->InstanceName());
    var_space::AddCommand(*bstack, agent_parser::bin::cmd::MODULE);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_connection_to_binary (const TConstPortInfo *from_port, const TConstPortInfo *to_port, TBinaryStack *bstack)
{
  var_space::AddPushID(*bstack, from_port->OuterModule->InstanceName());
  var_space::AddPushID(*bstack, from_port->Name);
  var_space::AddPushID(*bstack, to_port->OuterModule->InstanceName());
  var_space::AddPushID(*bstack, to_port->Name);
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::CONNECT);
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_agent_config_to_binary (const TAgent *agent, TBinaryStack *bstack)
{
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::ASGN_GCNF);
  agent->ParamBoxConfig().WriteToBinary (*bstack);
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::ASGN_END);
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_config_to_binary (const TModuleInterface *module, TBinaryStack *bstack)
{
  if(module->ParamBoxConfig().NoMember())  return true;
  var_space::AddPushID(*bstack, module->InstanceName());
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::ASGN_CNF);
  module->ParamBoxConfig().WriteToBinary (*bstack);
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::ASGN_END);
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_memory_to_binary (const TModuleInterface *module, TBinaryStack *bstack)
{
  if(module->ParamBoxMemory().NoMember())  return true;
  var_space::AddPushID(*bstack, module->InstanceName());
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::ASGN_MEM);
  module->ParamBoxMemory().WriteToBinary (*bstack);
  var_space::AddCommand(*bstack, agent_parser::bin::cmd::ASGN_END);
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_cmp_params_to_binary (const TCompositeModule::TModuleCell &module, TBinaryStack *bstack,
              bool save_conf, bool save_mem, bool ext_sto_available);

static bool save_cmp_params_to_binary_base (const TModuleInterface *module, TBinaryStack *bstack,
              bool save_conf, bool save_mem, bool ext_sto_available)
{
  using namespace boost::filesystem;
  TBinaryStack *s_bstack(bstack);
  if(ext_sto_available && module->Agent().IsLazyLoadModule(module->InstanceName()))
  {
    /*!\todo:FIXME*/FIXME("implement:lazyload");
    // s_bstack= ...
  }

  if(const TCompositeModule *cmodule= dynamic_cast<const TCompositeModule*>(module))
  {
    var_space::AddPushID(*bstack, module->InstanceName());
    var_space::AddCommand(*bstack, agent_parser::bin::cmd::EDIT);
    cmodule->ForEachSubModuleCell (boost::bind(save_cmp_params_to_binary,_1,s_bstack,save_conf,save_mem,false));
    var_space::AddCommand(*bstack, agent_parser::bin::cmd::EDIT_END);
  }
  else
  {
    if(save_conf)  save_config_to_binary(module,s_bstack);
    if(save_mem)   save_memory_to_binary(module,s_bstack);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_cmp_params_to_binary (const TCompositeModule::TModuleCell &module, TBinaryStack *bstack,
              bool save_conf, bool save_mem, bool ext_sto_available)
{
  if(!module.Managed)  return true;

  return save_cmp_params_to_binary_base(module.Ptr, bstack, save_conf, save_mem, ext_sto_available);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to binary */
bool TCompositeModule::WriteToBinary (TBinaryStack &bstack, bool ext_sto_available) const
//===========================================================================================
{
  ForEachSubModuleCell (boost::bind(save_module_to_binary,_1,&bstack));
  ForEachSubConnection (boost::bind(save_connection_to_binary,_1,_2,&bstack));

  if (!export_list_.empty())
  {
    for (std::list<TExportItem>::const_iterator itr(export_list_.begin()),last(export_list_.end()); itr!=last; ++itr)
    {
      var_space::AddPushID(bstack, itr->ModuleName);
      var_space::AddPushID(bstack, itr->ElemName);
      var_space::AddPushID(bstack, itr->ExportName);
      switch (itr->Kind)
      {
      case ekPort   : var_space::AddCommand(bstack, agent_parser::bin::cmd::EXPO_P_AS); break;
      case ekConfig : var_space::AddCommand(bstack, agent_parser::bin::cmd::EXPO_C_AS); break;
      case ekMemory : var_space::AddCommand(bstack, agent_parser::bin::cmd::EXPO_M_AS); break;
      default : LERROR("fatal!"); lexit(df);
      }
    }
  }

  ForEachSubModuleCell (boost::bind(save_cmp_params_to_binary,_1,&bstack,true,true,ext_sto_available));
  return true;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
//===========================================================================================


static bool save_cmodule_generator (const std::string &id, const TCompositeModuleGenerator::TGeneratorInfo &generator,
              std::ostream &os, const std::string &indent)
{
  os<<indent<<"composite  "<<id<<endl;
  os<<indent<<"{"<<endl;
//FIXME: os<<TIndentString(generator.Script, indent+"  ");
agent_parser::PrintToStream(generator.Binary,os);
// like that:
      // TCompositeModule cmodule(option("export_dot"), "temporary");
      // cmodule.SetAgent(agent);
      // agent.CompositeModuleGenerator().Create(cmodule, option("export_dot"));
      // cmodule.ExportToDOT(cout);
  os<<indent<<"}"<<endl;
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! Write all composite module definitions to a stream
bool TCompositeModuleGenerator::WriteToStream (std::ostream &os, const std::string &indent) const
//===========================================================================================
{
  std::map<std::string, TCompositeModuleGenerator::TGeneratorInfo>::const_iterator  igenerator;
  for (std::list<std::string>::const_iterator itr(cmodule_name_list_.begin()),last(cmodule_name_list_.end()); itr!=last; ++itr)
  {
    igenerator= generators_.find(*itr);
    if (igenerator==generators_.end())
    {
      LERROR("fatal! composite module generator is lost: "<<igenerator->first);
      lexit(df);
    }
    save_cmodule_generator(*itr,igenerator->second,os,indent);
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! Write all composite module definitions to binary
bool TCompositeModuleGenerator::WriteToBinary (TBinaryStack &bstack) const
//===========================================================================================
{
//*FIXME*/:
  return true;
}
//-------------------------------------------------------------------------------------------

static bool save_function (const std::string &id, const TFunctionManager::TFunctionInfo &finfo,
              std::ostream &os, const std::string &indent)
{
  os<<indent<<"def  "<<id<<"("<<ContainerToStr(finfo.ParamList.begin(),finfo.ParamList.end(),", ")<<")"<<endl;
  os<<indent<<"{"<<endl;
//FIXME: os<<TIndentString(finfo.Script, indent+"  ");
agent_parser::PrintToStream(finfo.Binary,os);
  os<<indent<<"}"<<endl;
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! Write all function definitions to a stream
bool TFunctionManager::WriteToStream (std::ostream &os, const std::string &indent) const
//===========================================================================================
{
  for (std::map<std::string, TFunctionInfo>::const_iterator itr(functions_.begin()),last(functions_.end()); itr!=last; ++itr)
  {
    save_function (itr->first,itr->second,os,indent);
  }
  return true;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
//! Write all function definitions to binary
bool TFunctionManager::WriteToBinary (TBinaryStack &bstack) const
//===========================================================================================
{
//*FIXME*/:
  return true;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief save modules, connections, configurations to the file [filename] (native path format) */
bool TAgent::SaveToFile (const std::string &filename, const std::string &ext_file_prefix) const
//===========================================================================================
{
  boost::filesystem::path  file_path (filename, boost::filesystem::native);
  return SaveAgentToFile (*this, file_path, ext_file_prefix);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to the file [path_list] */
bool SaveAgentToFile (const TAgent &agent, const boost::filesystem::path &file_path, const std::string &ext_file_prefix)
//===========================================================================================
{
  using namespace boost::filesystem;

  if (!exists(file_path.parent_path()))
  {
    LERROR("Cannot save data into the file: "<<file_path.file_string()
            <<" because the parent path: "<<file_path.parent_path().file_string()
            <<" does not exist.");
    return false;
  }
  if (!CanOpenFile(file_path.file_string(),fopAsk))  return false;

  string tmp_ext_file_prefix;
  if (ext_file_prefix!="")
  {
    tmp_ext_file_prefix= ext_file_prefix;
    std::swap(const_cast<string&>(agent.Config().ExtFilePrefix), tmp_ext_file_prefix);  // we need to use const_cast since `agent' should be const
  }

  ofstream  ofs(file_path.file_string().c_str());
  bool res= WriteAgentToStream(agent, ofs, true);

  if (ext_file_prefix!="")
    std::swap(const_cast<string&>(agent.Config().ExtFilePrefix), tmp_ext_file_prefix);
  return res;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief save modules, connections, configurations to the stream [os] */
bool WriteAgentToStream (const TAgent &agent, ostream &os, bool ext_sto_available)
//===========================================================================================
{
  save_agent_config_to_stream (&agent, &os, "");
  os<<endl;
  agent.CompositeModuleGenerator().WriteToStream(os);
  os<<endl;
  agent.FunctionManager().WriteToStream(os);
  os<<endl;
  return  agent.Modules().WriteToStream(os, "", ext_sto_available);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================


//===========================================================================================
/*!\brief dump information */
bool DumpCModInfo (const TCompositeModule &cmodule, const std::string &filename,
      const std::string &kind, const std::string *opt, const std::string &indent)
//===========================================================================================
{
  ostream  *p_os(NULL);
  ofstream  ofs;
  bool ext_sto_available;
  if (filename!="")
  {
    std::string file_path= cmodule.Agent().GetDataFileName(filename);
    if (!CanOpenFile(file_path,fopAsk))  return false;
    ofs.open(file_path.c_str());
    if (!ofs)
    {
      LERROR("Cannot save data into the file: "<<file_path);
      return false;
    }
    p_os= &ofs;
    ext_sto_available= true;
  }
  else
  {
    p_os= &std::cout;
    ext_sto_available= false;
  }

  if(kind=="agent")
  {
    return WriteAgentToStream (cmodule.Agent(), *p_os, ext_sto_available);
  }
  else if(kind=="config")
  {
    *p_os <<indent<< "config ={" << endl;
    cmodule.ParamBoxConfig().WriteToStream (*p_os, true, indent+"    ");
    *p_os <<indent<< "  }" << endl;
  }
  else if(kind=="mod_list")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_module_to_stream,_1,p_os,indent));
  }
  else if(kind=="conn_list")
  {
    cmodule.ForEachSubConnection (boost::bind(save_connection_to_stream,_1,_2,p_os,indent));
  }
  else if(kind=="all_mod")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,p_os,indent,true,true, ext_sto_available));
  }
  else if(kind=="all_mod_conf")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,p_os,indent,true,false, ext_sto_available));
  }
  else if(kind=="all_mod_mem")
  {
    cmodule.ForEachSubModuleCell (boost::bind(save_cmp_params_to_stream,_1,p_os,indent,false,true, ext_sto_available));
  }
  else if(kind=="all_cmp")
  {
    cmodule.Agent().CompositeModuleGenerator().WriteToStream(*p_os);
  }
  else if(kind=="all_func")
  {
    cmodule.Agent().FunctionManager().WriteToStream(*p_os);
  }
  else if(kind=="mod" || kind=="mod_conf" || kind=="mod_mem")
  {
    if (opt==NULL)  {LERROR("use dump2"); return false;}
    if (*opt=="")   {LERROR("invalid second option"); return false;}
    const TModuleInterface *module(cmodule.SubModulePtr(*opt));
    if (module==NULL)  {LERROR(*opt<<": module not found"); *p_os<<*opt<<": module not found"<<endl; return false;}
    if (kind=="mod")            save_cmp_params_to_stream_base (module, p_os, "",true,true,  ext_sto_available);
    else if (kind=="mod_conf")  save_cmp_params_to_stream_base (module, p_os, "",true,false, ext_sto_available);
    else if (kind=="mod_mem")   save_cmp_params_to_stream_base (module, p_os, "",false,true, ext_sto_available);
  }
  else if(kind=="cmp")
  {
    if (opt==NULL)  {LERROR("use dump2"); return false;}
    if (*opt=="")   {LERROR("invalid second option"); return false;}
    const TCompositeModuleGenerator::TGeneratorInfo *pgenerator= cmodule.Agent().CompositeModuleGenerator().Generator(*opt);
    if (pgenerator)  save_cmodule_generator(*opt,*pgenerator,*p_os,indent);
    else  return false;
  }
  else if(kind=="func")
  {
    if (opt==NULL)  {LERROR("use dump2"); return false;}
    if (*opt=="")   {LERROR("invalid second option"); return false;}
    const TFunctionManager::TFunctionInfo *pfunction= cmodule.Agent().FunctionManager().Function(*opt);
    if (pfunction)  save_function(*opt,*pfunction,*p_os,indent);
    else  return false;
  }
  else
  {
    LERROR("invalid dump kind: "<<ConvertToStr(kind));
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
