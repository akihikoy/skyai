//-------------------------------------------------------------------------------------------
/*! \file    agent_binexec.cpp
    \brief   libskyai - certain program (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Feb.03, 2012

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
#include <skyai/agent_binexec.h>
#include <skyai/agent_writer.h>
#include <skyai/agent_parser.h>
//-------------------------------------------------------------------------------------------
#include <boost/bind.hpp>
#include <boost/filesystem/operations.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace agent_parser
{

inline boost::filesystem::path  operator+ (const boost::filesystem::path &file_path, const std::string &rhs)
{
  return file_path.parent_path()/(file_path.filename()+rhs);
}

//===========================================================================================
// class TBinExecutor
//===========================================================================================

bool TBinExecutor::OnInclude (const std::string &file_name, std::string &abs_file_name, bool once)
{
  boost::filesystem::path absolute_path;
  if (!search_agent_file(file_name,absolute_path))
    return false;

  abs_file_name= absolute_path.file_string();
  if (included_list_!=NULL)
  {
    if (std::find(included_list_->begin(),included_list_->end(), abs_file_name)==included_list_->end())
      included_list_->push_back(abs_file_name);
    else if(once)
      abs_file_name= "";
  }
  return true;
}
//-------------------------------------------------------------------------------------------

bool TBinExecutor::search_agent_file (const boost::filesystem::path &file_path, boost::filesystem::path &absolute_path) const
{
  using namespace boost::filesystem;
  if (file_path.is_complete())
  {
    if (exists((absolute_path= file_path)))  return true;
    if (exists((absolute_path= file_path.parent_path()/(file_path.filename()+"."SKYAI_DEFAULT_AGENT_SCRIPT_EXT))))  return true;
    return false;
  }

  if (exists((absolute_path= current_dir_/file_path)))  return true;
  if (exists((absolute_path= current_dir_/file_path+"."SKYAI_DEFAULT_AGENT_SCRIPT_EXT)))  return true;

  if (path_list_==NULL)  return false;

  path  tmp_path(file_path);
  for (std::list<path>::const_iterator ditr(path_list_->begin()), ditr_last(path_list_->end()); ditr!=ditr_last; ++ditr)
    if (exists((absolute_path= *ditr/tmp_path)))  return true;
  tmp_path= file_path+"."SKYAI_DEFAULT_AGENT_SCRIPT_EXT;
  for (std::list<path>::const_iterator ditr(path_list_->begin()), ditr_last(path_list_->end()); ditr!=ditr_last; ++ditr)
    if (exists((absolute_path= *ditr/tmp_path)))  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

void TBinExecutor::inherit_module (bool ignore_export)
{
  std::string  cmodule_name(pop_id());
  if(mode_stack_.empty() || mode_stack_.back()!=emCompositeDef)
  {
    print_error("inherit/inherit_prv: allowed only within a composite module definition");
    return;
  }

  LASSERT(cmp_module_generator_!=NULL);
  LASSERT(!cmodule_stack_.empty());
  bool res= cmp_module_generator_->Create(*(cmodule_stack_.back()), cmodule_name, ignore_export);
  if(!res)  print_error(cmodule_name+": failed to inherit ");
}
//-------------------------------------------------------------------------------------------

//! call function of identifier func_id with arguments argv, store the return value into ret_val
/*override*/bool TBinExecutor::function_call(const std::string &func_id, std::list<var_space::TLiteral> &argv, var_space::TLiteral &ret_val)
{
  if(TParent::function_call(func_id, argv, ret_val))  return true;

  LASSERT(!cmodule_stack_.empty());
  LASSERT(function_manager_!=NULL);
  if (!function_manager_->ExecuteFunction(func_id, argv, *(cmodule_stack_.back()), &ret_val, ignore_export_))
  {
    print_error("failed to execute function: "+func_id);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//! access to the member of value
/*override*/var_space::TVariable TBinExecutor::member_access(const var_space::TLiteral &value, const var_space::TLiteral &member_c)
{
LDBGVAR(value);
LDBGVAR(member_c);
  if(!value.IsIdentifier() || !member_c.IsIdentifier())
    return TParent::member_access(value,member_c);

  var_space::TIdentifier identifier(value.AsIdentifier());
  var_space::TIdentifier member(member_c.AsIdentifier());

  if(identifier=="config")  // global config
    return cmodule_stack_.back()->ParamBoxConfig().GetMember(var_space::TVariable(member));

  if(member=="config")
    return cmodule_stack_.back()->SubModule(identifier).ParamBoxConfig();
  if(member=="memory")
    return cmodule_stack_.back()->SubModule(identifier).ParamBoxMemory();

  return TParent::member_access(value,member_c);
}
//-------------------------------------------------------------------------------------------

/*override*/void TBinExecutor::exec_command(int command, const TBinaryStack &bstack)
{
  if(!mode_stack_.empty())
  {
    if(mode_stack_.back()==emFunctionDef)
    {
      if(command==bin::cmd::FUNC_DEF)  {print_error("def: forbidden within a function definition"); return;}
      if(command!=bin::cmd::FDEF_END)
      {
        CopyCommand(command, bstack, tmp_func_info_.second.Binary);
        return;
      }
    }
    else if(mode_stack_.back()==emSkipIf)
    {
      if(command==bin::cmd::CTRL_ELSE || command==bin::cmd::CTRL_END_IF)
      {
        if(bstack.ReadI()==tmp_ctrl_id_)  mode_stack_.pop_back();
        return;
      }
      SkipCommand(command, bstack);
      return;
    }
  }

  if(command<bin::cmd::COMMAND_BASE)
  {
    TParent::exec_command(command,bstack);
    return;
  }
  switch(command)
  {
  #define CALL_CMD_EXEC(x_cmd) case bin::cmd::x_cmd: cmd_##x_cmd (command, bstack); break;
  CALL_CMD_EXEC( LINCLUDE  )
  CALL_CMD_EXEC( DUMP1     )
  CALL_CMD_EXEC( DUMP2     )

  CALL_CMD_EXEC( MODULE    )
  CALL_CMD_EXEC( REMOVE    )
  CALL_CMD_EXEC( CONNECT   )
  CALL_CMD_EXEC( DISCNCT   )

  CALL_CMD_EXEC( ASGN_GCNF )
  CALL_CMD_EXEC( ASGN_CNF  )
  CALL_CMD_EXEC( ASGN_MEM  )
  CALL_CMD_EXEC( ASGN_END  )
  CALL_CMD_EXEC( EDIT      )
  CALL_CMD_EXEC( EDIT_END  )

  CALL_CMD_EXEC( COMPOSITE )
  CALL_CMD_EXEC( CMP_END   )
  CALL_CMD_EXEC( FUNC_DEF  )
  CALL_CMD_EXEC( FDEF_END  )
  CALL_CMD_EXEC( S_PARAMS  )
  CALL_CMD_EXEC( RETURN    )

  CALL_CMD_EXEC( DESTROY   )

  CALL_CMD_EXEC( INHERIT   )
  CALL_CMD_EXEC( INHERITPR )

  CALL_CMD_EXEC( EXPO_P    )
  CALL_CMD_EXEC( EXPO_P_AS )
  CALL_CMD_EXEC( EXPO_C    )
  CALL_CMD_EXEC( EXPO_C_AS )
  CALL_CMD_EXEC( EXPO_M    )
  CALL_CMD_EXEC( EXPO_M_AS )

  CALL_CMD_EXEC( CTRL_IF     )
  CALL_CMD_EXEC( CTRL_ELSE   )
  CALL_CMD_EXEC( CTRL_END_IF )
  #undef CALL_CMD_EXEC

  default:  FIXME("unknown command code:"<<command);
  }
}
//-------------------------------------------------------------------------------------------

#define IMPL_CMD_EXEC(x_cmd)  void TBinExecutor::cmd_##x_cmd (int command, const TBinaryStack &bstack)

IMPL_CMD_EXEC( LINCLUDE  ) // bin=[-]; pop two values(1,2; 1:str,2:id), add str to the lazy-include-list of id;
{
  if(forbidden_in_composite("linclude"))  return;

  std::string filename(pop_string());
  std::string identifier(pop_id());

  LASSERT(!cmodule_stack_.empty());
  if (cmodule_stack_.back()->Agent().SearchFileName(filename)=="")
  {
    print_error(filename+": file not found");
    return;
  }
  cmodule_stack_.back()->SubModule(identifier).SetLazyLoadFile(filename);
}
IMPL_CMD_EXEC( DUMP1     ) // bin=[-]; pop two strings(1,2), dump 2 into file 1;
{
  std::string filename(pop_string()), str1(pop_string());
  LASSERT(!cmodule_stack_.empty());
  if (!DumpCModInfo(*(cmodule_stack_.back()), filename, str1, NULL))
    print_error("dump1: failed to dump info");
}
IMPL_CMD_EXEC( DUMP2     ) // bin=[-]; pop three strings(1,2,3), dump 3,2 into file 1;
{
  std::string filename(pop_string()), str2(pop_string()), str1(pop_string());
  LASSERT(!cmodule_stack_.empty());
  if (!DumpCModInfo(*(cmodule_stack_.back()), filename, str1, &str2))
    print_error("dump2: failed to dump info");
}

IMPL_CMD_EXEC( MODULE    ) // bin=[-]; pop two identifiers(1,2), create module: type 2, id 1;
{
  if(forbidden_in_edit("module"))  return;

  std::string identifier(pop_id()), type(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->AddSubModule(type, identifier);
}
IMPL_CMD_EXEC( REMOVE    ) // bin=[-]; pop an identifier(1), remove module: id 1;
{
  if(forbidden_in_edit("remove"))  return;

  std::string identifier(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->RemoveSubModule(identifier);
}
IMPL_CMD_EXEC( CONNECT   ) // bin=[-]; pop four identifiers(1,2,3,4), connect 4.3-2.1;
{
  if(forbidden_in_edit("connect"))  return;

  std::string port2(pop_id()), module2(pop_id()), port1(pop_id()), module1(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->SubConnect(module1, port1,  module2, port2);
}
IMPL_CMD_EXEC( DISCNCT   ) // bin=[-]; pop four identifiers(1,2,3,4), disconnect 4.3-2.1;
{
  if(forbidden_in_edit("disconnect"))  return;

  std::string port2(pop_id()), module2(pop_id()), port1(pop_id()), module1(pop_id());
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.back()->SubDisconnect(module1, port1,  module2, port2);
}

IMPL_CMD_EXEC( ASGN_GCNF ) // bin=[-]; start assigning to global config;
{
  if(forbidden_in_composite("config(global)"))  return;

  LASSERT(!cmodule_stack_.empty());
  TParent::PushVariable(cmodule_stack_.back()->ParamBoxConfig());
}
IMPL_CMD_EXEC( ASGN_CNF  ) // bin=[-]; pop an identifier, start assigning to its config;
{
  std::string identifier(pop_id());
  LASSERT(!cmodule_stack_.empty());
  TParent::PushVariable(cmodule_stack_.back()->SubModule(identifier).ParamBoxConfig());
}
IMPL_CMD_EXEC( ASGN_MEM  ) // bin=[-]; pop an identifier, start assigning to its memory;
{
  std::string identifier(pop_id());
  LASSERT(!cmodule_stack_.empty());
  TParent::PushVariable(cmodule_stack_.back()->SubModule(identifier).ParamBoxMemory());
}
IMPL_CMD_EXEC( ASGN_END  ) // bin=[-]; end assigning;
{
  TParent::PopVariable();
  LASSERT(TParent::VariableStackSize()==0);
}
IMPL_CMD_EXEC( EDIT      ) // bin=[-]; pop an identifier, start editing it;
{
  std::string identifier(pop_id());
  LASSERT(!cmodule_stack_.empty());
  TCompositeModule *cmodule= dynamic_cast<TCompositeModule*>(&(cmodule_stack_.back()->SubModule(identifier)));
  if (cmodule==NULL)
  {
    print_error(identifier+": not a composite module");
    return;
  }
  cmodule_stack_.push_back (cmodule);
  mode_stack_.push_back(emEdit);
}
IMPL_CMD_EXEC( EDIT_END  ) // bin=[-]; end editing;
{
  LASSERT(!mode_stack_.empty());
  LASSERT(mode_stack_.back()==emEdit);
  mode_stack_.pop_back();
  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.pop_back();
}

IMPL_CMD_EXEC( COMPOSITE ) // bin=[-]; pop an identifier, start defining a composite module;
{
  if(forbidden_in_composite("composite"))  return;
  if(forbidden_in_edit("edit"))  return;

  std::string module_name(pop_id());

  LASSERT(cmp_module_generator_!=NULL);
  if (cmp_module_generator_->GeneratorExists (module_name))
  {
    print_error(module_name+": already defined as a composite module");
    const TCompositeModuleGenerator::TGeneratorInfo *info= cmp_module_generator_->Generator(module_name);
    LERROR("first definition: "<<info->FileName<<":"<<info->LineNum);
    return;
  }
  tmp_cmod_generator_info_.first= module_name;
  tmp_cmod_generator_info_.second= TCompositeModuleGenerator::TGeneratorInfo();
  tmp_cmod_generator_info_.second.FileName=  file_name_;
  tmp_cmod_generator_info_.second.LineNum=  line_num_;

  LASSERT(!cmodule_stack_.empty());
  cmodule_entity_stack_.push_back (TCompositeModule(module_name,"temporary"));
  cmodule_entity_stack_.back().SetAgent (cmodule_stack_.back()->Agent());
  cmodule_stack_.push_back (&(cmodule_entity_stack_.back()));

  mode_stack_.push_back(emCompositeDef);
}
IMPL_CMD_EXEC( CMP_END   ) // bin=[-]; end defining the composite module;
{
  LASSERT(!mode_stack_.empty());
  LASSERT1op1(mode_stack_.back(),==,emCompositeDef);
  mode_stack_.pop_back();

  LASSERT(!cmodule_stack_.empty());
  cmodule_stack_.pop_back();

  LASSERT(!cmodule_entity_stack_.empty());
  cmodule_entity_stack_.back().WriteToBinary(tmp_cmod_generator_info_.second.Binary);
  cmodule_entity_stack_.pop_back();

  if (!cmp_module_generator_->AddGenerator(tmp_cmod_generator_info_.first, tmp_cmod_generator_info_.second))
  {
    error_= true;
    return;
  }
}
IMPL_CMD_EXEC( FUNC_DEF  ) // bin=[-]; pop identifiers (parameters,func-name), start defining a function;
{
  if(forbidden_in_composite("def"))  return;
  if(forbidden_in_edit("def"))  return;

  tmp_func_info_.second= TFunctionManager::TFunctionInfo();
  tmp_func_info_.second.ParamList.clear();
  for(var_space::TIdentifier id(pop_id()); id!="@def"; id=pop_id())
    tmp_func_info_.second.ParamList.push_front(id);
  tmp_func_info_.second.FileName=  file_name_;
  tmp_func_info_.second.LineNum=  line_num_;

  // function name:
  tmp_func_info_.first= tmp_func_info_.second.ParamList.front();
  tmp_func_info_.second.ParamList.pop_front();

  LASSERT(function_manager_!=NULL);
  if (function_manager_->FunctionExists (tmp_func_info_.first))
  {
    print_error(tmp_func_info_.first+": already defined as a function");
    const TFunctionManager::TFunctionInfo *info= function_manager_->Function(tmp_func_info_.first);
    LERROR("first definition: "<<info->FileName<<":"<<info->LineNum);
    return;
  }

  mode_stack_.push_back(emFunctionDef);
}
IMPL_CMD_EXEC( FDEF_END  ) // bin=[-]; end defining the function;
{
  LASSERT(!mode_stack_.empty());
  LASSERT1op1(mode_stack_.back(),==,emFunctionDef);
  mode_stack_.pop_back();

  if (!function_manager_->AddFunction (tmp_func_info_.first, tmp_func_info_.second))
  {
    error_= true;
  }
}
IMPL_CMD_EXEC( S_PARAMS  ) // bin=[-]; start parameters of function-definition;
{
  literal_stack_.push_back(var_space::LiteralId("@def"));
}
IMPL_CMD_EXEC( RETURN    ) // bin=[-]; pop a value, set it as a return value, exit the execution;
{
  return_value_= pop_literal();
  bstack.GoLast();
}

IMPL_CMD_EXEC( DESTROY   ) // bin=[-]; pop two values(1,2; 1:id,2:str), destroy 1 of kind 2;
{
  std::string identifier(pop_id());
  std::string str1(pop_string());

  LASSERT(function_manager_!=NULL);
  LASSERT(cmp_module_generator_!=NULL);
  if (str1=="func")
    {if (!function_manager_->RemoveFunction(identifier))  error_= true;}
  else if (str1=="cmp")
    {if (!cmp_module_generator_->RemoveGenerator(identifier))  error_= true;}
  else
    {error_= true; LERROR("invalid destroy kind: "<<ConvertToStr(str1));}
}

IMPL_CMD_EXEC( INHERIT   ) // bin=[-]; pop an identifier, inherit the module;
{
  inherit_module(false);
}
IMPL_CMD_EXEC( INHERITPR ) // bin=[-]; pop an identifier, inherit_prv the module;
{
  inherit_module(true);
}

IMPL_CMD_EXEC( EXPO_P    ) // bin=[-]; pop two identifier(1,2), export the port  2.1 as 1;
{
  if(forbidden_in_edit("export"))  return;

  std::string  port_name(pop_id()), module_name(pop_id());
  std::string  &export_name(port_name);

  if(ignore_export_)  return;
  LASSERT(!cmodule_stack_.empty());
  if(!cmodule_stack_.back()->ExportPort (module_name, port_name, export_name))
    error_= true;
}
IMPL_CMD_EXEC( EXPO_P_AS ) // bin=[-]; pop three identifier(1,2,3), export the port  3.2 as 1;
{
  if(forbidden_in_edit("export"))  return;

  std::string  export_name(pop_id()), port_name(pop_id()), module_name(pop_id());

  if(ignore_export_)  return;
  LASSERT(!cmodule_stack_.empty());
  if(!cmodule_stack_.back()->ExportPort (module_name, port_name, export_name))
    error_= true;
}
IMPL_CMD_EXEC( EXPO_C    ) // bin=[-]; pop two identifier(1,2), export the config  2.config.1 as 1;
{
  if(forbidden_in_edit("export"))  return;

  std::string  param_name(pop_id()), module_name(pop_id());
  std::string  &export_name(param_name);

  if(ignore_export_)  return;
  LASSERT(!cmodule_stack_.empty());
  if(!cmodule_stack_.back()->ExportConfig (module_name, param_name, export_name))
    error_= true;
}
IMPL_CMD_EXEC( EXPO_C_AS ) // bin=[-]; pop three identifier(1,2,3), export the config  3.config.2 as 1;
{
  if(forbidden_in_edit("export"))  return;

  std::string  export_name(pop_id()), param_name(pop_id()), module_name(pop_id());

  if(ignore_export_)  return;
  LASSERT(!cmodule_stack_.empty());
  if(!cmodule_stack_.back()->ExportConfig (module_name, param_name, export_name))
    error_= true;
}
IMPL_CMD_EXEC( EXPO_M    ) // bin=[-]; pop two identifier(1,2), export the memory  2.memory.1 as 1;
{
  if(forbidden_in_edit("export"))  return;

  std::string  param_name(pop_id()), module_name(pop_id());
  std::string  &export_name(param_name);

  if(ignore_export_)  return;
  LASSERT(!cmodule_stack_.empty());
  if(!cmodule_stack_.back()->ExportMemory (module_name, param_name, export_name))
    error_= true;
}
IMPL_CMD_EXEC( EXPO_M_AS ) // bin=[-]; pop three identifier(1,2,3), export the memory  3.memory.2 as 1;
{
  if(forbidden_in_edit("export"))  return;

  std::string  export_name(pop_id()), param_name(pop_id()), module_name(pop_id());

  if(ignore_export_)  return;
  LASSERT(!cmodule_stack_.empty());
  if(!cmodule_stack_.back()->ExportMemory (module_name, param_name, export_name))
    error_= true;
}

IMPL_CMD_EXEC( CTRL_IF     ) // bin=[- value]; pop a value, if false: skip until finding [ELSE value] or [END_IF value], if true: do nothing;
{
  var_space::TLiteral cond= pop_literal();
  LASSERT(cond.IsPrimitive());
  var_space::CastToBool(cond.AsPrimitive());
  tmp_ctrl_id_= bstack.ReadI();
  if(!cond.AsPrimitive().Bool())
  {
    mode_stack_.push_back(emSkipIf);
  }
}
IMPL_CMD_EXEC( CTRL_ELSE   ) // bin=[- value]; skip until finding [END_IF value];
{
  tmp_ctrl_id_= bstack.ReadI();
  mode_stack_.push_back(emSkipIf);
}
IMPL_CMD_EXEC( CTRL_END_IF ) // bin=[- value]; do nothing;
{
  bstack.ReadI();
}

#undef IMPL_CMD_EXEC


















//===========================================================================================
// class TBinWriter
//===========================================================================================

/*override*/void TBinWriter::exec_command(int command, const TBinaryStack &bstack)
{
  if(command<bin::cmd::COMMAND_BASE)
  {
    TParent::exec_command(command,bstack);
    return;
  }
  switch(command)
  {
  #define CALL_CMD_EXEC(x_cmd) case bin::cmd::x_cmd: cmd_##x_cmd (command, bstack); break;
  CALL_CMD_EXEC( LINCLUDE  )
  CALL_CMD_EXEC( DUMP1     )
  CALL_CMD_EXEC( DUMP2     )

  CALL_CMD_EXEC( MODULE    )
  CALL_CMD_EXEC( REMOVE    )
  CALL_CMD_EXEC( CONNECT   )
  CALL_CMD_EXEC( DISCNCT   )

  CALL_CMD_EXEC( ASGN_GCNF )
  CALL_CMD_EXEC( ASGN_CNF  )
  CALL_CMD_EXEC( ASGN_MEM  )
  CALL_CMD_EXEC( ASGN_END  )
  CALL_CMD_EXEC( EDIT      )
  CALL_CMD_EXEC( EDIT_END  )

  CALL_CMD_EXEC( COMPOSITE )
  CALL_CMD_EXEC( CMP_END   )
  CALL_CMD_EXEC( FUNC_DEF  )
  CALL_CMD_EXEC( FDEF_END  )
  CALL_CMD_EXEC( S_PARAMS  )
  CALL_CMD_EXEC( RETURN    )

  CALL_CMD_EXEC( DESTROY   )

  CALL_CMD_EXEC( INHERIT   )
  CALL_CMD_EXEC( INHERITPR )

  CALL_CMD_EXEC( EXPO_P    )
  CALL_CMD_EXEC( EXPO_P_AS )
  CALL_CMD_EXEC( EXPO_C    )
  CALL_CMD_EXEC( EXPO_C_AS )
  CALL_CMD_EXEC( EXPO_M    )
  CALL_CMD_EXEC( EXPO_M_AS )

  CALL_CMD_EXEC( CTRL_IF     )
  CALL_CMD_EXEC( CTRL_ELSE   )
  CALL_CMD_EXEC( CTRL_END_IF )
  #undef CALL_CMD_EXEC

  default:  FIXME("unknown command code:"<<command);
  }
}
//-------------------------------------------------------------------------------------------

#define IMPL_CMD_EXEC(x_cmd)  void TBinWriter::cmd_##x_cmd (int command, const TBinaryStack &bstack)

IMPL_CMD_EXEC( LINCLUDE  ) // bin=[-]; pop two values(1,2; 1:str,2:id), add str to the lazy-include-list of id;
{
  std::string filename(pop_literal());
  std::string identifier(pop_id());

  out_to_stream()<<"linclude "<<identifier<<" "<<filename<<std::endl;
}
IMPL_CMD_EXEC( DUMP1     ) // bin=[-]; pop two strings(1,2), dump 2 into file 1;
{
  std::string filename(pop_literal()), str1(pop_literal());
  out_to_stream()<<"dump1 "<<str1<<" "<<filename<<std::endl;
}
IMPL_CMD_EXEC( DUMP2     ) // bin=[-]; pop three strings(1,2,3), dump 3,2 into file 1;
{
  std::string filename(pop_literal()), str2(pop_literal()), str1(pop_literal());
  out_to_stream()<<"dump2 "<<str1<<" "<<str2<<" "<<filename<<std::endl;
}

IMPL_CMD_EXEC( MODULE    ) // bin=[-]; pop two identifiers(1,2), create module: type 2, id 1;
{
  std::string identifier(pop_id()), type(pop_id());
  out_to_stream()<<"module "<<type<<" "<<identifier<<std::endl;
}
IMPL_CMD_EXEC( REMOVE    ) // bin=[-]; pop an identifier(1), remove module: id 1;
{
  std::string identifier(pop_id());
  out_to_stream()<<"remove "<<identifier<<std::endl;
}
IMPL_CMD_EXEC( CONNECT   ) // bin=[-]; pop four identifiers(1,2,3,4), connect 4.3-2.1;
{
  std::string port2(pop_id()), module2(pop_id()), port1(pop_id()), module1(pop_id());
  out_to_stream()<<"connect "<<module1<<"."<<port1<<" ,  "<<module2<<"."<<port2<<std::endl;
}
IMPL_CMD_EXEC( DISCNCT   ) // bin=[-]; pop four identifiers(1,2,3,4), disconnect 4.3-2.1;
{
  std::string port2(pop_id()), module2(pop_id()), port1(pop_id()), module1(pop_id());
  out_to_stream()<<"disconnect "<<module1<<"."<<port1<<" ,  "<<module2<<"."<<port2<<std::endl;
}

IMPL_CMD_EXEC( ASGN_GCNF ) // bin=[-]; start assigning to global config;
{
  out_to_stream()<<"config ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( ASGN_CNF  ) // bin=[-]; pop an identifier, start assigning to its config;
{
  std::string identifier(pop_id());
  out_to_stream()<<identifier<<".config ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( ASGN_MEM  ) // bin=[-]; pop an identifier, start assigning to its memory;
{
  std::string identifier(pop_id());
  out_to_stream()<<identifier<<".memory ={"<<std::endl;
  indent_+=2;
}
IMPL_CMD_EXEC( ASGN_END  ) // bin=[-]; end assigning;
{
  --indent_;
  out_to_stream()<<"}"<<std::endl;
  --indent_;
}
IMPL_CMD_EXEC( EDIT      ) // bin=[-]; pop an identifier, start editing it;
{
  std::string identifier(pop_id());
  out_to_stream()<<"edit "<<identifier<<std::endl;
  out_to_stream()<<"{"<<std::endl;
  ++indent_;
}
IMPL_CMD_EXEC( EDIT_END  ) // bin=[-]; end editing;
{
  --indent_;
  out_to_stream()<<"}"<<std::endl;
}

IMPL_CMD_EXEC( COMPOSITE ) // bin=[-]; pop an identifier, start defining a composite module;
{
  std::string module_name(pop_id());
  out_to_stream()<<"composite "<<module_name<<std::endl;
  out_to_stream()<<"{"<<std::endl;
  ++indent_;
}
IMPL_CMD_EXEC( CMP_END   ) // bin=[-]; end defining the composite module;
{
  --indent_;
  out_to_stream()<<"}"<<std::endl;
}
IMPL_CMD_EXEC( FUNC_DEF  ) // bin=[-]; pop identifiers (parameters,func-name), start defining a function;
{
  std::list<std::string> param_list;
  for(var_space::TIdentifier id(pop_id()); id!="@def"; id=pop_id())
    param_list.push_front(id);
  std::string func_name= param_list.front();
  param_list.pop_front();

  std::string delim("");
  std::ostream &os(out_to_stream());
  os<<"def "<<func_name<<"(";
  for(std::list<std::string>::const_iterator itr(param_list.begin()),last(param_list.end());itr!=last;delim=",",++itr)
    os<<delim<<*itr;
  os<<")"<<std::endl;
  out_to_stream()<<"{"<<std::endl;
  ++indent_;
}
IMPL_CMD_EXEC( FDEF_END  ) // bin=[-]; end defining the function;
{
  --indent_;
  out_to_stream()<<"}"<<std::endl;
}
IMPL_CMD_EXEC( S_PARAMS  ) // bin=[-]; start parameters of function-definition;
{
  literal_stack_.push_back(var_space::LiteralId("@def"));
}
IMPL_CMD_EXEC( RETURN    ) // bin=[-]; pop a value, set it as a return value, exit the execution;
{
  std::string ret_val(pop_paren_value());
  out_to_stream()<<"return "<<ret_val<<std::endl;
}

IMPL_CMD_EXEC( DESTROY   ) // bin=[-]; pop two values(1,2; 1:id,2:str), destroy 1 of kind 2;
{
  std::string identifier(pop_id());
  std::string str1(pop_literal());
  out_to_stream()<<"_destroy "<<str1<<" "<<identifier<<std::endl;
}

IMPL_CMD_EXEC( INHERIT   ) // bin=[-]; pop an identifier, inherit the module;
{
  std::string  cmodule_name(pop_id());
  out_to_stream()<<"inherit "<<cmodule_name<<std::endl;
}
IMPL_CMD_EXEC( INHERITPR ) // bin=[-]; pop an identifier, inherit_prv the module;
{
  std::string  cmodule_name(pop_id());
  out_to_stream()<<"inherit_prv "<<cmodule_name<<std::endl;
}

IMPL_CMD_EXEC( EXPO_P    ) // bin=[-]; pop two identifier(1,2), export the port  2.1 as 1;
{
  std::string  port_name(pop_id()), module_name(pop_id());
  out_to_stream()<<"export "<<module_name<<"."<<port_name<<" as_is"<<std::endl;
}
IMPL_CMD_EXEC( EXPO_P_AS ) // bin=[-]; pop three identifier(1,2,3), export the port  3.2 as 1;
{
  std::string  export_name(pop_id()), port_name(pop_id()), module_name(pop_id());
  out_to_stream()<<"export "<<module_name<<"."<<port_name<<" as "<<export_name<<std::endl;
}
IMPL_CMD_EXEC( EXPO_C    ) // bin=[-]; pop two identifier(1,2), export the config  2.config.1 as 1;
{
  std::string  param_name(pop_id()), module_name(pop_id());
  out_to_stream()<<"export "<<module_name<<".config."<<param_name<<" as_is"<<std::endl;
}
IMPL_CMD_EXEC( EXPO_C_AS ) // bin=[-]; pop three identifier(1,2,3), export the config  3.config.2 as 1;
{
  std::string  export_name(pop_id()), param_name(pop_id()), module_name(pop_id());
  out_to_stream()<<"export "<<module_name<<".config."<<param_name<<" as "<<export_name<<std::endl;
}
IMPL_CMD_EXEC( EXPO_M    ) // bin=[-]; pop two identifier(1,2), export the memory  2.memory.1 as 1;
{
  std::string  param_name(pop_id()), module_name(pop_id());
  out_to_stream()<<"export "<<module_name<<".memory."<<param_name<<" as_is"<<std::endl;
}
IMPL_CMD_EXEC( EXPO_M_AS ) // bin=[-]; pop three identifier(1,2,3), export the memory  3.memory.2 as 1;
{
  std::string  export_name(pop_id()), param_name(pop_id()), module_name(pop_id());
  out_to_stream()<<"export "<<module_name<<".memory."<<param_name<<" as "<<export_name<<std::endl;
}

IMPL_CMD_EXEC( CTRL_IF     ) // bin=[- value]; pop a value, if false: skip until finding [ELSE value] or [END_IF value], if true: do nothing;
{
  bstack.ReadI();
  std::string cond= pop_paren_value();
  out_to_stream()<<"if("<<cond<<")"<<std::endl;
  out_to_stream()<<"{"<<std::endl;
  ++indent_;
}
IMPL_CMD_EXEC( CTRL_ELSE   ) // bin=[- value]; skip until finding [END_IF value];
{
  bstack.ReadI();
  --indent_;
  out_to_stream()<<"}"<<std::endl;
  out_to_stream()<<"else"<<std::endl;
  out_to_stream()<<"{"<<std::endl;
  ++indent_;
}
IMPL_CMD_EXEC( CTRL_END_IF ) // bin=[- value]; do nothing;
{
  bstack.ReadI();
  --indent_;
  out_to_stream()<<"}"<<std::endl;
}

#undef IMPL_CMD_EXEC








//===========================================================================================


static void partially_execute(TBinExecutor *executor, TBinaryStack *bin_stack, const std::string& file_name, int line_num, bool error_stat)
{
  bin_stack->GoFirst();
  executor->PartiallyExecute(file_name,line_num,error_stat);
  bin_stack->Clear();
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief load modules, connections, configurations from the file [file_name] */
bool LoadFromFile (const std::string &file_name, TCompositeModule &cmodule, std::list<std::string> *included_list)
//===========================================================================================
{
  boost::filesystem::path file_path
    = boost::filesystem::complete(boost::filesystem::path(cmodule.Agent().SearchFileName(file_name),boost::filesystem::native));

  if (boost::filesystem::exists(file_path) && boost::filesystem::is_empty(file_path))  // if file is empty...
    return true;

  if (included_list!=NULL && std::find(included_list->begin(),included_list->end(),file_path.file_string())==included_list->end())
    included_list->push_back(file_path.file_string());

  TBinExecutor executor;

  TBinaryStack bin_stack;

  executor.PushCmpModule(cmodule);
  executor.SetBinStack(&bin_stack);
  executor.SetLiteralTable(NULL);

  executor.SetCurrentDir(file_path.parent_path());
  executor.SetIncludedList(included_list);
  executor.SetIgnoreExport(false);

  executor.SetPathList(cmodule.Agent().PathListPtr());
  executor.SetCmpModuleGenerator(&cmodule.Agent().CompositeModuleGenerator());
  executor.SetFunctionManager(&cmodule.Agent().FunctionManager());

  TParserCallbacks callbacks;
  callbacks.OnEndOfLine= boost::bind(&partially_execute,&executor,&bin_stack,_1,_2,_3);
  callbacks.OnInclude= boost::bind(&TBinExecutor::OnInclude,&executor,_1,_2,false);
  callbacks.OnIncludeOnce= boost::bind(&TBinExecutor::OnInclude,&executor,_1,_2,true);
  if(ParseFile(file_path.file_string(),bin_stack,callbacks))
  {
    partially_execute(&executor,&bin_stack,file_name,-1,false);
      //! this code is needed if there is no newline at the end of file; \todo FIXME: the line number (-1)
    executor.PopCmpModule();
    LASSERT(executor.CmpModuleStackSize()==0);
    return !executor.Error();
  }
  return false;
}
//-------------------------------------------------------------------------------------------

bool ExecuteBinary (const TBinaryStack &bin_stack, TCompositeModule &cmodule, var_space::TLiteralTable *literal_table, var_space::TLiteral *ret_val, bool ignore_export)
{
  TBinExecutor executor;
  executor.ClearReturnValue();
  executor.PushCmpModule(cmodule);
  executor.SetBinStack(&bin_stack);
  executor.SetLiteralTable(literal_table);

  executor.SetCurrentDir(cmodule.Agent().CurrentDir());
  executor.SetIncludedList(NULL);
  executor.SetIgnoreExport(ignore_export);

  executor.SetPathList(cmodule.Agent().PathListPtr());
  executor.SetCmpModuleGenerator(&cmodule.Agent().CompositeModuleGenerator());
  executor.SetFunctionManager(&cmodule.Agent().FunctionManager());

  executor.Execute(true);

  executor.PopCmpModule();
  LASSERT(executor.CmpModuleStackSize()==0);
  if(ret_val)  *ret_val= executor.ReturnValue();
  return !executor.Error();
}
//-------------------------------------------------------------------------------------------


/*FIXME bool ExecuteScript(const std::string &exec_script,
      const boost::filesystem::path &current_dir, const std::string &file_name,
      TAgentParserInfoIn &in, TAgentParserInfoOut &out) */



//-------------------------------------------------------------------------------------------
}  // end of agent_parser
//-------------------------------------------------------------------------------------------



//===========================================================================================
bool TCompositeModule::LoadFromFile(const std::string &file_name, std::list<std::string> *included_list)
//===========================================================================================
{
  return agent_parser::LoadFromFile(file_name, *this, included_list);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief load modules, connections, configurations from the file [filename] (native path format)
    \param [in,out]included_list  :  included full-path (native) list
    \note  If you use include_once for multiple LoadFromFile, the same included_list should be specified */
bool TAgent::LoadFromFile (const std::string &file_name, std::list<std::string> *included_list)
//===========================================================================================
{
  return agent_parser::LoadFromFile(file_name, Modules(), included_list);
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! Create an instance of cmodule_name; return true for success
bool TCompositeModuleGenerator::Create(TCompositeModule &instance, const std::string &cmodule_name, bool ignore_export) const
//===========================================================================================
{
  const TGeneratorInfo *pgenerator= Generator(cmodule_name);
  if (pgenerator==NULL)  {return false;}

LDEBUG("----------------");
agent_parser::PrintToStream(pgenerator->Binary);
LDEBUG("----------------");
  pgenerator->Binary.GoFirst();
  if(!agent_parser::ExecuteBinary(pgenerator->Binary, instance, /*literal_table=*/NULL, /*ret_val=*/NULL, ignore_export))
  {
    LERROR("failed to instantiate "<<instance.InstanceName()<<" as "<<cmodule_name);
    return false;
  }
  return true;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
bool TFunctionManager::ExecuteFunction(
        const std::string &func_name, const std::list<var_space::TLiteral> &argv,
        TCompositeModule &context_cmodule, var_space::TLiteral *ret_val, bool ignore_export) const
//===========================================================================================
{
  const TFunctionInfo *pfunction= Function(func_name);
  if(pfunction==NULL)  {return false;}

  var_space::TLiteralTable  literal_table;
  if(pfunction->ParamList.size()!=argv.size())
  {
    LERROR("the function "<<func_name<<" takes "
            <<pfunction->ParamList.size()<<" arguments, but given "<<argv.size());
    return false;
  }
  std::list<std::string>::const_iterator  param_itr(pfunction->ParamList.begin());
  for(std::list<var_space::TLiteral>::const_iterator itr(argv.begin()),last(argv.end()); itr!=last; ++itr,++param_itr)
    literal_table.AddLiteral(*param_itr, *itr);

LDEBUG("----------------");
agent_parser::PrintToStream(pfunction->Binary);
LDEBUG("----------------");
  pfunction->Binary.GoFirst();
  if(!agent_parser::ExecuteBinary(pfunction->Binary, context_cmodule, &literal_table, ret_val, ignore_export))
  {
    LERROR("failed to execute function: "<<func_name);
    return false;
  }
  return true;


}
//-------------------------------------------------------------------------------------------

//===========================================================================================
bool TAgent::ExecuteScript(
        const std::string &exec_script, TCompositeModule &context_cmodule, std::list<std::string> *included_list,
        const std::string &file_name, int start_line_num, bool ignore_export)
//===========================================================================================
{
return true;
  // TAgentParserInfoIn  in(context_cmodule);
  // TAgentParserInfoOut out;
  // TAgentParseMode parse_mode;
  // parse_mode.NoExport= ignore_export;
  // in.ParseMode          = parse_mode;
  // in.StartLineNum       = start_line_num;
  // in.PathList           = path_list_;
  // in.IncludedList       = included_list;
  // in.CmpModuleGenerator = &cmp_module_generator_;
  // in.FunctionManager    = &function_manager_;

  // return loco_rabbits::ExecuteScript(exec_script,  boost::filesystem::current_path(), file_name, in, out);
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
