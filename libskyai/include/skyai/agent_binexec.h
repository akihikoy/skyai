//-------------------------------------------------------------------------------------------
/*! \file    agent_binexec.h
    \brief   libskyai - certain program (header)
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
#ifndef skyai_agent_binexec_h
#define skyai_agent_binexec_h
//-------------------------------------------------------------------------------------------
#include <skyai/agent_bindef.h>
#include <skyai/base.h>
#include <skyai/types.h>
//-------------------------------------------------------------------------------------------
#include <lora/variable_binexec.h>
//-------------------------------------------------------------------------------------------
#include <boost/filesystem/path.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace agent_parser
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TBinExecutor : public loco_rabbits::var_space::TBinExecutor
//===========================================================================================
{
public:

  typedef loco_rabbits::var_space::TBinExecutor TParent;

  TBinExecutor()
    : TParent(),
      included_list_(NULL),
      ignore_export_(false),
      path_list_(NULL),
      cmp_module_generator_(NULL),
      function_manager_(NULL)
    {}

  bool OnInclude (const std::string &file_name, std::string &abs_file_name, bool once);

  void PushCmpModule(TCompositeModule &cmod)  {cmodule_stack_.push_back(&cmod);}
  void PopCmpModule()  {cmodule_stack_.pop_back();}
  int  CmpModuleStackSize()  {return cmodule_stack_.size();}

  const var_space::TLiteral& ReturnValue() const {return return_value_;}
  void ClearReturnValue()  {return_value_.Unset();}

  void SetCurrentDir (const boost::filesystem::path &current_dir)  {current_dir_= current_dir;}
  void SetIncludedList (std::list<std::string> *included_list)  {included_list_= included_list;}
  void SetIgnoreExport (bool ignore_export)  {ignore_export_= ignore_export;}

  void SetPathList (std::list<boost::filesystem::path> *path_list)  {path_list_= path_list;}
  void SetCmpModuleGenerator (TCompositeModuleGenerator *cmp_module_generator)  {cmp_module_generator_= cmp_module_generator;}
  void SetFunctionManager (TFunctionManager *function_manager)  {function_manager_= function_manager;}

protected:

  // hide parent's member functions:
  void PushVariable(var_space::TVariable &var);
  void PopVariable();
  int  VariableStackSize();

  enum TExecutionMode {emNormal=0, emFunctionDef, emCompositeDef, emEdit};

  boost::filesystem::path              current_dir_;
  std::list<std::string>               *included_list_;
  bool                                 ignore_export_;

  std::list<boost::filesystem::path>   *path_list_;
  TCompositeModuleGenerator            *cmp_module_generator_;
  TFunctionManager                     *function_manager_;

  std::list<TCompositeModule*>         cmodule_stack_;
  std::list<TExecutionMode>            mode_stack_;
  std::list<TCompositeModule>          cmodule_entity_stack_;

  var_space::TLiteral                  return_value_;

// std::list<std::string>               func_param_stack_;
// TAgentParseMode                      parse_mode_;
// std::set<std::string>                keywords_;

// std::string                          tmp_func_name_;

// TBinaryStack *tmp_bin_stack_;
  std::pair<var_space::TIdentifier,TCompositeModuleGenerator::TGeneratorInfo> tmp_cmod_generator_info_;
  std::pair<var_space::TIdentifier,TFunctionManager::TFunctionInfo> tmp_func_info_;


  std::string pop_string (void)
    {
      var_space::TLiteral value(pop_literal());
      if(!value.IsPrimitive() || value.AsPrimitive().Type()!=var_space::TAnyPrimitive::ptString)
        {error_=true; LERROR("string is required, but used: "<<value); lexit(df);}
      return value.AsPrimitive().String();
    }

  bool search_agent_file (const boost::filesystem::path &file_path, boost::filesystem::path &absolute_path) const;
  void inherit_module (bool no_export);

  inline bool forbidden_in_composite(const std::string &x);
  inline bool forbidden_in_edit(const std::string &x);

  //! call function of identifier func_id with arguments argv, store the return value into ret_val
  override bool function_call(const std::string &func_id, std::list<var_space::TLiteral> &argv, var_space::TLiteral &ret_val);

  override void exec_command(int command, const TBinaryStack &bstack);

  #define DEF_CMD_EXEC(x_cmd)  void cmd_##x_cmd (int command, const TBinaryStack &bstack);
  DEF_CMD_EXEC( LINCLUDE  )
  DEF_CMD_EXEC( DUMP1     )
  DEF_CMD_EXEC( DUMP2     )

  DEF_CMD_EXEC( MODULE    )
  DEF_CMD_EXEC( REMOVE    )
  DEF_CMD_EXEC( CONNECT   )
  DEF_CMD_EXEC( DISCNCT   )

  DEF_CMD_EXEC( ASGN_GCNF )
  DEF_CMD_EXEC( ASGN_CNF  )
  DEF_CMD_EXEC( ASGN_MEM  )
  DEF_CMD_EXEC( ASGN_END  )
  DEF_CMD_EXEC( EDIT      )
  DEF_CMD_EXEC( EDIT_END  )

  DEF_CMD_EXEC( COMPOSITE )
  DEF_CMD_EXEC( CMP_END   )
  DEF_CMD_EXEC( FUNC_DEF  )
  DEF_CMD_EXEC( FDEF_END  )
  DEF_CMD_EXEC( S_PARAMS  )
  DEF_CMD_EXEC( RETURN    )

  DEF_CMD_EXEC( DESTROY   )

  DEF_CMD_EXEC( INHERIT   )
  DEF_CMD_EXEC( INHERITPR )

  DEF_CMD_EXEC( EXPO_P    )
  DEF_CMD_EXEC( EXPO_P_AS )
  DEF_CMD_EXEC( EXPO_C    )
  DEF_CMD_EXEC( EXPO_C_AS )
  DEF_CMD_EXEC( EXPO_M    )
  DEF_CMD_EXEC( EXPO_M_AS )
  #undef DEF_CMD_EXEC

};
//-------------------------------------------------------------------------------------------

inline bool TBinExecutor::forbidden_in_composite(const std::string &x)
{
  if(!mode_stack_.empty() && mode_stack_.back()==emCompositeDef)
  {
    print_error (x+": forbidden within a composite module definition");
    return true;
  }
  return false;
}
inline bool TBinExecutor::forbidden_in_edit(const std::string &x)
{
  if(!mode_stack_.empty() && mode_stack_.back()==emEdit)
  {
    print_error (x+": forbidden within an edit mode");
    return true;
  }
  return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================

/*!\brief load modules, connections, configurations from the file [file_name] */
bool LoadFromFile (const std::string &file_name, TCompositeModule &cmodule, std::list<std::string> *included_list=NULL);

bool ExecuteBinary (const TBinaryStack &bin_stack, TCompositeModule &cmodule, var_space::TLiteralTable *literal_table=NULL, var_space::TLiteral *ret_val=NULL, bool ignore_export=false);



//-------------------------------------------------------------------------------------------
}  // end of agent_parser
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_agent_binexec_h
//-------------------------------------------------------------------------------------------
