//-------------------------------------------------------------------------------------------
/*! \file    variable_binexec.h
    \brief   liblora - certain program (header)
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
#ifndef loco_rabbits_variable_binexec_h
#define loco_rabbits_variable_binexec_h
//-------------------------------------------------------------------------------------------
#include <lora/variable_bindef.h>
#include <lora/variable_space.h>
#include <lora/variable_literal.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace var_space
{
//-------------------------------------------------------------------------------------------


//! add a builtin function (globally affect) whose return value is a void
void AddToBuiltinFunctions_Void(const TIdentifier &func_id, const boost::function<void(TVariableList&)> &f);
//! add a builtin function (globally affect) whose return value is a real
void AddToBuiltinFunctions_Real(const TIdentifier &func_id, const boost::function<void(TVariableList&)> &f);
//! add a builtin function (globally affect) whose return value is a bool
void AddToBuiltinFunctions_Bool(const TIdentifier &func_id, const boost::function<void(TVariableList&)> &f);
//! add a builtin function (globally affect) whose return value is a list
void AddToBuiltinFunctions_List(const TIdentifier &func_id, const boost::function<void(TVariableList&)> &f);
//-------------------------------------------------------------------------------------------


class TExtForwardIterator;

//===========================================================================================
//! extended TVariable class to treat fill-all
class TExtVariable
//===========================================================================================
{
public:
  enum TKind {kUnknown=0,kSingle,kArray};
  TExtVariable () : kind_(kUnknown)  {}
  TExtVariable (TKind k) : kind_(k)  {}
  TExtVariable (TVariable v_var) : kind_(kSingle), entity_(v_var)  {}
  TExtVariable (TExtForwardIterator &first, TExtForwardIterator &last);

  const TVariable& ToVariable() const
    {
      LASSERT(kind_==kSingle);
      return entity_;
    }

  bool IsSingle() const {return kind_==kSingle;}
  bool IsArray() const {return kind_==kArray;}

  inline bool IfDefFunctionCall() const;

  inline void DirectAssign (const TVariable &value);
  inline void SetMember (const TVariable &id, const TVariable &value);
  inline TExtVariable GetMember (const TVariable &id);
  inline void FunctionCall (const TIdentifier &id, TVariableList &argv);
  inline bool FunctionExists(const TIdentifier &id) const;
  inline void DirectCall (TVariableList &argv);
  inline TExtVariable Push (void);
  inline void GetBegin (TExtForwardIterator &res);
  inline void GetEnd (TExtForwardIterator &res);

private:

  TKind                  kind_;
  TVariable              entity_;
  std::list<TVariable>   array_;

  friend class TExtForwardIterator;

};
//-------------------------------------------------------------------------------------------

//! extended TVariable class to treat fill-all
class TExtForwardIterator
{
public:
  enum TKind {kUnknown=0,kSingle,kArray};

  TExtForwardIterator() : kind_(kUnknown) {}

  TExtVariable& operator*(void)  {i_dereference_(); return dereferenced_;}
  TExtVariable* operator->(void)  {i_dereference_(); return &dereferenced_;}
  const TExtForwardIterator& operator++(void);
  const TExtForwardIterator& operator--(void);
  bool operator==(const TExtForwardIterator &rhs) const;
  bool operator!=(const TExtForwardIterator &rhs) const {return !operator==(rhs);}

private:
  TKind                          kind_;
  TForwardIterator               entity_;
  std::list<TForwardIterator>    array_;

  TExtVariable                   dereferenced_;
  void i_dereference_();

  friend class TExtVariable;
};
//-------------------------------------------------------------------------------------------


inline bool TExtVariable::IfDefFunctionCall() const
{
  switch(kind_)
  {
  case kSingle :  return entity_.IfDefFunctionCall();
  case kArray :   for (std::list<TVariable>::const_iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    if(!itr->IfDefFunctionCall())  return false;
                  return true;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return false;
}
inline void TExtVariable::DirectAssign (const TVariable &value)
{
  switch(kind_)
  {
  case kSingle :  return entity_.DirectAssign(value);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->DirectAssign(value);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
inline void TExtVariable::SetMember (const TVariable &id, const TVariable &value)
{
  switch(kind_)
  {
  case kSingle :  return entity_.SetMember(id, value);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->SetMember(id, value);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
inline TExtVariable TExtVariable::GetMember (const TVariable &id)
{
  switch(kind_)
  {
  case kSingle :  return TExtVariable(entity_.GetMember(id));
  case kArray :   {TExtVariable res(kArray);
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    res.array_.push_back(itr->GetMember(id));
                  return res;}
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return TExtVariable();
}
inline void TExtVariable::FunctionCall (const TIdentifier &id, TVariableList &argv)
{
  switch(kind_)
  {
  case kSingle :  return entity_.FunctionCall(id,argv);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->FunctionCall(id,argv);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
inline bool TExtVariable::FunctionExists(const TIdentifier &id) const
{
  switch(kind_)
  {
  case kSingle :  return entity_.FunctionExists(id);
  case kArray :   for (std::list<TVariable>::const_iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    if(!itr->FunctionExists(id))  return false;
                  return true;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return false;
}
inline void TExtVariable::DirectCall (TVariableList &argv)
{
  switch(kind_)
  {
  case kSingle :  return entity_.DirectCall(argv);
  case kArray :   for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    itr->DirectCall(argv);
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
inline TExtVariable TExtVariable::Push (void)
{
  switch(kind_)
  {
  case kSingle :  return entity_.Push();
  case kArray :   {TExtVariable res(kArray);
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    res.array_.push_back(itr->Push());
                  return res;}
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
  return TExtVariable();
}
inline void TExtVariable::GetBegin (TExtForwardIterator &res)
{
  switch(kind_)
  {
  case kSingle :  res.kind_= TExtForwardIterator::kSingle;
                  entity_.GetBegin(res.entity_);
                  return;
  case kArray :   res.kind_= TExtForwardIterator::kArray;
                  res.array_.clear();
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    {res.array_.push_back(TForwardIterator()); itr->GetBegin(res.array_.back());}
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
inline void TExtVariable::GetEnd (TExtForwardIterator &res)
{
  switch(kind_)
  {
  case kSingle :  res.kind_= TExtForwardIterator::kSingle;
                  entity_.GetEnd(res.entity_);
                  return;
  case kArray :   res.kind_= TExtForwardIterator::kArray;
                  res.array_.clear();
                  for (std::list<TVariable>::iterator itr(array_.begin()),last(array_.end()); itr!=last; ++itr)
                    {res.array_.push_back(TForwardIterator()); itr->GetEnd(res.array_.back());}
                  return;
  default: LERROR("fatal! (internal error)"); lexit(df);
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TBinExecutor
//===========================================================================================
{
public:

  TBinExecutor() : bin_stack_(NULL), literal_table_(NULL), error_(false) {}

  virtual void PartiallyExecute(const std::string& file_name, int line_num, bool error_stat);

  virtual void Execute(bool from_current=false);

  void PushVariable(TExtVariable &var)  {variable_stack_.push_back(var); update_keyword_this();}
  void PushVariable(TVariable &var)  {variable_stack_.push_back(TExtVariable(var)); update_keyword_this();}
  void PopVariable()  {variable_stack_.pop_back(); update_keyword_this();}
  int  VariableStackSize()  {return variable_stack_.size();}

  const TBinaryStack& BinStack() const {return *bin_stack_;}
  const TLiteralTable& LiteralTable() const {return *literal_table_;}

  void SetBinStack(const TBinaryStack *p)  {bin_stack_= p;}
  void SetLiteralTable(TLiteralTable *p)  {literal_table_= p;}

  const std::string& FileName() const {return file_name_;}
  int LineNum() const {return line_num_;}
  bool Error() const {return error_;}

protected:

  const TBinaryStack *bin_stack_;

  TLiteralTable  *literal_table_;

  std::list<TLiteral> literal_stack_;
  std::list<TExtVariable>  variable_stack_;

  std::string  file_name_;
  int  line_num_;
  bool error_;


  void update_keyword_this()
    {
      if(literal_table_ && !variable_stack_.empty())
      {
        if(variable_stack_.back().IsSingle())
          literal_table_->AddLiteral(VAR_SPACE_KEYWORD_THIS, TLiteral(variable_stack_.back().ToVariable()));
        else
          literal_table_->RemoveLiteral(VAR_SPACE_KEYWORD_THIS);
      }
    }

  TLiteral evaluate_literal (const TLiteral &src, const TEvaluateLiteralConfig &config=TEvaluateLiteralConfig())
    {
      return EvaluateLiteral(src, literal_table_, config, error_);
    }

  TLiteral pop_literal (const TEvaluateLiteralConfig &config=TEvaluateLiteralConfig())
    {
      LASSERT(!literal_stack_.empty());
      TLiteral value= literal_stack_.back();
      literal_stack_.pop_back();
      LASSERT(!value.IsCommand());
      return evaluate_literal(value, config);
    }

  void pop_literal_list(std::list<TLiteral> &literal_list, const TEvaluateLiteralConfig &config=TEvaluateLiteralConfig())
    {
      std::list<TLiteral>::iterator  ilast(literal_stack_.end()),ifirst(literal_stack_.begin());
      std::list<TLiteral>::iterator  itr(ilast);
      LASSERT(ifirst!=ilast);
      for(--itr; itr!=ifirst && !itr->IsCommand(bin::cmd::LLISTS); --itr) {}
      LASSERT(itr->IsCommand(bin::cmd::LLISTS));
      std::list<TLiteral>::iterator  itr2(itr);
      for(++itr; itr!=ilast; ++itr)
        literal_list.push_back(evaluate_literal(*itr, config));
      literal_stack_.erase(itr2,ilast);
    }

  std::string pop_id (void)
    {
      TEvaluateLiteralConfig config;  config.AllowId= true;
      TLiteral id(pop_literal(config));
      if(!id.IsIdentifier())  {LERROR("identifier is required, but used: "<<id); lexit(df);}
      return id.AsIdentifier();
    }

  void print_error (const std::string &str)
    {
      error_= true;
      std::cerr<<"("<<file_name_<<":"<<line_num_<<") "<<str<<std::endl;
    }


  //! call function of identifier func_id with arguments argv, store the return value into ret_val
  virtual bool function_call(const std::string &func_id, std::list<TLiteral> &argv, TLiteral &ret_val);

  //! access to the member of value
  virtual TVariable member_access(const TLiteral &value, const TLiteral &member_c);

  virtual void exec_command(int command, const TBinaryStack &bstack);

  #define DEF_CMD_EXEC(x_cmd)  void cmd_##x_cmd (int command, const TBinaryStack &bstack);
  DEF_CMD_EXEC( PUSH      )
  DEF_CMD_EXEC( PUSHL     )
  DEF_CMD_EXEC( LAPPEND   )
  DEF_CMD_EXEC( PUSH_EMPL )
  DEF_CMD_EXEC( LLISTS    )
  DEF_CMD_EXEC( POP       )

  DEF_CMD_EXEC( PRINT     )

  DEF_CMD_EXEC( M_ASGN_P  )
  DEF_CMD_EXEC( M_ASGN_CS )
  DEF_CMD_EXEC( E_ASGN_P  )
  DEF_CMD_EXEC( E_ASGN_CS )
  DEF_CMD_EXEC( P_ASGN_P  )
  DEF_CMD_EXEC( P_ASGN_CS )
  DEF_CMD_EXEC( F_ASGN_P  )
  DEF_CMD_EXEC( F_ASGN_CS )
  DEF_CMD_EXEC( CASGN_END )

  DEF_CMD_EXEC( FUNC_CALL )

  DEF_CMD_EXEC( CONCAT )
  DEF_CMD_EXEC( ADD    )
  DEF_CMD_EXEC( SUBT   )
  DEF_CMD_EXEC( MULT   )
  DEF_CMD_EXEC( DIV    )
  DEF_CMD_EXEC( MOD    )
  DEF_CMD_EXEC( AND    )
  DEF_CMD_EXEC( OR     )
  DEF_CMD_EXEC( NOT    )
  DEF_CMD_EXEC( EQ     )
  DEF_CMD_EXEC( NEQ    )
  DEF_CMD_EXEC( LTEQ   )
  DEF_CMD_EXEC( GTEQ   )
  DEF_CMD_EXEC( LT     )
  DEF_CMD_EXEC( GT     )
  DEF_CMD_EXEC( MEMBER )
  DEF_CMD_EXEC( ELEM   )

  DEF_CMD_EXEC( CAST   )

  DEF_CMD_EXEC( T_TO_LIST )
  #undef DEF_CMD_EXEC

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
class TBinWriter
//===========================================================================================
{
public:

  TBinWriter() : bin_stack_(NULL), out_stream_(NULL), indent_(0), error_(false) {}

  virtual void PartiallyExecute(const std::string& file_name, int line_num, bool error_stat);

  virtual void Execute(bool from_current=false);

  const TBinaryStack& BinStack() const {return *bin_stack_;}
  std::ostream& OutStream() const {return *out_stream_;}
  int  Indent() const {return indent_;}

  void SetBinStack(const TBinaryStack *p)  {bin_stack_= p;}
  void SetOutStream(std::ostream *p)  {out_stream_= p;}
  void SetIndent(int i)  {indent_= i;}

  const std::string& FileName() const {return file_name_;}
  int LineNum() const {return line_num_;}
  bool Error() const {return error_;}

protected:

  const TBinaryStack *bin_stack_;
  std::ostream *out_stream_;

  std::list<TLiteral> literal_stack_;

  int indent_;

  std::string  file_name_;
  int  line_num_;
  bool error_;


  std::ostream& out_to_stream()
    {
      LASSERT(out_stream_!=NULL);
      for(int i(0);i<indent_;++i)  *out_stream_<<"  ";
      return *out_stream_;
    }

  std::string pop_literal ()
    {
      LASSERT(!literal_stack_.empty());
      TLiteral value= literal_stack_.back();
      literal_stack_.pop_back();
      LASSERT((value.IsIdentifier() || value.IsPrimitive()) && value.AsPrimitive().IsString());
      return value.AsPrimitive().String();
    }

  void pop_literal_list(std::list<std::string> &literal_list)
    {
      std::list<TLiteral>::iterator  ilast(literal_stack_.end()),ifirst(literal_stack_.begin());
      std::list<TLiteral>::iterator  itr(ilast);
      LASSERT(ifirst!=ilast);
      for(--itr; itr!=ifirst && !itr->IsCommand(bin::cmd::LLISTS); --itr) {}
      LASSERT(itr->IsCommand(bin::cmd::LLISTS));
      std::list<TLiteral>::iterator  itr2(itr);
      for(++itr; itr!=ilast; ++itr)
      {
        LASSERT((itr-> IsIdentifier() || itr->IsPrimitive()) && itr->AsPrimitive().IsString());
        literal_list.push_back("("+itr->AsPrimitive().String()+")");
      }
      literal_stack_.erase(itr2,ilast);
    }

  std::string pop_id (void)
    {
      return pop_literal();
    }

  std::string pop_paren_value (void)
    {
      return "("+pop_literal()+")";
    }

  void print_error (const std::string &str)
    {
      error_= true;
      std::cerr<<"("<<file_name_<<":"<<line_num_<<") "<<str<<std::endl;
    }

  virtual void exec_command(int command, const TBinaryStack &bstack);

  #define DEF_CMD_EXEC(x_cmd)  void cmd_##x_cmd (int command, const TBinaryStack &bstack);
  DEF_CMD_EXEC( PUSH      )
  DEF_CMD_EXEC( PUSHL     )
  DEF_CMD_EXEC( LAPPEND   )
  DEF_CMD_EXEC( PUSH_EMPL )
  DEF_CMD_EXEC( LLISTS    )
  DEF_CMD_EXEC( POP       )

  DEF_CMD_EXEC( PRINT     )

  DEF_CMD_EXEC( M_ASGN_P  )
  DEF_CMD_EXEC( M_ASGN_CS )
  DEF_CMD_EXEC( E_ASGN_P  )
  DEF_CMD_EXEC( E_ASGN_CS )
  DEF_CMD_EXEC( P_ASGN_P  )
  DEF_CMD_EXEC( P_ASGN_CS )
  DEF_CMD_EXEC( F_ASGN_P  )
  DEF_CMD_EXEC( F_ASGN_CS )
  DEF_CMD_EXEC( CASGN_END )

  DEF_CMD_EXEC( FUNC_CALL )

  DEF_CMD_EXEC( CONCAT )
  DEF_CMD_EXEC( ADD    )
  DEF_CMD_EXEC( SUBT   )
  DEF_CMD_EXEC( MULT   )
  DEF_CMD_EXEC( DIV    )
  DEF_CMD_EXEC( MOD    )
  DEF_CMD_EXEC( AND    )
  DEF_CMD_EXEC( OR     )
  DEF_CMD_EXEC( NOT    )
  DEF_CMD_EXEC( EQ     )
  DEF_CMD_EXEC( NEQ    )
  DEF_CMD_EXEC( LTEQ   )
  DEF_CMD_EXEC( GTEQ   )
  DEF_CMD_EXEC( LT     )
  DEF_CMD_EXEC( GT     )
  DEF_CMD_EXEC( MEMBER )
  DEF_CMD_EXEC( ELEM   )

  DEF_CMD_EXEC( CAST   )

  DEF_CMD_EXEC( T_TO_LIST )
  #undef DEF_CMD_EXEC

};
//-------------------------------------------------------------------------------------------


//===========================================================================================

bool LoadFromFile (const std::string &file_name, TVariable &var, TLiteralTable &literal_table);

bool ExecuteBinary (const TBinaryStack &bin_stack, TVariable &var, TLiteralTable &literal_table);


//-------------------------------------------------------------------------------------------
}  // end of var_space
}  // end of loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_variable_binexec_h
//-------------------------------------------------------------------------------------------
