//-------------------------------------------------------------------------------------------
/*! \file    signal_utils.h
    \brief   libskyai - signal/slot utilities
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \version 0.1
    \date    Jun.28, 2010

    Copyright (C) 2010  Akihiko Yamaguchi

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
#ifndef skyai_signal_utils_h
#define skyai_signal_utils_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief just forward a signal (num of args is 0) */
class MForwarder0
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface      TParent;
  typedef MForwarder0           TThis;
  SKYAI_MODULE_NAMES(MForwarder0)

  MForwarder0 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  MAKE_SLOT_PORT(slot_in, void, (void), (), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (void), TThis);

  virtual void slot_in_exec (void)
    {
      signal_out.ExecAll();
    }

};  // end of MForwarder0
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief just forward a signal (num of args is 1) */
template <typename t_arg1>
class MForwarder1
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface      TParent;
  typedef MForwarder1<t_arg1>   TThis;
  SKYAI_MODULE_NAMES(MForwarder1)

  MForwarder1 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  MAKE_SLOT_PORT(slot_in, void, (const t_arg1 &a1), (a1), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (const t_arg1 &), TThis);

  virtual void slot_in_exec (const t_arg1 &a1)
    {
      signal_out.ExecAll(a1);
    }

};  // end of MForwarder1
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief emit the first signal caught at slot_in since reset (for void(void) typed signal) */
class MEmitOnce0
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MEmitOnce0          TThis;
  SKYAI_MODULE_NAMES(MEmitOnce0)

  MEmitOnce0 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      emitted_       (false),
      slot_reset     (*this),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_reset );
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  bool emitted_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_in, void, (void), (), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (void), TThis);

  virtual void slot_reset_exec (void)
    {
      emitted_= false;
    }

  virtual void slot_in_exec (void)
    {
      if (!emitted_)
      {
        signal_out.ExecAll();
        emitted_= true;
      }
    }

};  // end of MEmitOnce0
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief emit the first signal caught at slot_in since reset
    \todo implement a "signature style" rather than \p t_arg1 */
template <typename t_arg1>
class MEmitOnce
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MEmitOnce<t_arg1>   TThis;
  SKYAI_MODULE_NAMES(MEmitOnce)

  MEmitOnce (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      emitted_       (false),
      slot_reset     (*this),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_reset );
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  bool emitted_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_in, void, (const t_arg1 &a1), (a1), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (const t_arg1 &), TThis);

  virtual void slot_reset_exec (void)
    {
      emitted_= false;
    }

  virtual void slot_in_exec (const t_arg1 &a1)
    {
      if (!emitted_)
      {
        signal_out.ExecAll(a1);
        emitted_= true;
      }
    }

};  // end of MEmitOnce
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief emit if the argument of slot_in is true */
class MEmitIf
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MEmitIf             TThis;
  SKYAI_MODULE_NAMES(MEmitIf)

  MEmitIf (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  MAKE_SLOT_PORT(slot_in, void, (const TBool &b), (b), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (void), TThis);

  virtual void slot_in_exec (const TBool &b)
    {
      if (b)
        signal_out.ExecAll();
    }

};  // end of MEmitIf
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief remove the arguments of an input signal and emit it
    \todo implement a "signature style" rather than \p t_arg1 */
template <typename t_arg1>
class MRemoveSignalArguments
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface                TParent;
  typedef MRemoveSignalArguments<t_arg1>  TThis;
  SKYAI_MODULE_NAMES(MRemoveSignalArguments)

  MRemoveSignalArguments (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  MAKE_SLOT_PORT(slot_in, void, (const t_arg1 &a), (a), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (void), TThis);

  virtual void slot_in_exec (const t_arg1 &a)
    {
      signal_out.ExecAll();
    }

};  // end of MRemoveSignalArguments
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief replace the arguments of an input signal by a data obtained from the in-port and emit it */
template <typename t_arg1, typename t_replace1>
class MReplaceSignalArguments
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface             TParent;
  typedef MReplaceSignalArguments
                  <t_arg1,t_replace1>  TThis;
  SKYAI_MODULE_NAMES(MReplaceSignalArguments)

  MReplaceSignalArguments (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      in_replace     (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_in_port     (in_replace );
      add_signal_port (signal_out );
    }

protected:

  MAKE_SLOT_PORT(slot_in, void, (const t_arg1 &a), (a), TThis);
  MAKE_IN_PORT(in_replace, const t_replace1& (void), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (const t_replace1 &), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(replace, const t_replace1&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual void slot_in_exec (const t_arg1 &a)
    {
      signal_out.ExecAll(get_replace());
    }

};  // end of MReplaceSignalArguments
//-------------------------------------------------------------------------------------------
/*!\brief partial specialization to t_arg1==TVoid  */
template <typename t_replace1>
class MReplaceSignalArguments <TVoid, t_replace1>
    : public TModuleInterface
{
public:
  typedef TModuleInterface             TParent;
  typedef MReplaceSignalArguments
                  <TVoid,t_replace1>   TThis;
  SKYAI_MODULE_NAMES(MReplaceSignalArguments)

  MReplaceSignalArguments (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      in_replace     (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_in_port     (in_replace );
      add_signal_port (signal_out );
    }

protected:

  MAKE_SLOT_PORT(slot_in, void, (void), (), TThis);
  MAKE_IN_PORT(in_replace, const t_replace1& (void), TThis);
  MAKE_SIGNAL_PORT(signal_out, void (const t_replace1 &), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(replace, const t_replace1&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual void slot_in_exec (void)
    {
      signal_out.ExecAll(get_replace());
    }

};  // end of MReplaceSignalArguments
//-------------------------------------------------------------------------------------------


//===========================================================================================
struct TSignalCounterMemory
{
  TInt         Count;

  TSignalCounterMemory(var_space::TVariableMap &mmap)
    :
      Count(0)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Count   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief count a signal */
class MSignalCounter
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MSignalCounter     TThis;
  SKYAI_MODULE_NAMES(MSignalCounter)

  MSignalCounter (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      mem_           (TParent::param_box_memory_map()),
      slot_reset     (*this),
      slot_in        (*this),
      out_count      (*this)
    {
      add_slot_port (slot_reset );
      add_slot_port (slot_in    );
      add_out_port  (out_count  );
    }

protected:

  TSignalCounterMemory  mem_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_in, void, (void), (), TThis);

  MAKE_OUT_PORT(out_count, const TInt&, (void), (), TThis);

  void slot_reset_exec (void)
    {
      mem_.Count= 0;
    }
  void slot_in_exec (void)
    {
      ++mem_.Count;
    }
  const TInt& out_count_get (void) const
    {
      return mem_.Count;
    }

};  // end of MSignalCounter
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MForwarder1,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MForwarder1,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MForwarder1,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MForwarder1,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MForwarder1,TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEmitOnce,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEmitOnce,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEmitOnce,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEmitOnce,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TInt,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TReal,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TBool,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MReplaceSignalArguments,TVoid,TString)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_signal_utils_h
//-------------------------------------------------------------------------------------------
