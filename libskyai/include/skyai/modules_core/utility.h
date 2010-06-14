//-------------------------------------------------------------------------------------------
/*! \file    utility.h
    \brief   libskyai - misc. utility modules (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Dec.18, 2009-

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
#ifndef skyai_utility_h
#define skyai_utility_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <lora/stl_ext.h>   // for max_element_index, min_element_index
#include <lora/stl_math.h>   // for operator+ for TIntVector
#include <lora/small_classes.h>   // for TGridGenerator
#include <lora/string_list_ext.h>   // for saving/loading map
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
// template utilities
//===========================================================================================

//-------------------------------------------------------------------------------------------
namespace utility_detail
{
//-------------------------------------------------------------------------------------------

//! make a map [index --> one of connected ports] from lowerport_to_indexes. index should be in {0,..,max_index-1}
template<typename t_port_for_lower>
void MakeLowerPortsIndexConnectionIteratorMap (
    t_port_for_lower  &port_for_lower,
    std::vector<typename t_port_for_lower::TConnectedPortIterator >  &lower_modules_citr,
    TInt max_index, const std::map<TString, TString>  &lowerport_to_indexes)
{
  int lower_size (max_index);
  lower_modules_citr.clear();
  lower_modules_citr.resize (lower_size);

  std::vector<TInt>  indexes_of_lower;
  std::map<TString, TString>::const_iterator  order_itr;
  for (typename t_port_for_lower::TConnectedPortIterator citr(port_for_lower.ConnectedPortBegin()); citr!=port_for_lower.ConnectedPortEnd(); ++citr)
  {
    order_itr= lowerport_to_indexes.find ((*citr)->UniqueCode());
    if (order_itr != lowerport_to_indexes.end())
    {
      indexes_of_lower= ConvertFromStr<std::vector<TInt> >(order_itr->second);
      for (std::vector<TInt>::const_iterator itr(indexes_of_lower.begin()); itr!=indexes_of_lower.end(); ++itr)
        lower_modules_citr[*itr]= citr;
    }
    else
      {LERROR("module "<<(*citr)->UniqueCode()<<" should described in lowerport_to_indexes"); lexit(df);}
  }
}
//-------------------------------------------------------------------------------------------

template <typename t_value> inline void ApplyConstraintMax (t_value &value, const t_value &max);
template <> inline void ApplyConstraintMax (TInt &value, const TInt &max)
{
  if (value>max)  value= max;
}
template <> inline void ApplyConstraintMax (TReal &value, const TReal &max)
{
  if (value>max)  value= max;
}
template <> inline void ApplyConstraintMax (TIntVector &value, const TIntVector &max)
{
  LASSERT1op1(GenSize(value),==,GenSize(max));
  TypeExt<TIntVector>::const_iterator mitr(GenBegin(max));
  for (TypeExt<TIntVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr>*mitr)  *itr= *mitr;
}
template <> inline void ApplyConstraintMax (TRealVector &value, const TRealVector &max)
{
  LASSERT1op1(GenSize(value),==,GenSize(max));
  TypeExt<TRealVector>::const_iterator mitr(GenBegin(max));
  for (TypeExt<TRealVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr>*mitr)  *itr= *mitr;
}
//-------------------------------------------------------------------------------------------
template <typename t_value> inline void ApplyConstraintMin (t_value &value, const t_value &min);
template <> inline void ApplyConstraintMin (TInt &value, const TInt &min)
{
  if (value<min)  value= min;
}
template <> inline void ApplyConstraintMin (TReal &value, const TReal &min)
{
  if (value<min)  value= min;
}
template <> inline void ApplyConstraintMin (TIntVector &value, const TIntVector &min)
{
  LASSERT1op1(GenSize(value),==,GenSize(min));
  TypeExt<TIntVector>::const_iterator mitr(GenBegin(min));
  for (TypeExt<TIntVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr<*mitr)  *itr= *mitr;
}
template <> inline void ApplyConstraintMin (TRealVector &value, const TRealVector &min)
{
  LASSERT1op1(GenSize(value),==,GenSize(min));
  TypeExt<TRealVector>::const_iterator mitr(GenBegin(min));
  for (TypeExt<TRealVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr<*mitr)  *itr= *mitr;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of utility_detail
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief get a max/min element value/index of an input vector */
// template <typename t_arg1>
class MMinMaxElementRv
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  // typedef t_arg1               TArgument1;
  typedef MMinMaxElementRv     TThis;
  SKYAI_MODULE_NAMES(MMinMaxElementRv)

  MMinMaxElementRv (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_1              (*this),
      out_max           (*this),
      out_max_value     (*this),
      out_max_index     (*this),
      out_min           (*this),
      out_min_value     (*this),
      out_min_index     (*this)
    {
      add_in_port   (in_1          );
      add_out_port  (out_max       );
      add_out_port  (out_max_value );
      add_out_port  (out_max_index );
      add_out_port  (out_min       );
      add_out_port  (out_min_value );
      add_out_port  (out_min_index );
    }

protected:

  mutable TReal  tmp_value_; //!< \warning problematic for multithread mode
  mutable TInt  tmp_index_; //!< \warning ditto

  //!\brief input the current state
  MAKE_IN_PORT(in_1, const TRealVector& (void), TThis);

  MAKE_OUT_PORT(out_max, void, (TReal &value, TInt &index), (value,index), TThis);
  MAKE_OUT_PORT(out_max_value, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_max_index, const TInt&, (void), (), TThis);

  MAKE_OUT_PORT(out_min, void, (TReal &value, TInt &index), (value,index), TThis);
  MAKE_OUT_PORT(out_min_value, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_min_index, const TInt&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(1, const TRealVector&, (void), ())

  #undef GET_FROM_IN_PORT

  void  out_max_get (TReal &value, TInt &index) const
    {
      index= max_element_index(GenBegin(get_1()),GenEnd(get_1()));
      value= GenAt(get_1(),index);
    }
  const TReal&  out_max_value_get (void) const
    {
      tmp_value_= *std::max_element(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_value_;
    }
  const TInt&  out_max_index_get (void) const
    {
      tmp_index_= max_element_index(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_index_;
    }

  void  out_min_get (TReal &value, TInt &index) const
    {
      index= min_element_index(GenBegin(get_1()),GenEnd(get_1()));
      value= GenAt(get_1(),index);
    }
  const TReal&  out_min_value_get (void) const
    {
      tmp_value_= *std::min_element(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_value_;
    }
  const TInt&  out_min_index_get (void) const
    {
      tmp_index_= min_element_index(GenBegin(get_1()),GenEnd(get_1()));
      return tmp_index_;
    }

};  // end of MMinMaxElementRv
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MNonzeroElements
template <typename t_value>
class TNonzeroElementsConfigurations
//===========================================================================================
{
public:

  typename TypeExt<t_value>::value_type    Threshold;

  TNonzeroElementsConfigurations (var_space::TVariableMap &mmap)
    :
      Threshold(0.0l)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Threshold      );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief get nonzero elements */
template <typename t_value>
class MNonzeroElements
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  // typedef t_arg1               TArgument1;
  typedef MNonzeroElements     TThis;
  SKYAI_MODULE_NAMES(MNonzeroElements)

  MNonzeroElements (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      in_1              (*this),
      out_value         (*this),
      out_count         (*this)
    {
      add_in_port   (in_1       );
      add_out_port  (out_value  );
      add_out_port  (out_count  );
    }

protected:

  TNonzeroElementsConfigurations<t_value>  conf_;

  mutable TInt count_;
  mutable t_value nonzero_;

  //!\brief input the current state
  MAKE_IN_PORT(in_1, const t_value& (void), TThis);

  MAKE_OUT_PORT(out_value, const t_value&, (void), (), TThis);

  MAKE_OUT_PORT(out_count, const TInt&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(1, const t_value&, (void), ())

  #undef GET_FROM_IN_PORT

  const t_value&  out_value_get (void) const
    {
      GenResize(nonzero_,out_count_get());
      typename TypeExt<t_value>::iterator  nzitr(GenBegin(nonzero_));
      for (typename TypeExt<t_value>::const_iterator itr(GenBegin(get_1())), end(GenEnd(get_1())); itr!=end; ++itr)
        if (*itr>conf_.Threshold || *itr<-conf_.Threshold)  {*nzitr= *itr; ++nzitr;}
      return nonzero_;
    }

  const TInt&  out_count_get (void) const
    {
      count_= 0;
      for (typename TypeExt<t_value>::const_iterator itr(GenBegin(get_1())), end(GenEnd(get_1())); itr!=end; ++itr)
        if (*itr>conf_.Threshold || *itr<-conf_.Threshold)  ++count_;
      return count_;
    }

};  // end of MNonzeroElements
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
  typedef TModuleInterface        TParent;
  typedef MRemoveSignalArguments  TThis;
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
//!\brief Configurations of MSignalDistributor*
class TSignalDistributorConfigurations
//===========================================================================================
{
public:

  // for organization
  TInt                  SizeOfLowers;

  typedef  std::map<TString, TString>  TLowerportToIndexesMap;
  TLowerportToIndexesMap    IndexesOfLowers;  //!< map of (UniqueCode of lower port -> set of index). index should be in {0,..,SizeOfLowers-1}

  TSignalDistributorConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( SizeOfLowers           );
      ADD( IndexesOfLowers        );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief distribute the input signal to one of the connected port which is specified with the input signal (base class) */
class MSignalDistributorBase
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface             TParent;
  typedef MSignalDistributorBase       TThis;
  SKYAI_MODULE_NAMES(MSignalDistributorBase)

  MSignalDistributorBase (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      slot_reset     (*this),
      out_lower_size (*this)
    {
      add_slot_port   (slot_reset );
      add_out_port    (out_lower_size);
    }

protected:

  TSignalDistributorConfigurations conf_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_OUT_PORT(out_lower_size, const TInt&, (void), (), TThis);

  mutable TInt tmp_lsize_;
  virtual const TInt& out_lower_size_get (void) const
    {
      return (tmp_lsize_= conf_.SizeOfLowers);
    }

  virtual void slot_reset_exec (void) = 0;

};  // end of MSignalDistributorBase
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief distribute the input signal to one of the connected port which is specified with the input signal (arguments of signal is void) */
class MSignalDistributor0
    : public MSignalDistributorBase
//===========================================================================================
{
public:
  typedef MSignalDistributorBase       TParent;
  typedef MSignalDistributor0          TThis;
  SKYAI_MODULE_NAMES(MSignalDistributor0)

  MSignalDistributor0 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  //!\brief input signal by this slot. index indicates a module into which this module emits the signal
  MAKE_SLOT_PORT(slot_in, void, (const TInt &index), (index), TThis);

  MAKE_SIGNAL_PORT(signal_out, void (void), TThis);

  std::vector<GET_PORT_TYPE(signal_out)::TConnectedPortIterator >  lower_modules_citr_;

  override void slot_reset_exec (void)
    {
      using namespace utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (signal_out, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
    }

  virtual void slot_in_exec (const TInt &index)
    {
      LASSERT1op1(index,>=,0);
      LASSERT1op1(index,<,static_cast<int>(GenSize(lower_modules_citr_)));
      signal_out.ExecCurrent (lower_modules_citr_[index]);
    }

};  // end of MSignalDistributor0
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief distribute the input signal to one of the connected port which is specified with the input signal */
template <typename t_arg1>
class MSignalDistributor1
    : public MSignalDistributorBase
//===========================================================================================
{
public:
  typedef MSignalDistributorBase       TParent;
  typedef MSignalDistributor1<t_arg1>  TThis;
  SKYAI_MODULE_NAMES(MSignalDistributor1)

  MSignalDistributor1 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      slot_in        (*this),
      signal_out     (*this)
    {
      add_slot_port   (slot_in    );
      add_signal_port (signal_out );
    }

protected:

  //!\brief input signal by this slot. index indicates a module into which this module emits the signal
  MAKE_SLOT_PORT(slot_in, void, (const TInt &index, const t_arg1 &a1), (index,a1), TThis);

  MAKE_SIGNAL_PORT(signal_out, void (const t_arg1 &a1), TThis);

  std::vector<typename GET_PORT_TYPE(signal_out)::TConnectedPortIterator >  lower_modules_citr_;

  override void slot_reset_exec (void)
    {
      using namespace utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (signal_out, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
    }

  virtual void slot_in_exec (const TInt &index, const t_arg1 &a1)
    {
      LASSERT1op1(index,>=,0);
      LASSERT1op1(index,<,static_cast<int>(GenSize(lower_modules_citr_)));
      signal_out.ExecCurrent (lower_modules_citr_[index],a1);
    }

};  // end of MSignalDistributor1
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MFunctionSelector*
class TFunctionSelectorConfigurations
//===========================================================================================
{
public:

  // for organization
  TInt                  SizeOfLowers;

  typedef  std::map<TString, TString>  TLowerportToIndexesMap;
  TLowerportToIndexesMap    IndexesOfLowers;  //!< map of (UniqueCode of lower port -> set of index). index should be in {0,..,SizeOfLowers-1}

  TFunctionSelectorConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( SizeOfLowers        );
      ADD( IndexesOfLowers     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief select one of the connected in-ports (lowers) which is specified by the argument of the out-port (base class) */
class MFunctionSelectorBase
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface            TParent;
  typedef MFunctionSelectorBase       TThis;
  SKYAI_MODULE_NAMES(MFunctionSelectorBase)

  MFunctionSelectorBase (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      slot_reset     (*this),
      out_lower_size (*this)
    {
      add_slot_port   (slot_reset );
      add_out_port    (out_lower_size);
    }

protected:

  TFunctionSelectorConfigurations conf_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_OUT_PORT(out_lower_size, const TInt&, (void), (), TThis);

  virtual const TInt& out_lower_size_get (void) const
    {
      return conf_.SizeOfLowers;
    }

  virtual void slot_reset_exec (void) = 0;

};  // end of MFunctionSelectorBase
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief select one of the connected in-ports (lowers) which is specified by the argument of the out-port (argument of in-ports is void) */
template <typename t_ret>
class MFunctionSelector0
    : public MFunctionSelectorBase
//===========================================================================================
{
public:
  typedef MFunctionSelectorBase       TParent;
  typedef MFunctionSelector0<t_ret>   TThis;
  SKYAI_MODULE_NAMES(MFunctionSelector0)

  MFunctionSelector0 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      out_function   (*this),
      in_function    (*this)
    {
      add_out_port (out_function );
      add_in_port  (in_function  );
    }

protected:

  //!\brief output function. actual function is one of in_function specified by the index
  MAKE_OUT_PORT(out_function, const t_ret&, (const TInt &index), (index), TThis);

  MAKE_IN_PORT_SPECIFIC(in_function, const t_ret& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  std::vector<typename GET_PORT_TYPE(in_function)::TConnectedPortIterator >  lower_modules_citr_;

  override void slot_reset_exec (void)
    {
      using namespace utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (in_function, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
    }

  virtual const t_ret& out_function_get (const TInt &index) const
    {
      LASSERT1op1(index,>=,0);
      LASSERT1op1(index,<,static_cast<int>(GenSize(lower_modules_citr_)));
      return in_function.GetCurrent (lower_modules_citr_[index]);
    }

};  // end of MFunctionSelector0
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief select one of the connected in-ports (lowers) which is specified by the argument of the out-port (argument of in-ports is t_arg1) */
template <typename t_ret, typename t_arg1>
class MFunctionSelector1
    : public MFunctionSelectorBase
//===========================================================================================
{
public:
  typedef MFunctionSelectorBase             TParent;
  typedef MFunctionSelector1<t_ret,t_arg1>  TThis;
  SKYAI_MODULE_NAMES(MFunctionSelector1)

  MFunctionSelector1 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      out_function   (*this),
      in_function    (*this)
    {
      add_out_port (out_function );
      add_in_port  (in_function  );
    }

protected:

  //!\brief output function. actual function is one of in_function specified by the index
  MAKE_OUT_PORT(out_function, const t_ret&, (const TInt &index, const t_arg1 &a1), (index,a1), TThis);

  MAKE_IN_PORT_SPECIFIC(in_function, const t_ret& (const t_arg1 &a1), TThis, SKYAI_CONNECTION_SIZE_MAX);

  std::vector<typename GET_PORT_TYPE(in_function)::TConnectedPortIterator >  lower_modules_citr_;

  override void slot_reset_exec (void)
    {
      using namespace utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (in_function, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
    }

  virtual const t_ret& out_function_get (const TInt &index, const t_arg1 &a1) const
    {
      LASSERT1op1(index,>=,0);
      LASSERT1op1(index,<,static_cast<int>(GenSize(lower_modules_citr_)));
      return in_function.GetCurrent (lower_modules_citr_[index],a1);
    }

};  // end of MFunctionSelector1
//-------------------------------------------------------------------------------------------
/*!\brief select one of the connected in-ports (lowers) which is specified by the argument of the out-port (t_ret is (partially) specialized to void)
    \note the following code is almost the same as the MFunctionSelector1; differences are only about t_ret */
template <typename t_arg1>
class MFunctionSelector1 <void, t_arg1>
    : public MFunctionSelectorBase
{
public:
  typedef MFunctionSelectorBase             TParent;
  typedef MFunctionSelector1<void,t_arg1>   TThis;
  SKYAI_MODULE_NAMES(MFunctionSelector1)

  MFunctionSelector1 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      out_function   (*this),
      in_function    (*this)
    {
      add_out_port (out_function );
      add_in_port  (in_function  );
    }

protected:

  //!\brief output function. actual function is one of in_function specified by the index
  MAKE_OUT_PORT(out_function, void, (const TInt &index, const t_arg1 &a1), (index,a1), TThis);

  MAKE_IN_PORT_SPECIFIC(in_function, void (const t_arg1 &a1), TThis, SKYAI_CONNECTION_SIZE_MAX);

  std::vector<typename GET_PORT_TYPE(in_function)::TConnectedPortIterator >  lower_modules_citr_;

  override void slot_reset_exec (void)
    {
      using namespace utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (in_function, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
    }

  virtual void out_function_get (const TInt &index, const t_arg1 &a1) const
    {
      LASSERT1op1(index,>=,0);
      LASSERT1op1(index,<,static_cast<int>(GenSize(lower_modules_citr_)));
      return in_function.GetCurrent (lower_modules_citr_[index],a1);
    }

};  // end of MFunctionSelector1 <void,t_arg1>
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief return signal to the same module;
    slot_inN (a1) --> signal_in (a1) --> ... --> slot_return () --> signal_returnN () */
template <typename t_arg1>
class MReturnToSameModule10
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface               TParent;
  typedef MReturnToSameModule10<t_arg1>  TThis;
  SKYAI_MODULE_NAMES(MReturnToSameModule10)

  MReturnToSameModule10 (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      // slot_reset     (*this),
      slot_in1       (*this),
      slot_in2       (*this),
      slot_in3       (*this),
      slot_in4       (*this),
      slot_in5       (*this),
      signal_in      (*this),
      slot_return    (*this),
      signal_return1 (*this),
      signal_return2 (*this),
      signal_return3 (*this),
      signal_return4 (*this),
      signal_return5 (*this)
    {
      // add_slot_port   (slot_reset     );
      add_slot_port   (slot_in1       );
      add_slot_port   (slot_in2       );
      add_slot_port   (slot_in3       );
      add_slot_port   (slot_in4       );
      add_slot_port   (slot_in5       );
      add_signal_port (signal_in      );
      add_slot_port   (slot_return    );
      add_signal_port (signal_return1 );
      add_signal_port (signal_return2 );
      add_signal_port (signal_return3 );
      add_signal_port (signal_return4 );
      add_signal_port (signal_return5 );
    }

protected:

  // MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  boost::function <void (void)>  signal_returnN_execall_;

  MAKE_SLOT_PORT(slot_in1, void, (const t_arg1 &a1), (a1), TThis);
  MAKE_SLOT_PORT(slot_in2, void, (const t_arg1 &a1), (a1), TThis);
  MAKE_SLOT_PORT(slot_in3, void, (const t_arg1 &a1), (a1), TThis);
  MAKE_SLOT_PORT(slot_in4, void, (const t_arg1 &a1), (a1), TThis);
  MAKE_SLOT_PORT(slot_in5, void, (const t_arg1 &a1), (a1), TThis);

  MAKE_SIGNAL_PORT(signal_in, void (const t_arg1 &), TThis);

  MAKE_SLOT_PORT(slot_return, void, (void), (), TThis);

  MAKE_SIGNAL_PORT(signal_return1, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_return2, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_return3, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_return4, void (void), TThis);
  MAKE_SIGNAL_PORT(signal_return5, void (void), TThis);

  // std::map<typename GET_PORT_TYPE(slot_in)::TConnectedPortIterator,
            // typename GET_PORT_TYPE(signal_return)::TConnectedPortIterator >  in_return_map_;

  // virtual void slot_reset_exec (void)
    // {
      // make in_return_map_
    // }
  void slot_in1_exec (const t_arg1 &a1)  {signal_returnN_execall_= boost::bind(&GET_PORT_TYPE(signal_return1)::ExecAll,&signal_return1); signal_in.ExecAll(a1);}
  void slot_in2_exec (const t_arg1 &a1)  {signal_returnN_execall_= boost::bind(&GET_PORT_TYPE(signal_return2)::ExecAll,&signal_return2); signal_in.ExecAll(a1);}
  void slot_in3_exec (const t_arg1 &a1)  {signal_returnN_execall_= boost::bind(&GET_PORT_TYPE(signal_return3)::ExecAll,&signal_return3); signal_in.ExecAll(a1);}
  void slot_in4_exec (const t_arg1 &a1)  {signal_returnN_execall_= boost::bind(&GET_PORT_TYPE(signal_return4)::ExecAll,&signal_return4); signal_in.ExecAll(a1);}
  void slot_in5_exec (const t_arg1 &a1)  {signal_returnN_execall_= boost::bind(&GET_PORT_TYPE(signal_return5)::ExecAll,&signal_return5); signal_in.ExecAll(a1);}

  void slot_return_exec (void)
    {
      if (signal_returnN_execall_.empty())
        {LERROR("every slot_in* has not been called"); lexit(df);}
      signal_returnN_execall_();
    }

};  // end of MReturnToSameModule10
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief reward accumulator
    \todo TODO: replace by MSimpleAccumulator\<TReal\> */
class MRewardAccumulator
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MRewardAccumulator  TThis;
  SKYAI_MODULE_NAMES(MRewardAccumulator)

  MRewardAccumulator (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      sum_              (0.0l),
      last_accessed_    (0.0l),
      slot_reset        (*this),
      slot_add          (*this),
      out_sum           (*this),
      out_last_accessed (*this)
    {
      add_slot_port (slot_reset        );
      add_slot_port (slot_add          );
      add_out_port  (out_sum           );
      add_out_port  (out_last_accessed );
    }

protected:

  TSingleReward  sum_;
  mutable TSingleReward  last_accessed_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_add, void, (const TSingleReward &r), (r), TThis);

  MAKE_OUT_PORT(out_sum, const TSingleReward&, (void), (), TThis);
  MAKE_OUT_PORT(out_last_accessed, const TSingleReward&, (void), (), TThis);


  virtual void slot_reset_exec (void)
    {
      sum_= 0.0l;
    }

  virtual void slot_add_exec (const TSingleReward &r)
    {
      sum_+= r;
    }

  virtual const TSingleReward& out_sum_get (void) const
    {
      last_accessed_= sum_;
      return sum_;
    }

  virtual const TSingleReward& out_last_accessed_get (void) const
    {
      return last_accessed_;
    }

};  // end of MRewardAccumulator
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MSimpleAccumulator
template <typename t_value>
class TSimpleAccumulatorConfigurations
//===========================================================================================
{
public:

  t_value       Zero;

  bool          UsingMaxConstraint;
  bool          UsingMinConstraint;
  t_value       Min;
  t_value       Max;

  TSimpleAccumulatorConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Zero                 );
      ADD( UsingMaxConstraint   );
      ADD( UsingMinConstraint   );
      ADD( Min                  );
      ADD( Max                  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief simple accumulator */
template <typename t_value>
class MSimpleAccumulator
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface    TParent;
  typedef MSimpleAccumulator  TThis;
  SKYAI_MODULE_NAMES(MSimpleAccumulator)

  MSimpleAccumulator (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      slot_reset        (*this),
      slot_add          (*this),
      signal_sum        (*this),
      out_sum           (*this)
    {
      add_slot_port  (slot_reset        );
      add_slot_port  (slot_add          );
      add_signal_port(signal_sum        );
      add_out_port   (out_sum           );
    }

protected:

  TSimpleAccumulatorConfigurations<t_value>  conf_;

  t_value  sum_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_add, void, (const t_value &x), (x), TThis);

  MAKE_SIGNAL_PORT(signal_sum, void (const t_value &x), TThis);

  MAKE_OUT_PORT(out_sum, const t_value&, (void), (), TThis);


  virtual void slot_reset_exec (void)
    {
      sum_= conf_.Zero;
    }

  virtual void slot_add_exec (const t_value &x)
    {
      using namespace utility_detail;
      sum_+= x;
      if (conf_.UsingMaxConstraint)  ApplyConstraintMax (sum_, conf_.Max);
      if (conf_.UsingMinConstraint)  ApplyConstraintMin (sum_, conf_.Min);
      signal_sum.ExecAll(sum_);
    }

  virtual const t_value& out_sum_get (void) const
    {
      return sum_;
    }

};  // end of MSimpleAccumulator
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
  typedef MEmitOnce           TThis;
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
/*!\brief signal holder module which holds a parameter of last signal */
template <typename t_arg1>
class MHolder
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef t_arg1               TArgument1;
  typedef MHolder<TArgument1>  TThis;
  SKYAI_MODULE_NAMES(MHolder)

  MHolder (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      slot_1            (*this),
      out_1             (*this)
    {
      add_slot_port (slot_1        );
      add_out_port  (out_1         );
    }

protected:

  TArgument1  last_signal1_;

  MAKE_SLOT_PORT(slot_1, void, (const TArgument1 &arg1), (arg1), TThis);

  MAKE_OUT_PORT(out_1, const TArgument1&, (void), (), TThis);

  virtual void slot_1_exec (const TArgument1 &arg1)
    {
      last_signal1_= arg1;
    }

  virtual const TArgument1& out_1_get (void) const
    {
      return last_signal1_;
    }

};  // end of MHolder
//-------------------------------------------------------------------------------------------


enum THolderKind
{
  hkExclusive=0   //! hold only the last signal; the others are cleared by conf_.Clear*
  // hkInclusive     //! treat every signal ports independently (so, this mode is useless. use MHolder)
};
ENUM_STR_MAP_BEGIN(THolderKind)
  ENUM_STR_MAP_ADD(hkExclusive    )
  // ENUM_STR_MAP_ADD(hkInclusive    )
ENUM_STR_MAP_END  (THolderKind)
SPECIALIZE_TVARIABLE_TO_ENUM(THolderKind)

//===========================================================================================
template <typename t_arg1, typename t_arg2>
class TCompositHolder2Configurations
//===========================================================================================
{
public:

  THolderKind               HolderKind;
  t_arg1                    Clear1;
  t_arg2                    Clear2;

  TCompositHolder2Configurations (var_space::TVariableMap &mmap)
    :
      HolderKind  (hkExclusive)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( HolderKind   );
      ADD( Clear1       );
      ADD( Clear2       );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief signal holder module which holds a parameter of last signal (multiple input) */
template <typename t_arg1, typename t_arg2>
class MCompositHolder2
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef t_arg1               TArgument1;
  typedef t_arg2               TArgument2;
  typedef MCompositHolder2<
                  TArgument1,
                  TArgument2>  TThis;
  SKYAI_MODULE_NAMES(MCompositHolder2)

  MCompositHolder2 (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      slot_1            (*this),
      slot_2            (*this),
      out_1             (*this),
      out_2             (*this)
    {
      add_slot_port (slot_1        );
      add_slot_port (slot_2        );
      add_out_port  (out_1         );
      add_out_port  (out_2         );
    }

protected:

  TCompositHolder2Configurations<TArgument1,TArgument2>  conf_;

  TArgument1  last_signal1_;
  TArgument2  last_signal2_;

  MAKE_SLOT_PORT(slot_1, void, (const TArgument1 &arg1), (arg1), TThis);
  MAKE_SLOT_PORT(slot_2, void, (const TArgument2 &arg2), (arg2), TThis);

  MAKE_OUT_PORT(out_1, const TArgument1&, (void), (), TThis);
  MAKE_OUT_PORT(out_2, const TArgument2&, (void), (), TThis);

  virtual void slot_1_exec (const TArgument1 &arg1)
    {
      last_signal1_= arg1;
      if (conf_.HolderKind == hkExclusive)  last_signal2_= conf_.Clear2;
    }
  virtual void slot_2_exec (const TArgument2 &arg2)
    {
      last_signal2_= arg2;
      if (conf_.HolderKind == hkExclusive)  last_signal1_= conf_.Clear1;
    }

  virtual const TArgument1& out_1_get (void) const
    {
      return last_signal1_;
    }
  virtual const TArgument2& out_2_get (void) const
    {
      return last_signal2_;
    }

};  // end of MCompositHolder2
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TConstMultiplierRvConfigurations
//===========================================================================================
{
public:

  TRealVector       Factor;

  TConstMultiplierRvConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Factor     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief multiplier which emits a signal with the parameter of an input signal multiplied by Factor */
class MConstMultiplierRv
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef MConstMultiplierRv   TThis;
  SKYAI_MODULE_NAMES(MConstMultiplierRv)

  MConstMultiplierRv (const std::string &v_instance_name)
    : TParent         (v_instance_name),
      conf_           (TParent::param_box_config_map()),
      slot_in         (*this),
      signal_out      (*this)
    {
      add_slot_port   (slot_in          );
      add_signal_port (signal_out       );
    }

  virtual ~MConstMultiplierRv() {}

protected:

  TConstMultiplierRvConfigurations  conf_;

  TRealVector      out_;

  MAKE_SLOT_PORT(slot_in, void, (const TRealVector &in), (in), TThis);

  MAKE_SIGNAL_PORT(signal_out, void (const TRealVector &), TThis);

  void slot_in_exec (const TRealVector &in)
    {
      out_.resize (GenSize(in));
      TypeExt<TRealVector>::const_iterator iitr(GenBegin(in)), ilastitr(GenEnd(in)), fitr(GenBegin(conf_.Factor));
      if (GenSize(conf_.Factor)==GenSize(in))
      {
        for (TypeExt<TRealVector>::iterator oitr(GenBegin(out_)); iitr!=ilastitr; ++iitr,++oitr,++fitr)
          (*oitr)= (*iitr) * (*fitr);
      }
      else if (GenSize(conf_.Factor)==1)
      {
        for (TypeExt<TRealVector>::iterator oitr(GenBegin(out_)); iitr!=ilastitr; ++iitr,++oitr)
          (*oitr)= (*iitr) * (*fitr);
      }
      else
        {LERROR("size disagreement!"); LDBGVAR(GenSize(conf_.Factor)); LDBGVAR(GenSize(in)); lexit(df);}

      signal_out.ExecAll (out_);
    }

};  // end of MConstMultiplierRv
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TDiscretizerConfigurations
//===========================================================================================
{
public:

  TRealVector               Min;  //! continuous vector which decides the lower bound of the grid
  TRealVector               Max;  //! continuous vector which decides the upper bound of the grid
  TIntVector                Division;  //! the numbers into which the grid is divided

  TDiscretizerConfigurations(var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Min         );
      ADD( Max         );
      ADD( Division    );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief discretizer which catches a signal with discrete parameter and emits a real (continuous) vector.
        the continuous vector space is discretized by a grid */
class MDiscretizer
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MDiscretizer       TThis;
  SKYAI_MODULE_NAMES(MDiscretizer)

  MDiscretizer (const std::string &v_instance_name)
    : TParent         (v_instance_name),
      conf_           (TParent::param_box_config_map()),
      slot_initialize (*this),
      slot_in         (*this),
      signal_out      (*this),
      out_set_size    (*this)
    {
      add_slot_port   (slot_initialize  );
      add_slot_port   (slot_in          );
      add_signal_port (signal_out       );
      add_out_port    (out_set_size     );
    }

  virtual ~MDiscretizer() {}

protected:

  TDiscretizerConfigurations  conf_;

  TGridGenerator   grid_;
  TRealVector      real_vector_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_in, void, (const TInt &n), (n), TThis);

  MAKE_SIGNAL_PORT(signal_out, void (const TRealVector &), TThis);

  MAKE_OUT_PORT(out_set_size, const TInt&, (void), (), TThis);

  void slot_initialize_exec (void)
    {
      if (GenSize(conf_.Min)!=GenSize(conf_.Max) || GenSize(conf_.Min)==0)
        {LERROR("size disagreement!");  LDBGVAR(GenSize(conf_.Min)); LDBGVAR(GenSize(conf_.Max)); lexit(df);}
      if (GenSize(conf_.Division)!=GenSize(conf_.Max) && GenSize(conf_.Division)!=1)
        {LERROR("GenSize(conf_.Division)(="<<GenSize(conf_.Division)<<") should be 1 or "<<GenSize(conf_.Max));  lexit(df);}
      if (GenSize(conf_.Division)==GenSize(conf_.Max))
      {
        grid_.Init (GenBegin(conf_.Division), GenEnd(conf_.Division),
                    GenBegin(conf_.Min),
                    GenBegin(conf_.Max));
      }
      else  // GenSize(conf_.Division)==1
      {
        TIntVector  divs (GenSize(conf_.Max), conf_.Division[1]);
        grid_.Init (GenBegin(divs), GenEnd(divs),
                    GenBegin(conf_.Min),
                    GenBegin(conf_.Max));
      }
      real_vector_.resize (GenSize(conf_.Max));
    }

  void slot_in_exec (const TInt &n)
    {
      grid_.IntToContVector (n, GenBegin(real_vector_));
      signal_out.ExecAll (real_vector_);
    }

  mutable TInt tmp_ssize_;
  const TInt& out_set_size_get (void) const
    {
      return (tmp_ssize_= grid_.Size());
    }

};  // end of MDiscretizer
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MIntToIntMapper
class TIntToIntMapperConfigurations
//===========================================================================================
{
public:

  typedef  std::map<TInt, TInt>  TIntToIntMap;
  TIntToIntMap    Map;

  TIntToIntMapperConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Map   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief mapper from an integer to an integer
    \todo this module has not been tested yet. */
class MIntToIntMapper
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface      TParent;
  typedef MIntToIntMapper       TThis;
  SKYAI_MODULE_NAMES(MIntToIntMapper)

  MIntToIntMapper (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      out_1          (*this),
      in_1           (*this)
    {
      add_out_port    (out_1);
      add_in_port     (in_1);
    }

protected:

  TIntToIntMapperConfigurations conf_;

  MAKE_OUT_PORT(out_1, const TInt&, (void), (), TThis);

  MAKE_IN_PORT(in_1, const TInt& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(1, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual const TInt& out_1_get (void) const
    {
      TIntToIntMapperConfigurations::TIntToIntMap::const_iterator  itr= conf_.Map.find(get_1());
      if (itr==conf_.Map.end())
        {LERROR(get_1()<<" is not found in the conf_.Map"); lexit(df);}
      return itr->second;
    }

};  // end of MIntToIntMapper
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MIntToBoolVectorMapper
class TIntToBoolVectorMapperConfigurations
//===========================================================================================
{
public:

  TInt                  Size;  //!< size of bool vector

  typedef  std::map<TInt, TString>  TIntToIntVectorMap;
  TIntToIntVectorMap    TrueSet;  //!< mapper of integer to set of true integers

  TIntToBoolVectorMapperConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Size        );
      ADD( TrueSet     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief mapper from an integer to a set of boolean vector */
class MIntToBoolVectorMapper
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface         TParent;
  typedef MIntToBoolVectorMapper   TThis;
  SKYAI_MODULE_NAMES(MIntToBoolVectorMapper)

  MIntToBoolVectorMapper (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      slot_reset     (*this),
      out_1          (*this),
      out_2          (*this),
      in_1           (*this)
    {
      add_slot_port   (slot_reset);
      add_out_port    (out_1);
      add_out_port    (out_2);
      add_in_port     (in_1);
    }

protected:

  TIntToBoolVectorMapperConfigurations conf_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_OUT_PORT(out_1, const TBoolVector&, (void), (), TThis);

  MAKE_OUT_PORT(out_2, const TBoolVector&, (const TInt &x), (x), TThis);

  MAKE_IN_PORT(in_1, const TInt& (void), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(1, const TInt&, (void), ())

  #undef GET_FROM_IN_PORT

  std::map<TInt, TBoolVector>  int_to_bvec_;
  virtual void slot_reset_exec (void)
    {
      TBoolVector  tmpb;
      TIntVector   tmpi;
      tmpb.resize(conf_.Size);
      int_to_bvec_.clear();
      for (TIntToBoolVectorMapperConfigurations::TIntToIntVectorMap::const_iterator itr= conf_.TrueSet.begin();
            itr!=conf_.TrueSet.end(); ++itr)
      {
        tmpi= ConvertFromStr<TIntVector >(itr->second);
        std::fill (GenBegin(tmpb),GenEnd(tmpb), false);
        for (TypeExt<TIntVector>::const_iterator tmpiitr(GenBegin(tmpi)); tmpiitr!=GenEnd(tmpi); ++tmpiitr)
          tmpb[*tmpiitr]= true;
        int_to_bvec_[itr->first]= tmpb;
      }
    }

  virtual const TBoolVector& out_1_get (void) const
    {
      return out_2_get(get_1());
    }
  virtual const TBoolVector& out_2_get (const TInt &x) const
    {
      std::map<TInt, TBoolVector>::const_iterator  itr= int_to_bvec_.find(x);
      if (itr==int_to_bvec_.end())
        {LERROR(x<<" is not found in the conf_.TrueSet (or slot_reset has not been executed)"); lexit(df);}
      return itr->second;
    }

};  // end of MIntToBoolVectorMapper
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMinMaxElement,TIntVector)
// SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMinMaxElement,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MNonzeroElements,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MNonzeroElements,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MRemoveSignalArguments,TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSignalDistributor1,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSignalDistributor1,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSignalDistributor1,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSignalDistributor1,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MFunctionSelector0,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MFunctionSelector0,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MFunctionSelector0,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MFunctionSelector0,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSelector1,TVoid,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MReturnToSameModule10,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MReturnToSameModule10,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MReturnToSameModule10,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MReturnToSameModule10,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleAccumulator,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleAccumulator,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleAccumulator,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleAccumulator,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MEmitOnce,TReal)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHolder,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHolder,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHolder,TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCompositHolder2,TInt,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCompositHolder2,TRealVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCompositHolder2,TInt,TInt)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_utility_h
//-------------------------------------------------------------------------------------------
