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
#ifndef skyai_core_utility_h
#define skyai_core_utility_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <lora/stl_math.h>   // for operator+ for TIntVector
#include <lora/small_classes.h>   // for TGridGenerator, TStatisticsFilter
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
// template utilities
//===========================================================================================

//-------------------------------------------------------------------------------------------
namespace skyai_utility_detail
{
//-------------------------------------------------------------------------------------------

//! make a map [index --> one of connected ports] from lowerport_to_indexes. index should be in {0,..,max_index-1}
template<typename t_port_for_lower>
void MakeLowerPortsIndexConnectionIteratorMap (
    const TCompositeModule &parent_c_module,
    t_port_for_lower  &port_for_lower,
    std::vector<typename t_port_for_lower::TConnectedPortIterator >  &lower_modules_citr,
    TInt max_index, const std::map<TString, TIntVector>  &lowerport_to_indexes)
{
  int lower_size (max_index);
  lower_modules_citr.clear();
  lower_modules_citr.resize (lower_size);

  std::map<TString, TIntVector>::const_iterator  order_itr;
  for (typename t_port_for_lower::TConnectedPortIterator citr(port_for_lower.ConnectedPortBegin()); citr!=port_for_lower.ConnectedPortEnd(); ++citr)
  {
    order_itr= lowerport_to_indexes.find (parent_c_module.SearchSubPortUniqueCode(*citr));
    if (order_itr != lowerport_to_indexes.end())
    {
      for (std::vector<TInt>::const_iterator itr(order_itr->second.begin()),last(order_itr->second.end()); itr!=last; ++itr)
        lower_modules_citr[*itr]= citr;
    }
    else
      {LERROR("module.port: "<<parent_c_module.SearchSubPortUniqueCode(*citr)<<" should be listed in IndexesOfLowers"); lexit(df);}
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
  TypeExtS<TIntVector>::const_iterator mitr(GenBegin(max));
  for (TypeExtS<TIntVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr>*mitr)  *itr= *mitr;
}
template <> inline void ApplyConstraintMax (TRealVector &value, const TRealVector &max)
{
  LASSERT1op1(GenSize(value),==,GenSize(max));
  TypeExtS<TRealVector>::const_iterator mitr(GenBegin(max));
  for (TypeExtS<TRealVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
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
  TypeExtS<TIntVector>::const_iterator mitr(GenBegin(min));
  for (TypeExtS<TIntVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr<*mitr)  *itr= *mitr;
}
template <> inline void ApplyConstraintMin (TRealVector &value, const TRealVector &min)
{
  LASSERT1op1(GenSize(value),==,GenSize(min));
  TypeExtS<TRealVector>::const_iterator mitr(GenBegin(min));
  for (TypeExtS<TRealVector>::iterator itr(GenBegin(value)); itr!=GenEnd(value); ++itr,++mitr)
    if (*itr<*mitr)  *itr= *mitr;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of skyai_utility_detail
//-------------------------------------------------------------------------------------------




//===========================================================================================
/*!\brief print to stdout (for debug) */
class MPrinter
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MPrinter           TThis;
  SKYAI_MODULE_NAMES(MPrinter)

  MPrinter (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_int            (*this),
      in_real           (*this),
      in_int_vector     (*this),
      in_real_vector    (*this),
      in_bool           (*this),
      in_string         (*this),

      slot_void         (*this),
      slot_int          (*this),
      slot_real         (*this),
      slot_int_vector   (*this),
      slot_real_vector  (*this),
      slot_bool         (*this),
      slot_string       (*this),

      slot_print_in     (*this)
    {
      add_in_port   (in_int            );
      add_in_port   (in_real           );
      add_in_port   (in_int_vector     );
      add_in_port   (in_real_vector    );
      add_in_port   (in_bool           );
      add_in_port   (in_string         );

      add_slot_port (slot_void         );
      add_slot_port (slot_int          );
      add_slot_port (slot_real         );
      add_slot_port (slot_int_vector   );
      add_slot_port (slot_real_vector  );
      add_slot_port (slot_bool         );
      add_slot_port (slot_string       );

      add_slot_port (slot_print_in     );
    }

protected:

  MAKE_IN_PORT(in_int         , const TInt        & (void), TThis);
  MAKE_IN_PORT(in_real        , const TReal       & (void), TThis);
  MAKE_IN_PORT(in_int_vector  , const TIntVector  & (void), TThis);
  MAKE_IN_PORT(in_real_vector , const TRealVector & (void), TThis);
  MAKE_IN_PORT(in_bool        , const TBool       & (void), TThis);
  MAKE_IN_PORT(in_string      , const TString     & (void), TThis);

  MAKE_SLOT_PORT(slot_void, void, (void), (), TThis);
  MAKE_SLOT_PORT(slot_int         , void, (const TInt        &in), (in), TThis);
  MAKE_SLOT_PORT(slot_real        , void, (const TReal       &in), (in), TThis);
  MAKE_SLOT_PORT(slot_int_vector  , void, (const TIntVector  &in), (in), TThis);
  MAKE_SLOT_PORT(slot_real_vector , void, (const TRealVector &in), (in), TThis);
  MAKE_SLOT_PORT(slot_bool        , void, (const TBool       &in), (in), TThis);
  MAKE_SLOT_PORT(slot_string      , void, (const TString     &in), (in), TThis);

  MAKE_SLOT_PORT(slot_print_in, void, (void), (), TThis);

  void slot_void_exec (void)  {std::cout<<InstanceName()<<":"<<std::endl;}
  #define DEF_SLOT(x_port,x_type) \
    void slot_##x_port##_exec (const x_type &in)  {std::cout<<InstanceName()<<": "<<ConvertToStr(in)<<std::endl;}
  DEF_SLOT(int         , TInt        )
  DEF_SLOT(real        , TReal       )
  DEF_SLOT(int_vector  , TIntVector  )
  DEF_SLOT(real_vector , TRealVector )
  DEF_SLOT(bool        , TBool       )
  DEF_SLOT(string      , TString     )
  #undef DEF_SLOT

  void slot_print_in_exec (void)
    {
      #define PRINT_IN(x_port)  \
        if (in_##x_port.ConnectionSize()!=0)  \
          {std::cout<<InstanceName()<<":"#x_port" "<<ConvertToStr(in_##x_port.GetFirst())<<std::endl;}
      PRINT_IN(int         )
      PRINT_IN(real        )
      PRINT_IN(int_vector  )
      PRINT_IN(real_vector )
      PRINT_IN(bool        )
      PRINT_IN(string      )
      #undef PRINT_IN
    }

};  // end of MPrinter
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief just output the input (num of args is 0) */
template <typename t_ret>
class MMediator0
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MMediator0<t_ret>  TThis;
  SKYAI_MODULE_NAMES(MMediator0)

  MMediator0 (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_1              (*this),
      out_1             (*this)
    {
      add_in_port   (in_1  );
      add_out_port  (out_1 );
    }

protected:

  MAKE_IN_PORT(in_1, const t_ret& (void), TThis);
  MAKE_OUT_PORT(out_1, const t_ret&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }
  GET_FROM_IN_PORT(1, const t_ret&, (void), ())
  #undef GET_FROM_IN_PORT

  virtual const t_ret& out_1_get (void) const
    {
      return get_1();
    }

};  // end of MMediator0
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief just output the input (num of args is 1) */
template <typename t_ret, typename t_arg1>
class MMediator1
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface          TParent;
  typedef MMediator1<t_ret,t_arg1>  TThis;
  SKYAI_MODULE_NAMES(MMediator1)

  MMediator1 (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_1              (*this),
      out_1             (*this)
    {
      add_in_port   (in_1  );
      add_out_port  (out_1 );
    }

protected:

  MAKE_IN_PORT(in_1, const t_ret& (const t_arg1 &a1), TThis);
  MAKE_OUT_PORT(out_1, const t_ret&, (const t_arg1 &a1), (a1), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }
  GET_FROM_IN_PORT(1, const t_ret&, (const t_arg1 &a1), (a1))
  #undef GET_FROM_IN_PORT

  virtual const t_ret& out_1_get (const t_arg1 &a1) const
    {
      return get_1(a1);
    }

};  // end of MMediator1
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief just output the input (num of args is 1; specialized for void) */
template <typename t_arg1>
class MMediator1<void,t_arg1>
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface          TParent;
  typedef MMediator1<void,t_arg1>  TThis;
  SKYAI_MODULE_NAMES(MMediator1)

  MMediator1 (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_1              (*this),
      out_1             (*this)
    {
      add_in_port   (in_1  );
      add_out_port  (out_1 );
    }

protected:

  MAKE_IN_PORT(in_1, void (const t_arg1 &a1), TThis);
  MAKE_OUT_PORT(out_1, void, (const t_arg1 &a1), (a1), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }
  GET_FROM_IN_PORT(1, void, (const t_arg1 &a1), (a1))
  #undef GET_FROM_IN_PORT

  virtual void out_1_get (const t_arg1 &a1) const
    {
      return get_1(a1);
    }

};  // end of MMediator1
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MVectorMixer
class TVectorMixerConfigurations
//===========================================================================================
{
public:

  struct TElement
    {
      TString  PortCode ;  //!< unique code of port connected to in_vectors
      TInt     Element  ;  //!< element index of the input vector of PortCode
      TElement()
        :
          PortCode  (""),
          Element   (-1)
        {}
    };

  std::vector<TElement>  MixingTable;  //!< mixing configuration of each element of the output vector (out_mixed)

  TVectorMixerConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( MixingTable   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
namespace var_space{
  void Register (TVectorMixerConfigurations::TElement &x, TVariableMap &mmap)
  {
    #define ADD(x_member)  AddToVarMap(mmap, #x_member, x.x_member)
    ADD( PortCode  );
    ADD( Element   );
    #undef ADD
  }
}
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief create a vector mixing some input vectors */
template <typename t_vector>
class MVectorMixer
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MVectorMixer       TThis;
  SKYAI_MODULE_NAMES(MVectorMixer)

  MVectorMixer (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      slot_initialize   (*this),
      in_vectors        (*this),
      out_mixed         (*this)
    {
      add_slot_port (slot_initialize  );
      add_in_port   (in_vectors  );
      add_out_port  (out_mixed        );
    }

protected:

  TVectorMixerConfigurations  conf_;

  //!\brief construct port_iterator_map_
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  //!\brief input vectors
  MAKE_IN_PORT_SPECIFIC(in_vectors, const t_vector& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  MAKE_OUT_PORT(out_mixed, const t_vector&, (void), (), TThis);

  struct TElement
    {
      typename GET_PORT_TYPE(in_vectors)::TConnectedPortIterator  CItr;
      TInt     Element  ;
      TElement() : Element(-1)  {}
    };

  std::vector<TElement>  mixing_table_;  //! hold the mixing configuration of each element of the output vector (out_mixed)

  mutable t_vector output_;

  void slot_initialize_exec (void)
    {
      mixing_table_.clear();
      mixing_table_.resize(conf_.MixingTable.size());
      GenResize(output_, conf_.MixingTable.size());

      typename std::vector<TElement>::iterator  mixt_itr(mixing_table_.begin());
      for (std::vector<TVectorMixerConfigurations::TElement>::const_iterator
            conf_itr(conf_.MixingTable.begin()),conf_last(conf_.MixingTable.end());
              conf_itr!=conf_last; ++conf_itr,++mixt_itr)
      {
        typename GET_PORT_TYPE(in_vectors)::TConnectedPortIterator
                  iv_citr= in_vectors.ConnectedPortFind (ParentCModule().SearchSubPort(conf_itr->PortCode));
        if (iv_citr==in_vectors.ConnectedPortEnd())
        {
          LERROR("Port "<<conf_itr->PortCode<<" is not connected to "<<ParentCModule().SearchSubPortUniqueCode(&in_vectors));
          lexit(df);
        }
        mixt_itr->CItr= iv_citr;
        mixt_itr->Element= conf_itr->Element;
      }
    }

  const t_vector& out_mixed_get (void) const
    {
      typename TypeExtS<t_vector>::iterator out_itr(GenBegin(output_));
      for (typename std::vector<TElement>::const_iterator mixt_itr(mixing_table_.begin()),mixt_last(mixing_table_.end());
            mixt_itr!=mixt_last; ++mixt_itr,++out_itr)
      {
        (*out_itr)= GenAt(in_vectors.GetCurrent(mixt_itr->CItr),mixt_itr->Element);
      }
      return output_;
    }

};  // end of MVectorMixer
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief Configurations of MWeightedMixerRv
class TWeightedMixerRvConfigurations
//===========================================================================================
{
public:

  struct TElement
    {
      TString  PortCode ;  //!< unique code of port connected to in_real_vectors
      TInt     Element  ;  //!< element index of the input vector of PortCode
      TReal    Weight   ;  //!< output weight
      TElement()
        :
          PortCode  (""),
          Element   (-1),
          Weight    (1.0l)
        {}
    };

  std::vector<TElement>  MixingTable;  //!< mixing configuration of each element of the output vector (out_mixed)

  TWeightedMixerRvConfigurations (var_space::TVariableMap &mmap)
    // :
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( MixingTable   );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
namespace var_space{
  void Register (TWeightedMixerRvConfigurations::TElement &x, TVariableMap &mmap)
  {
    #define ADD(x_member)  AddToVarMap(mmap, #x_member, x.x_member)
    ADD( PortCode  );
    ADD( Element   );
    ADD( Weight    );
    #undef ADD
  }
}
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief create a vector mixing some input vectors */
class MWeightedMixerRv
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MWeightedMixerRv   TThis;
  SKYAI_MODULE_NAMES(MWeightedMixerRv)

  MWeightedMixerRv (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      slot_initialize   (*this),
      in_real_vectors   (*this),
      out_mixed         (*this)
    {
      add_slot_port (slot_initialize  );
      add_in_port   (in_real_vectors  );
      add_out_port  (out_mixed        );
    }

protected:

  TWeightedMixerRvConfigurations  conf_;

  //!\brief construct port_iterator_map_
  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  //!\brief input vectors (real)
  MAKE_IN_PORT_SPECIFIC(in_real_vectors, const TRealVector& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  // TODO: add following port:
  // MAKE_IN_PORT_SPECIFIC(in_int_vectors, const TIntVector& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  MAKE_OUT_PORT(out_mixed, const TRealVector&, (void), (), TThis);


  struct TElement
    {
      GET_PORT_TYPE(in_real_vectors)::TConnectedPortIterator  CItr;
      TInt     Element  ;
      TReal    Weight   ;
      TElement() : Element(-1), Weight(1.0l) {}
    };

  std::vector<TElement>  mixing_table_;  //! hold the mixing configuration of each element of the output vector (out_mixed)

  mutable TRealVector output_;


  void slot_initialize_exec (void)
    {
      mixing_table_.clear();
      mixing_table_.resize(conf_.MixingTable.size());
      GenResize(output_, conf_.MixingTable.size());

      std::vector<TElement>::iterator  mixt_itr(mixing_table_.begin());
      for (std::vector<TWeightedMixerRvConfigurations::TElement>::const_iterator
            conf_itr(conf_.MixingTable.begin()),conf_last(conf_.MixingTable.end());
              conf_itr!=conf_last; ++conf_itr,++mixt_itr)
      {
        GET_PORT_TYPE(in_real_vectors)::TConnectedPortIterator
                  iv_citr= in_real_vectors.ConnectedPortFind (ParentCModule().SearchSubPort(conf_itr->PortCode));
        if (iv_citr==in_real_vectors.ConnectedPortEnd())
        {
          LERROR("Port "<<conf_itr->PortCode<<" is not connected to "<<ParentCModule().SearchSubPortUniqueCode(&in_real_vectors));
          lexit(df);
        }
        mixt_itr->CItr= iv_citr;
        mixt_itr->Element= conf_itr->Element;
        mixt_itr->Weight= conf_itr->Weight;
      }
    }

  const TRealVector& out_mixed_get (void) const
    {
      TypeExtS<TRealVector>::iterator out_itr(GenBegin(output_));
      for (std::vector<TElement>::const_iterator mixt_itr(mixing_table_.begin()),mixt_last(mixing_table_.end());
            mixt_itr!=mixt_last; ++mixt_itr,++out_itr)
      {
        (*out_itr)= mixt_itr->Weight * GenAt(in_real_vectors.GetCurrent(mixt_itr->CItr),mixt_itr->Element);
      }
      return output_;
    }

};  // end of MWeightedMixerRv
//-------------------------------------------------------------------------------------------



//===========================================================================================
//!\brief Configurations of MSignalDistributor*
class TSignalDistributorConfigurations
//===========================================================================================
{
public:

  // for organization
  TInt                  SizeOfLowers;

  typedef  std::map<TString, TIntVector>  TLowerportToIndexesMap;
  TLowerportToIndexesMap    IndexesOfLowers;  //!< map of (PortUniqueCode of lower port -> set of index). index should be in {0,..,SizeOfLowers-1}

  TSignalDistributorConfigurations (var_space::TVariableMap &mmap)
    : SizeOfLowers(-1)
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
      using namespace skyai_utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (ParentCModule(), signal_out, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
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
      using namespace skyai_utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (ParentCModule(), signal_out, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
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

  typedef  std::map<TString, TIntVector>  TLowerportToIndexesMap;
  TLowerportToIndexesMap    IndexesOfLowers;  //!< map of (PortUniqueCode of lower port -> set of index). index should be in {0,..,SizeOfLowers-1}

  TFunctionSelectorConfigurations (var_space::TVariableMap &mmap)
    : SizeOfLowers(-1)
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
      using namespace skyai_utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (ParentCModule(), in_function, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
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
      using namespace skyai_utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (ParentCModule(), in_function, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
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
      using namespace skyai_utility_detail;
      MakeLowerPortsIndexConnectionIteratorMap (ParentCModule(), in_function, lower_modules_citr_, conf_.SizeOfLowers, conf_.IndexesOfLowers);
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
//!\brief Configurations of MSimpleAccumulator
template <typename t_value>
class TSimpleAccumulatorConfigurations
//===========================================================================================
{
public:

  t_value       Zero;

  bool          UsingMaxConstraint;
  bool          UsingMinConstraint;
  t_value       Max;
  t_value       Min;

  TSimpleAccumulatorConfigurations (var_space::TVariableMap &mmap)
    :
      UsingMaxConstraint (false),
      UsingMinConstraint (false)
    {
      SetZero(Zero);
      SetZero(Max);
      SetZero(Min);
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( Zero                 );
      ADD( UsingMaxConstraint   );
      ADD( UsingMinConstraint   );
      ADD( Max                  );
      ADD( Min                  );
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
      out_sum           (*this),
      out_last_accessed (*this)
    {
      add_slot_port  (slot_reset        );
      add_slot_port  (slot_add          );
      add_signal_port(signal_sum        );
      add_out_port   (out_sum           );
      add_out_port   (out_last_accessed );
    }

protected:

  TSimpleAccumulatorConfigurations<t_value>  conf_;

  t_value  sum_;
  mutable t_value  last_accessed_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_add, void, (const t_value &x), (x), TThis);

  MAKE_SIGNAL_PORT(signal_sum, void (const t_value &x), TThis);

  MAKE_OUT_PORT(out_sum, const t_value&, (void), (), TThis);
  MAKE_OUT_PORT(out_last_accessed, const t_value&, (void), (), TThis);


  virtual void slot_reset_exec (void)
    {
      sum_= conf_.Zero;
      // do not do here: last_accessed_= conf_.Zero;
    }

  virtual void slot_add_exec (const t_value &x)
    {
      using namespace skyai_utility_detail;
      sum_+= x;
      if (conf_.UsingMaxConstraint)  ApplyConstraintMax (sum_, conf_.Max);
      if (conf_.UsingMinConstraint)  ApplyConstraintMin (sum_, conf_.Min);
      signal_sum.ExecAll(sum_);
    }

  virtual const t_value& out_sum_get (void) const
    {
      last_accessed_= sum_;
      return sum_;
    }

  virtual const t_value& out_last_accessed_get (void) const
    {
      return last_accessed_;
    }

};  // end of MSimpleAccumulator
//-------------------------------------------------------------------------------------------


ENUM_STR_MAP_BEGIN_NS(TStatisticsFilter<TReal>, TFilterKind)
  ENUM_STR_MAP_ADD_NS(TStatisticsFilter<TReal>, fkNormal   )
  ENUM_STR_MAP_ADD_NS(TStatisticsFilter<TReal>, fkDynamic  )
ENUM_STR_MAP_END_NS  (TStatisticsFilter<TReal>, TFilterKind)
SPECIALIZE_TVARIABLE_TO_ENUM(TStatisticsFilter<TReal>::TFilterKind)

//===========================================================================================
//!\brief Configurations of MStatisticsFilter
class TStatisticsFilterConfigurations
//===========================================================================================
{
public:

  TStatisticsFilter<TReal>::TFilterKind  FilterKind;
  TReal                                  LearningRate;  //!< used with FilterKind==fkDynamic

  TStatisticsFilterConfigurations (var_space::TVariableMap &mmap)
    :
      FilterKind    (TStatisticsFilter<TReal>::fkNormal),
      LearningRate  (0.01l)
    {
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( FilterKind    );
      ADD( LearningRate  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------
//===========================================================================================
/*!\brief statistics filter that calculates sum, mean, variance, std-deviation, max, min. */
class MStatisticsFilter
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MStatisticsFilter  TThis;
  SKYAI_MODULE_NAMES(MStatisticsFilter)

  MStatisticsFilter (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      conf_             (TParent::param_box_config_map()),
      tmp_variance_     (0.0l),
      tmp_stddev_       (0.0l),
      slot_reset        (*this),
      slot_step1        (*this),
      slot_step2        (*this),
      in_value          (*this),
      out_sum           (*this),
      out_max           (*this),
      out_min           (*this),
      out_mean          (*this),
      out_variance      (*this),
      out_std_deviation (*this),
      out_num_samples   (*this)
    {
      add_slot_port (slot_reset        );
      add_slot_port (slot_step1        );
      add_slot_port (slot_step2        );
      add_in_port   (in_value          );
      add_out_port  (out_sum           );
      add_out_port  (out_max           );
      add_out_port  (out_min           );
      add_out_port  (out_mean          );
      add_out_port  (out_variance      );
      add_out_port  (out_std_deviation );
      add_out_port  (out_num_samples   );
    }

protected:

  TStatisticsFilterConfigurations  conf_;
  TStatisticsFilter<TReal>  sfilter_;

  mutable TReal tmp_variance_;
  mutable TReal tmp_stddev_;

  MAKE_SLOT_PORT(slot_reset, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_step1, void, (const TReal &x), (x), TThis);
  //! step using in_value
  MAKE_SLOT_PORT(slot_step2, void, (void), (), TThis);

  MAKE_IN_PORT(in_value, const TReal& (void), TThis);

  MAKE_OUT_PORT(out_sum, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_max, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_min, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_mean, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_variance, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_std_deviation, const TReal&, (void), (), TThis);
  MAKE_OUT_PORT(out_num_samples, const TInt&, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(value, const TReal&, (void), ())

  #undef GET_FROM_IN_PORT

  void slot_reset_exec (void)  {sfilter_.Reset();}
  void slot_step1_exec (const TReal &x)  {sfilter_.Step(x);}
  void slot_step2_exec (void)  {sfilter_.Step(get_value());}

  const TReal& out_sum_get  (void) const {return sfilter_.Sum();}
  const TReal& out_max_get  (void) const {return sfilter_.Max();}
  const TReal& out_min_get  (void) const {return sfilter_.Min();}
  const TReal& out_mean_get (void) const {return sfilter_.Mean();}
  const TReal& out_variance_get (void) const {tmp_variance_= sfilter_.Variance(); return tmp_variance_;}
  const TReal& out_std_deviation_get (void) const {tmp_stddev_= sfilter_.StdDeviation(); return tmp_stddev_;}
  const TInt& out_num_samples_get (void) const {return sfilter_.NumSamples();}

};  // end of MStatisticsFilter
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief function sharer which shares a SISO-function in some signal-slot pairs */
template <typename t_input, typename t_output>
class MFunctionSISOSharer
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface                       TParent;
  typedef t_input                                TInput;
  typedef t_output                               TOutput;
  typedef MFunctionSISOSharer<t_input,t_output>  TThis;
  SKYAI_MODULE_NAMES(MFunctionSISOSharer)

  MFunctionSISOSharer (const std::string &v_instance_name)
    : TParent           (v_instance_name),
      in_function       (*this),
      slot_x1           (*this),
      slot_x2           (*this),
      slot_x3           (*this),
      slot_x4           (*this),
      slot_x5           (*this),
      signal_y1         (*this),
      signal_y2         (*this),
      signal_y3         (*this),
      signal_y4         (*this),
      signal_y5         (*this)
    {
      add_in_port (in_function);
      add_slot_port (slot_x1);
      add_slot_port (slot_x2);
      add_slot_port (slot_x3);
      add_slot_port (slot_x4);
      add_slot_port (slot_x5);
      add_signal_port (signal_y1);
      add_signal_port (signal_y2);
      add_signal_port (signal_y3);
      add_signal_port (signal_y4);
      add_signal_port (signal_y5);
    }

protected:

  TOutput  tmp_y1_;
  TOutput  tmp_y2_;
  TOutput  tmp_y3_;
  TOutput  tmp_y4_;
  TOutput  tmp_y5_;

  MAKE_IN_PORT(in_function, void (const TInput &x, TOutput &y), TThis);

  MAKE_SLOT_PORT(slot_x1, void, (const TInput &x), (x), TThis);
  MAKE_SLOT_PORT(slot_x2, void, (const TInput &x), (x), TThis);
  MAKE_SLOT_PORT(slot_x3, void, (const TInput &x), (x), TThis);
  MAKE_SLOT_PORT(slot_x4, void, (const TInput &x), (x), TThis);
  MAKE_SLOT_PORT(slot_x5, void, (const TInput &x), (x), TThis);
  MAKE_SIGNAL_PORT(signal_y1, void (const TOutput&), TThis);
  MAKE_SIGNAL_PORT(signal_y2, void (const TOutput&), TThis);
  MAKE_SIGNAL_PORT(signal_y3, void (const TOutput&), TThis);
  MAKE_SIGNAL_PORT(signal_y4, void (const TOutput&), TThis);
  MAKE_SIGNAL_PORT(signal_y5, void (const TOutput&), TThis);

  // generate get_##x_in (e.g. get_reward)
  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(function, void, (const TInput &x, TOutput &y), (x,y))

  #undef GET_FROM_IN_PORT

  #define GEN_SLOT_N(x_N)  \
    void slot_x##x_N##_exec (const TInput &x)     \
      {                                           \
        get_function (x,tmp_y##x_N##_);           \
        signal_y##x_N.ExecAll (tmp_y##x_N##_);  \
      }
  GEN_SLOT_N(1)
  GEN_SLOT_N(2)
  GEN_SLOT_N(3)
  GEN_SLOT_N(4)
  GEN_SLOT_N(5)
  #undef GEN_SLOT_N

};  // end of MFunctionSISOSharer
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
class TCompositeHolder2Configurations
//===========================================================================================
{
public:

  THolderKind               HolderKind;
  t_arg1                    Clear1;
  t_arg2                    Clear2;

  TCompositeHolder2Configurations (var_space::TVariableMap &mmap)
    :
      HolderKind  (hkExclusive)
    {
      SetZero(Clear1);
      SetZero(Clear2);
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
class MCompositeHolder2
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface     TParent;
  typedef t_arg1               TArgument1;
  typedef t_arg2               TArgument2;
  typedef MCompositeHolder2<
                  TArgument1,
                  TArgument2>  TThis;
  SKYAI_MODULE_NAMES(MCompositeHolder2)

  MCompositeHolder2 (const std::string &v_instance_name)
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

  TCompositeHolder2Configurations<TArgument1,TArgument2>  conf_;

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

};  // end of MCompositeHolder2
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

  typedef  std::map<TInt, TIntVector>  TIntToIntVectorMap;
  TIntToIntVectorMap    TrueSet;  //!< mapper of integer to set of true integers

  TIntToBoolVectorMapperConfigurations (var_space::TVariableMap &mmap)
    : Size(-1)
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
      tmpb.resize(conf_.Size);
      int_to_bvec_.clear();
      for (TIntToBoolVectorMapperConfigurations::TIntToIntVectorMap::const_iterator itr= conf_.TrueSet.begin();
            itr!=conf_.TrueSet.end(); ++itr)
      {
        std::fill (GenBegin(tmpb),GenEnd(tmpb), false);
        for (TypeExtS<TIntVector>::const_iterator iv_itr(GenBegin(itr->second)),iv_last(GenEnd(itr->second)); iv_itr!=iv_last; ++iv_itr)
          tmpb[*iv_itr]= true;
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
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TBoolVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TRealMatrix)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MMediator0,TRealVectorSet)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TBool)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TBoolVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MMediator1,TVoid,TRealMatrix)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorMixer,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorMixer,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MVectorMixer,TBoolVector)
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
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TInt,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TReal,TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TIntVector,TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MFunctionSISOSharer,TRealVector,TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHolder,TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHolder,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MHolder,TComposite1)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCompositeHolder2,TInt,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCompositeHolder2,TRealVector,TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MCompositeHolder2,TInt,TInt)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_core_utility_h
//-------------------------------------------------------------------------------------------
