//-------------------------------------------------------------------------------------------
/*! \file    bf_ngnet.h
    \brief   libskyai - NGnet basis functions
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.21, 2009-

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
#ifndef skyai_bf_ngnet_h
#define skyai_bf_ngnet_h
//-------------------------------------------------------------------------------------------
#include <skyai/interfaces/functions.h>
#include <skyai/types.h>
#include <lora/oldngnet.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace basis_functions_ngnet_detail
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TConfigurations
//===========================================================================================
{
public:

  std::string            NGnetFileName;

  TConfigurations(var_space::TVariableMap &mmap)
    :
      NGnetFileName      ("")
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( NGnetFileName     );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//!\brief NGnet basis functions module
class MBasisFunctionsNGnet
    : public MBasisFunctionsInterface <TContinuousState, TRealVector>
//===========================================================================================
{
public:
  typedef MBasisFunctionsInterface <
                  TContinuousState,
                  TRealVector>           TParent;
  typedef MBasisFunctionsNGnet           TThis;
  SKYAI_MODULE_NAMES(MBasisFunctionsNGnet)

  MBasisFunctionsNGnet (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_          (TParent::param_box_config_map()),
      ngnetcnf_      (-1/*xt*/,-1/*y*/,-1/*x*/,-1/*u*/,wcNone),
      ngnet_         (ngnetcnf_)
    {
    }

  virtual ~MBasisFunctionsNGnet() {}

  TConfigurations&       Config()       {return conf_;}
  const TConfigurations& Config() const {return conf_;}

  TNGnet&       NGnet()       {return ngnet_;}
  const TNGnet& NGnet() const {return ngnet_;}

  TNGnetConfiguration&       NGnetConfig()       {return ngnetcnf_;}
  const TNGnetConfiguration& NGnetConfig() const {return ngnetcnf_;}

protected:

  TConfigurations      conf_;

  TNGnetConfiguration  ngnetcnf_;
  TNGnet               ngnet_;

  override void slot_initialize_exec (void);

  override void function (const TInput &x, TFeature &y) const
    {
      if (ngnet_.size()==0)
        {LERROR("NGnet is not loaded"); lexit(df);}
      y= ngnet_.Pi(x);
    }

  mutable TInt tmp_fdim_;
  override const TInt& out_feature_dim_get (void) const  {return (tmp_fdim_= ngnet_.size());}

};  // end of MBasisFunctionsNGnet
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TDCOBNGnetConfigurations
//===========================================================================================
{
public:

  /*! if sqrt(eigenvalue) of the covariance matrix is larger than this value,
      the gaussian is assumed to diverge w.r.t. the axis.
      this value is used in get_eigvmax which is used in calc_distance_to_nearest_bf and calc_ext_distance_to_nearest_bf */
  TReal              StdDeviationInfinityThreshold;

  /*! using maximum norm as distance_to_nearest_bf_ instead of L-2 norm */
  bool               UsingMaxNorm;

  TDCOBNGnetConfigurations(var_space::TVariableMap &mmap)
    :
      StdDeviationInfinityThreshold    (500.0l),
      UsingMaxNorm                     (false)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( StdDeviationInfinityThreshold       );
      ADD( UsingMaxNorm                        );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief NGnet basis functions module
class MBasisFunctionsDCOBNGnet
    : public MBasisFunctionsNGnet
//===========================================================================================
{
public:
  typedef MBasisFunctionsNGnet      TParent;
  typedef MBasisFunctionsDCOBNGnet  TThis;
  SKYAI_MODULE_NAMES(MBasisFunctionsDCOBNGnet)

  MBasisFunctionsDCOBNGnet (const std::string &v_instance_name)
    : TParent        (v_instance_name),
      conf_dcobngnet_(TParent::param_box_config_map()),
      out_distance_to_nearest_bf   (*this),
      out_center_state_set         (*this),
      in_extract_proportional      (*this)
    {
      add_out_port (out_distance_to_nearest_bf);
      add_out_port (out_center_state_set);
      add_in_port  (in_extract_proportional);
    }

  virtual ~MBasisFunctionsDCOBNGnet() {}

protected:

  TDCOBNGnetConfigurations  conf_dcobngnet_;

  TRealVector distance_to_nearest_bf_;
  TRealVectorSet   center_state_set_;

  //!\brief distance vector to the nearest basis function from each BF (dim == num. of BFs)
  MAKE_OUT_PORT(out_distance_to_nearest_bf, const TRealVector&, (void), (), TThis);

  MAKE_OUT_PORT(out_center_state_set, const TRealVectorSet&, (void), (), TThis);

  /*!\brief extract proportional elements of a state (Cp)
      \note in_extract_proportional is not required always.
            if a port is connected, state extraction is used.  */
  MAKE_IN_PORT(in_extract_proportional, void (const TRealVector &in, TRealVector &out), TThis);

  override void slot_initialize_exec (void);

  virtual const TRealVector&  out_distance_to_nearest_bf_get (void) const
    {return distance_to_nearest_bf_;}
  virtual const TRealVectorSet&  out_center_state_set_get (void) const
    {return center_state_set_;}

  /*!\brief for every basis function, calculate the distance to the nearest BF
      \param [out]neighbor_distance  resulted distance
      \note if the distance is too small, the maximum sqrt(eigenvalue) of its covariance is used  */
  void calc_distance_to_nearest_bf (TRealVector &neighbor_distance) const;

  /*!\brief for every basis function, calculate the distance to the nearest BF in extracted space (e.g.joint angle space)
      \param [out]neighbor_distance  resulted distance
      \note if the distance is too small, the maximum sqrt(eigenvalue) of its covariance (of extracted space) is used  */
  void calc_ext_distance_to_nearest_bf (TRealVector &neighbor_distance) const;

  /*! calculate maximum eigenvalue from eigen value vector, where StdDeviationInfinityThreshold is considered,
      i.e. if the sqrt(eigenvalue) > StdDeviationInfinityThreshold, the eigenvalue is ignored. */
  TReal get_eigvmax (const ComplexColumnVector &eigvalvector) const;

};  // end of MBasisFunctionsDCOBNGnet
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of basis_functions_ngnet_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_bf_ngnet_h
//-------------------------------------------------------------------------------------------
