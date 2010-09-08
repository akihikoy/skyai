//-------------------------------------------------------------------------------------------
/*! \file    bf_ngnet.cpp
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
#include <skyai/modules_std/bf_ngnet.h>
//-------------------------------------------------------------------------------------------
#include <lora/file.h>
#include <octave/EIG.h>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
namespace basis_functions_ngnet_detail
{
using namespace std;
// using namespace boost;


//===========================================================================================
// class MBasisFunctionsNGnet
//===========================================================================================

override void MBasisFunctionsNGnet::slot_initialize_exec (void)
{
  if (conf_.NGnetFileName=="")
    {LERROR("conf_.NGnetFileName must be specified!"); lexit(df);}
  string  filename(Agent().SearchFileName(conf_.NGnetFileName));
  if (filename=="")
    {LERROR("NGnet file "<<conf_.NGnetFileName<<" does not exist!"); lexit(df);}

  ngnet_.LoadFromFile (filename);
  if (ngnet_.size()==0)
    {LERROR("no Gaussians in "<<conf_.NGnetFileName); lexit(df);}
  int state_dim= ngnet_.unit(0).mu().length();
  ngnetcnf_.InitParam (TNGnetModelDyn::getXtDim(state_dim,1)/*xt*/,1/*y*/,state_dim/*x*/,1/*u*/);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MBasisFunctionsDCOBNGnet
//===========================================================================================

override void MBasisFunctionsDCOBNGnet::slot_initialize_exec (void)
{
  TParent::slot_initialize_exec();

  if (in_extract_proportional.ConnectionSize()==0)  // state extraction is not used
  {
    calc_distance_to_nearest_bf (distance_to_nearest_bf_);

    center_state_set_.resize(ngnet_.size());
    TRealVectorSet::iterator  csitr(center_state_set_.begin());
    for (TNGnet::const_iterator sitr(ngnet_.begin()); sitr!=ngnet_.end(); ++sitr,++csitr)
      (*csitr)= sitr->mu();
  }
  else // state extraction is used
  {
    calc_ext_distance_to_nearest_bf (distance_to_nearest_bf_);

    center_state_set_.resize(ngnet_.size());
    TRealVectorSet::iterator  csitr(center_state_set_.begin());
    for (TNGnet::const_iterator sitr(ngnet_.begin()); sitr!=ngnet_.end(); ++sitr,++csitr)
      in_extract_proportional.GetFirst (sitr->mu(), *csitr);
  }
}
//-------------------------------------------------------------------------------------------


/*!\brief for every basis function, calculate the distance to the nearest BF
    \param [out]neighbor_distance  resulted distance
    \note if the distance is too small, the maximum sqrt(eigenvalue) of its covariance is used  */
void MBasisFunctionsDCOBNGnet::calc_distance_to_nearest_bf (TRealVector &neighbor_distance) const
{
  LDEBUG("calc_distance_to_nearest_bf ...");
  neighbor_distance.resize(ngnet_.size());
  TypeExtS<TRealVector>::iterator ditr(GenBegin(neighbor_distance));
  for (TNGnet::const_iterator sitr(ngnet_.begin()); sitr!=ngnet_.end(); ++sitr, ++ditr)
  {
    TReal dmin (REAL_MAX), d;
    for (TNGnet::const_iterator snitr(ngnet_.begin()); snitr!=ngnet_.end(); ++snitr)
    {
      if(!conf_dcobngnet_.UsingMaxNorm)  {if(sitr!=snitr && (d=GetNorm(snitr->mu()-sitr->mu()))<dmin) dmin=d;}
      else                               {if(sitr!=snitr && (d=GetMaxNorm(snitr->mu()-sitr->mu()))<dmin) dmin=d;}
    }
    //!calculate eigen-value
    EIG  eig(sitr->Sigma());
    TReal eigvmax = get_eigvmax(eig.eigenvalues());
// LDBGVAR(eigvmax);
    *ditr=std::max(dmin, eigvmax);
// LDBGVAR(*ditr);
  }
  LDEBUG("calc_distance_to_nearest_bf ... done");
}
//-------------------------------------------------------------------------------------------

/*!\brief for every basis function, calculate the distance to the nearest BF in extracted space (e.g.joint angle space)
    \param [out]neighbor_distance  resulted distance
    \note if the distance is too small, the maximum sqrt(eigenvalue) of its covariance (of extracted space) is used  */
void MBasisFunctionsDCOBNGnet::calc_ext_distance_to_nearest_bf (TRealVector &neighbor_distance) const
{
  LDEBUG("calc_ext_distance_to_nearest_bf ...");
  ColumnVector  ext_diff;
  Matrix ext_sigma;
  neighbor_distance.resize(ngnet_.size());
  TypeExtS<TRealVector>::iterator ditr(GenBegin(neighbor_distance));
  for (TNGnet::const_iterator sitr(ngnet_.begin()); sitr!=ngnet_.end(); ++sitr, ++ditr)
  {
    TReal dmin (REAL_MAX), d;
    for (TNGnet::const_iterator snitr(ngnet_.begin()); snitr!=ngnet_.end(); ++snitr)
    {
      in_extract_proportional.GetFirst (snitr->mu()-sitr->mu(), ext_diff);
      if(!conf_dcobngnet_.UsingMaxNorm)  {if(sitr!=snitr && (d=GetNorm(ext_diff))<dmin) dmin=d;}
      else                               {if(sitr!=snitr && (d=GetMaxNorm(ext_diff))<dmin) dmin=d;}
    }

    //!calculate eigen-value
    LinearMapSigma (sitr->Sigma(), ext_sigma,
          boost::bind(&GET_PORT_TYPE(in_extract_proportional)::GetFirst, in_extract_proportional, _1,_2));
    /*dbg*/LASSERT1op1(ext_diff.length(),==,ext_sigma.rows());
    EIG  eig(ext_sigma);
    TReal eigvmax = get_eigvmax(eig.eigenvalues());
// LDBGVAR(eigvmax);
    *ditr=std::max(dmin, eigvmax);
// LDBGVAR(*ditr);
  }
  LDEBUG("calc_ext_distance_to_nearest_bf ... done");
}
//-------------------------------------------------------------------------------------------

/*! calculate maximum eigenvalue from eigen value vector, where StdDeviationInfinityThreshold is considered,
    i.e. if the sqrt(eigenvalue) > StdDeviationInfinityThreshold, the eigenvalue is ignored. */
TReal MBasisFunctionsDCOBNGnet::get_eigvmax (const ComplexColumnVector &eigvalvector) const
{
  const TReal  eigvth (Square(conf_dcobngnet_.StdDeviationInfinityThreshold));
  TReal res(0.0l), rev;
  bool every_eigv_diverges(true);
  for (const Complex *ev_itr(OctBegin(eigvalvector)); ev_itr!=OctEnd(eigvalvector); ++ev_itr)
  {
    rev= real(*ev_itr);
    if (rev<eigvth)
    {
      every_eigv_diverges= false;
      if (rev>res)  res= rev;
    }
  }
  if (every_eigv_diverges)  res= eigvth;
  return real_sqrt(res);
  // TReal eigvmax = real_sqrt(real(eig.eigenvalues().max()));
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MBasisFunctionsNGnet)
SKYAI_ADD_MODULE(MBasisFunctionsDCOBNGnet)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of basis_functions_ngnet_detail
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

