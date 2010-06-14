//-------------------------------------------------------------------------------------------
/*! \file    oldngnet.cpp
    \brief   liblora - NGnet and EM (old version: this implementation is DEPRECATED)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.09, 2009

    Copyright (C) 2008, 2009, 2010  Akihiko Yamaguchi

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
#include <lora/oldngnet.h>
//-------------------------------------------------------------------------------------------
#include <lora/small_classes.h>
#include <lora/file.h>
#include <lora/sys.h>
#include <lora/string_list.h>
#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>
#include <octave/EIG.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
using namespace boost;


//-------------------------------------------------------------------------------------------
TNGnetResult ngnetresult;
//-------------------------------------------------------------------------------------------

//! return true if x is included in val
template <class T>
inline bool is_included (const T &x, const std::list<T> &val )
{
  for (typename std::list<T>::const_iterator itr(val.begin());itr!=val.end();++itr)
    if (x==*itr)  return true;
  return false;
}
//-------------------------------------------------------------------------------------------

//! return true if i is included in val
template <class T>
inline bool is_included (typename std::list<T>::const_iterator i, const std::list<T> &val)
{
  for (typename std::list<T>::const_iterator itr(val.begin());itr!=val.end();++itr)
    if (i==itr)  return true;
  return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// struct TNGnetResult
//===========================================================================================

void TNGnetResult::open(const std::string &_dir)
{
  dir=_dir;
  if (!DirectoryExists(dir))
  {
    std::string str;
    std::cerr<<ioscc::blue<<dir<<" does not exist! continue?"<<std::endl;
    if (AskYesNo())  {std::cout<<"continue.."<<std::endl;}
    else               {std::cout<<"exit."<<std::endl; lexit(qfail);}
  }
  if (FileExists(dir+"ngnet.dat"))
  {
    std::string str;
    std::cerr<<ioscc::blue<<dir<<"ngnet.dat "<<ioscc::red<<"already exists!"<<ioscc::blue<<" continue?"<<std::endl;
    if (AskYesNo())  {std::cout<<"continue.."<<std::endl;}
    else               {std::cout<<"exit."<<std::endl; lexit(qfail);}
  }
  // uadir = dir+std::string("ua/");
  misc        .open((dir+"misc.dat").c_str());
  dbg         .open((dir+"debug.dat").c_str());
  fullEM      .open((dir+"fullEM.dat").c_str());
  pEM         .open((dir+"pEM.dat").c_str());
  // sample      .open((dir+"sampled-data.dat").c_str());
  // intervals   .open((dir+"intervals.dat").c_str());
  // merge_pair  .open((dir+"merge_pair.dat").c_str());
  // step_ssgm   .open((dir+"step_ssgm.dat").c_str());

  // no_unit_generated         .open((dir+std::string("no_unit_generated.dat")).c_str());
  // split_and_generate_failed .open((dir+std::string("split_and_generate_failed.dat")).c_str());
  // merge_failed              .open((dir+std::string("merge_failed.dat")).c_str());
}
//-------------------------------------------------------------------------------------------

TNGnetResult::~TNGnetResult (void)
{
  misc        .close();
  dbg         .close();
  fullEM      .close();
  pEM         .close();
  // sample      .close();
  // intervals   .close();
  // merge_pair  .close();
  // step_ssgm   .close();

  // no_unit_generated         .close();
  // split_and_generate_failed .close();
  // merge_failed              .close();
};
//-------------------------------------------------------------------------------------------





//===========================================================================================
// class TGaussianUnit
//===========================================================================================


void TGaussianUnit::CalculateParam (const TNGnetDataSetBase &data)  //!< calculate params from data
{
  if(data.size()<2)  return;
  calcMu (data);
  calcInvSigma (data);
  calcWt (data);
  calcSig2 (data);
}
//-------------------------------------------------------------------------------------------

void TGaussianUnit::calcMu (const TNGnetDataSetBase &data)
{
  const int T(data.size());
  SetZero(_mu);
  for (data.goFirst(); !data.isEnd(); data.increment())
    _mu += data.current().x();
  _mu = _mu / static_cast<TReal>(T);
  setmu(_mu);
}
//-------------------------------------------------------------------------------------------

void TGaussianUnit::calcInvSigma (const TNGnetDataSetBase &data)
  //! invSigma must be calculated after mu is obtained
{
  const int T(data.size());
  SetZero(_invSigma);
  for (data.goFirst(); !data.isEnd(); data.increment())
    _invSigma += (data.current().x()-mu())*(data.current().x()-mu()).transpose();
  _invSigma = _invSigma / static_cast<TReal>(T-1);
  {
    EIG eig(_invSigma);
    int dim(eig.eigenvalues().dim1());
    DiagMatrix ev(dim,dim,0.0);
    for(int r(0);r<dim;++r)
    {
      TReal dev (real(eig.eigenvalues()(r)));
      if (dev<Square(cnf.VAR_COMPONENT_MIN))       dev=Square(cnf.VAR_COMPONENT_MIN);
      else if (dev>Square(cnf.VAR_COMPONENT_MAX))  dev=Square(cnf.VAR_COMPONENT_MAX);
      ev(r,r) = 1.0/dev;
    }
    _invSigma = Matrix(real(eig.eigenvectors())*ev*OCT_INVERSE(real(eig.eigenvectors())));
  }//*/
  setinvSigma(_invSigma);
}
//-------------------------------------------------------------------------------------------

void TGaussianUnit::calcWt (const TNGnetDataSetBase &data)
{
  static const ColumnVector one(1,1.0);
  //SetZero(_Wt);
  Matrix   tmp_Wx (cnf.XTDIM(), cnf.XTDIM(), 0.0), tmp_W (cnf.YDIM(), cnf.XTDIM(), 0.0);
  for (data.goFirst(); !data.isEnd(); data.increment())
  {
    tmp_Wx += data.current().xt() * data.current().xt().transpose();
    tmp_W  += data.current().y() * data.current().xt().transpose();
  }
  setWt (tmp_W * OCT_INVERSE(tmp_Wx+cnf.WEIGHT_PENALTY));
}
//-------------------------------------------------------------------------------------------

void TGaussianUnit::calcSig2 (const TNGnetDataSetBase &data)
{
  const int T(data.size());
  SetZero(_sig2);
  for (data.goFirst(); !data.isEnd(); data.increment())
    _sig2 += GetNormSq(data.current().y() - out(data.current().xt()));
  _sig2 /= static_cast<TReal>(T-1);
  _sig2 /= static_cast<TReal>(cnf.YDIM());
  setsig2(_sig2);
}
//-------------------------------------------------------------------------------------------


void TGaussianUnit::SaveToStringList (TStringListEx &str_list, const string &prefix) const
{
  /*
      Wt
        (Matrix)
      end
      mu
        (ColumnVector)
      end
      invSigma
        (Matrix)
      sig2   sig2
    end
  */
  const string indent("  ");
  const string blank(" ");
  #define _SAVE(OBJ) \
    str_list.Add (prefix+indent+#OBJ); \
    loco_rabbits::SaveToStringList (_##OBJ, str_list, prefix+indent);
  _SAVE(Wt);
  _SAVE(mu);
  _SAVE(invSigma);
  str_list.Add (prefix+indent+"sig2"+blank+FloatToStr(_sig2));
  str_list.Add (prefix+"end");
  #undef _SAVE
}
//-------------------------------------------------------------------------------------------

void TGaussianUnit::LoadFromStringList (TStringListEx &str_list)
{
  clear();
  list<string> token;
  while(1)
  {
    token = str_list.Tokenize(true);
    list<string>::iterator itr = token.begin();
    if( token.empty() || IsComment(*itr) )
    {
    }
    else if( *itr == "Wt" )
    {
      str_list.Increment();
      loco_rabbits::LoadFromStringList(_Wt, str_list);
    }
    else if( *itr == "mu" )
    {
      str_list.Increment();
      loco_rabbits::LoadFromStringList(_mu, str_list);
    }
    else if( *itr == "invSigma" )
    {
      str_list.Increment();
      loco_rabbits::LoadFromStringList(_invSigma, str_list);
    }
    else if( *itr == "sig2" )
    {
      ++itr;
      _sig2 = StrToFloat(*itr);
    }
    else if( *itr == "end" )
      break;
    else
    {
      cerr << "invalid text format" << endl;
    }
    str_list.Increment();
  }
  update();
};
//-------------------------------------------------------------------------------------------

void TGaussianUnit::print_unit_area (ostream &os, const Matrix &mu_trans) const
  //! \param [in] mu_trans : R(2x(dim(mu))) matrix to reduce the dimension of mu to 2
{
  Matrix Sigma22(2,2,0.0);
  Sigma22 = OCT_INVERSE(mu_trans*_invSigma*mu_trans.transpose());
  ColumnVector tmu (2,0.0);
  tmu = mu_trans*_mu;
  for(TReal t(0.0l);t<=REAL_2PI;t+=0.05l)
  {
    TReal x =  real_sqrt(Sigma22(0,0)) * real_cos(t);
    TReal y =  Sigma22(0,1) / Sigma22(0,0)* x + real_sqrt(Sigma22(1,1) -Square(Sigma22(0,1))/Sigma22(0,0)) * real_sin(t);
    os  << 1.0l*x + tmu(0) << "  " << 1.0l*y + tmu(1);
    os  << "  0.0" << endl;
  }
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TNGnet
//===========================================================================================

void TNGnet::updateCache (TNGnetCacheBase &cache) const
{
  cache.setSumGval(0.0l);
  cache.setMaxGval_exp(-REAL_MAX);
  cache.setSymbol(-1);
  if (size() == 0)  return;
  int i(0);
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr, ++i)
  {
    itr->pred_likelihood = 0.0l;
    itr->cached_Gval =  itr->lnG(cache.data().x()) + itr->pred_likelihood;
    // itr->cached_Gval =  max<TReal>(itr->lnG(cache.data().x()) + itr->pred_likelihood, DBL_TINY_EXP);
    if(itr->cached_Gval > cache.maxGval_exp())
    {
      cache.setMaxGval_exp(itr->cached_Gval);
      cache.setSymbol(i);
    }
  }
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr)
    cache.setSumGval(cache.sumGval()+real_exp(itr->cached_Gval-cache.maxGval_exp()));

  ColumnVector &cPi (cache.setPi());
  TReal tmpdiv (cache.maxGval_exp() + real_log(cache.sumGval()));
  i=0;
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i)
  {
    cPi(i) = real_exp(itr->cached_Gval-tmpdiv);
    if (cPi(i)>1.0)
    {
      cerr<<"numerical error!!!"<<endl;
      cerr<<"P("<<i<<"|x)= "<<cPi(i)<<endl;
      cerr<<"cached_Gval= "<<itr->cached_Gval<<endl;
      cerr<<"tmpdiv= "<<tmpdiv<<endl;
    }
  }

  cache.setCached(true);
}
//-------------------------------------------------------------------------------------------

TReal TNGnet::Pi_x (int i, const ColumnVector &x) const
  //! return P(i|x)
{
  if(size() == 0)  return 0.0l;
  TReal sumGval(0.0l), maxGval_exp(-REAL_MAX);
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr)
  {
    itr->tmpGval = itr->lnG(x);
    if(itr->tmpGval > maxGval_exp) maxGval_exp = itr->tmpGval;
  }
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr)
    sumGval += real_exp(itr->tmpGval-maxGval_exp);
  return  real_exp(unit(i).tmpGval-maxGval_exp - real_log(sumGval));
}
//-------------------------------------------------------------------------------------------

ColumnVector TNGnet::Pi_x (const ColumnVector &x) const
  //! return {P(i|x) | i=0..M-1}
{
  ColumnVector res (size(),0.0);
  if(size() == 0)  return res;
  TReal sumGval(0.0), maxGval_exp(-REAL_MAX);
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr)
  {
    itr->tmpGval = itr->lnG(x);
    if(itr->tmpGval > maxGval_exp) maxGval_exp = itr->tmpGval;
  }
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr)
    sumGval += real_exp(itr->tmpGval-maxGval_exp);
  TReal tmpdiv (maxGval_exp + real_log(sumGval));
  int i(0);
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i)
    res(i) = real_exp(itr->tmpGval-tmpdiv);
  return res;
}
//-------------------------------------------------------------------------------------------

int  TNGnet::symbol (const ColumnVector &x) const
  //! get symbol index at x
{
  int i(0), index(-1);
  TReal maxGval_exp(-REAL_MAX);
  TReal tmp;
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr, ++i)
  {
    tmp = itr->lnG(x);
    if(tmp > maxGval_exp) { maxGval_exp = tmp; index = i; }
  }
  return index;
}
//-------------------------------------------------------------------------------------------

ColumnVector TNGnet::out (const ColumnVector &xt) const
  //! calculate output y for state xt
{
  ColumnVector out(cnf->YDIM(),0.0l);
  if(size() == 0)  return out;
  TReal sumGval(0.0l), maxGval_exp(-REAL_MAX);  // weight
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
  {
    itr->tmpGval = itr->lnG(TNGnetModelBase::extractX(xt,cnf->XDIM()));
    if(itr->tmpGval > maxGval_exp) maxGval_exp = itr->tmpGval;
  }
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    sumGval += real_exp(itr->tmpGval-maxGval_exp);
  TReal tmpdiv (maxGval_exp + real_log(sumGval));
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    out += real_exp(itr->tmpGval-tmpdiv) * itr->out(xt);
    // out += real_exp(itr->tmpGval-maxGval_exp - real_log(sumGval)) * itr->out(itr->getmu());
  return out;
};
//-------------------------------------------------------------------------------------------

ColumnVector TNGnet::out (TNGnetCacheBase &cache) const
  //! calculate output y for state xt
{
  ColumnVector out(cnf->YDIM(),0.0l);
  if(size() == 0)  return out;
  cacheIECU (cache);
  int i(0);
  for (const_iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i)
    out += cache.Pi()(i) * itr->out(cache.data().xt());
  return out;
};
//-------------------------------------------------------------------------------------------

TNGnet::unit_type TNGnet::genUnit (const ColumnVector &x) const
  //! calculate generalized param of gaussian unit at state x
{
  if (cnf==NULL) {LERROR("fatal in TNGnet::genUnit: cnf is NULL");lexit(df);}
  TNGnet::unit_type  out(*cnf);
  if(size() == 0)  return out;
  TReal sumGval(0.0l), maxGval_exp(-REAL_MAX);  // weight
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
  {
    itr->tmpGval = itr->lnG(x);
    if(itr->tmpGval > maxGval_exp) maxGval_exp = itr->tmpGval;
  }
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    sumGval += real_exp(itr->tmpGval-maxGval_exp);
  TReal tmpdiv (maxGval_exp + real_log(sumGval));
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    out.__weight_add (real_exp(itr->tmpGval-tmpdiv), *itr);
  out.afterUpdate();
  return out;
}
//-------------------------------------------------------------------------------------------

TNGnet::unit_type TNGnet::genUnit (TNGnetCacheBase &cache) const
  //! calculate generalized param of gaussian unit at state x; using cache
{
  if (cnf==NULL) {LERROR("fatal in TNGnet::genUnit: cnf is NULL");lexit(df);}
  TNGnet::unit_type  out(*cnf);
  if(size() == 0)  return out;
  cacheIECU (cache);
  int i(0);
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i )
    out.__weight_add (cache.Pi()(i), *itr);
  out.afterUpdate();
  return out;
}
//-------------------------------------------------------------------------------------------

TReal TNGnet::logLikelihood( void ) const
{
   const int M(size()), T(data->size());
   TReal likelihood(0.0l);
   data->goFirst();
   for(int t(0); t<T; ++t,data->increment())
   {
     const TNGnetModelBase &val(data->current());
     TReal Pdat(0.0l);
     for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
       Pdat += itr->prob(val,M);
     likelihood += Pdat;
   }
   return likelihood;
};
//-------------------------------------------------------------------------------------------

TReal TNGnet::Px (const ColumnVector &x) const
  //! calculate P(x|theta)
{
   const int M(size());
   if(M == 0)  return 0.0l;
   TReal p(0.0l);
   for (const_iterator itr(uset.begin());itr!=uset.end();++itr)
     p += itr->G(x);
   p /= static_cast<TReal>(M);  // =P(i|theta)
   return p;
}
//-------------------------------------------------------------------------------------------

TReal TNGnet::Py_xt (const ColumnVector &y, const ColumnVector &xt) const
  //! calculate P(y|xt,theta)
{
  if(size() == 0)  return 0.0l;
  TReal p(0.0l);
  TReal sumGval(0.0l), maxGval_exp(-REAL_MAX);  // weight
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
  {
    itr->tmpGval = itr->lnG(TNGnetModelBase::extractX(xt,cnf->XDIM()));
    if(itr->tmpGval > maxGval_exp) maxGval_exp = itr->tmpGval;
  }
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    sumGval += real_exp(itr->tmpGval-maxGval_exp);
  TReal tmpdiv (maxGval_exp + real_log(sumGval));
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    p += real_exp(itr->tmpGval-tmpdiv) * itr->Py_xt(y,xt); //!\bug itr->lnPy_xt(y,xt); bug fixed at 06.24.2008.
  return p;
}
//-------------------------------------------------------------------------------------------

TReal TNGnet::Py_xt (const ColumnVector &y, TNGnetCacheBase &cache) const
  //! calculate P(y|xt,theta); using cache
{
  if(size() == 0)  return 0.0l;
  TReal p(0.0l);
  cacheIECU (cache);
  int i(0);
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i )
    p += cache.Pi()(i) * itr->Py_xt(y,cache.data().xt()); //!\bug itr->lnPy_xt(y,xt); bug fixed at 06.24.2008.
  return p;
}
//-------------------------------------------------------------------------------------------

bool TNGnet::erase_small_posterior (void)
  //! erase units of small posterior; \note posterior will no longer be valid after this routine (run initEM)
{
  if (size()<2)  return false;
  bool erased(false);
  ColumnVector sum_posterior (posterior()*ColumnVector(data->size(),1.0l));
  //*dbg*/ngnetresult.dbg << "sum_posterior=  " << sum_posterior.transpose() << endl;
  iterator itr (begin());
  for (int i(0); i<sum_posterior.dim1(); ++i)
    if (sum_posterior(i) < cnf->ERASE_UNIT_THRESHOLD)
    {
      ngnetresult.dbg << "TNGnet::erase_small_posterior: erased unit " << i << ", sum_posterior= " << sum_posterior(i) << endl;
      itr = erase (itr);
      erased=true;
    }
    else
      ++itr;
  return erased;
}
//-------------------------------------------------------------------------------------------

bool TNGnet::divide_unit (void)
  //! divide units of large sig2; \note posterior will no longer be valid after this routine (run initEM)
{
  uset_type appended;
  int i(0);
  for (iterator itr(begin()); itr!=end(); ++itr,++i)
  {
    if (itr->sig2()>Square(cnf->UNIT_DIVIDING_SIG) && RollADice(cnf->UNIT_DIVIDING_PROB))
    {
      TNGnet::unit_type unit(*itr);  //! new unit
      EIG eig (itr->Sigma());
      #define eigvec(i)  (real(eig.eigenvectors().column(i)))
      #define eigval(i)  (real(eig.eigenvalues()(i)))
      // int dim (eig.eigenvectors().dim1());
      int maxdim (0);
      double eigvmax (eigval(0));
      for (int n(1);n<cnf->XDIM();++n)  if(eigval(n)>eigvmax) {eigvmax=eigval(n); maxdim=n;}
      unit.setmu (itr->mu() - cnf->UNIT_DIVIDING_RATE*real_sqrt(eigval(maxdim))*eigvec(maxdim));
      itr->setmu (itr->mu() + cnf->UNIT_DIVIDING_RATE*real_sqrt(eigval(maxdim))*eigvec(maxdim));
      /*method1*-/{
        Matrix sum (XDIM, XDIM, 0.0l);
        for (int n(0);n<XDIM-1;++n)  sum+=1.0l/eigval(n)*eigvec(n)*eigvec(n).transpose();
        itr->setinvSigma (4.0l/eigval(dim-1)*eigvec(dim-1)*eigvec(dim-1).transpose() + sum);
      }//*/
      /*method2*/{
        DiagMatrix ev(cnf->XDIM(),cnf->XDIM(),0.0);
        for(int r(0);r<cnf->XDIM();++r)
        {
          TReal div (eigval(r));
          if(r==maxdim) div/=4.0;
          if (div<Square(cnf->VAR_COMPONENT_MIN))  div=Square(cnf->VAR_COMPONENT_MIN);
          else if (div>Square(cnf->VAR_COMPONENT_MAX))  div=Square(cnf->VAR_COMPONENT_MAX);
          ev(r,r) = 1.0/div;
        }
        itr->setinvSigma (Matrix(real(eig.eigenvectors())*ev*OCT_INVERSE(real(eig.eigenvectors()))));
      }//*/
      unit.setinvSigma (itr->invSigma());
      itr->setsig2 (itr->sig2() * 0.5l);
      unit.setsig2 (itr->sig2());
      unit.setWt (itr->Wt());
      appended.push_back(unit);
      ngnetresult.dbg << "TNGnet::divide_unit: unit " << i << " is divided" << endl;
      #undef eigvec
      #undef eigval
    }
  }
  if (!appended.empty())
  {
    uset.splice (uset.end(), appended);
    return true;
  }
  else
    return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// TNGnet utility
//===========================================================================================

void TNGnet::SaveToStringList (TStringListEx &str_list, const string &prefix) const
{
  /*
      TGaussianUnit 0
        (TGaussianUnit)
      end
      ...
      TGaussianUnit 10
        (TGaussianUnit)
      end
    end
  */
  const string indent("  ");
  const string blank(" ");
  int i(0);
  for (const_iterator itr(begin()); itr!=end(); ++itr,++i)
  {
    str_list.Add( prefix+indent+string("TGaussianUnit")+blank+lexical_cast<string>(i) );
    itr->SaveToStringList (str_list, prefix+indent);
  }
  str_list.Add( prefix+string("end") );
}
//-------------------------------------------------------------------------------------------
void TNGnet::LoadFromStringList (TStringListEx &str_list)
{
   clear();
   list<string> token;
   while(1)
   {
     token = str_list.Tokenize(true);
     list<string>::iterator itr = token.begin();
     if( token.empty() || IsComment(*itr) )
     {
     }
     else if( *itr == "TGaussianUnit" )
     {
       if (cnf==NULL) {LERROR("fatal in TNGnet::LoadFromStringList: cnf is NULL");lexit(df);}
       str_list.Increment();
       add (TNGnet::unit_type(*cnf));
       back().LoadFromStringList (str_list);
     }
     else if( *itr == "end" )
       break;
     else
       cerr << "invalid text format" << endl;
     str_list.Increment();
   }
}
//-------------------------------------------------------------------------------------------


bool TNGnet::SaveToFile (const string &filename) const
{
  try
  {
    TStringListEx str_list;
    SaveToStringList (str_list, string(""));
    str_list.SaveToFile (filename);
    return true;
  }
  catch(...)
  {
    cerr << "fatal in TNGnet::SaveToFile: cannot save to  " << filename << endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------
bool TNGnet::LoadFromFile (const string &filename)
{
  try
  {
    TStringListEx str_list;
    str_list.LoadFromFile( filename );
    LoadFromStringList( str_list );
    return true;
  }
  catch(...)
  {
    cerr << "fatal in TNGnet::LoadFromFile: cannot load from  " << filename << endl;
    return false;
  }
}
//-------------------------------------------------------------------------------------------

void TNGnet::print_info (ostream &os) const
{
  TStringListEx str_list;
  SaveToStringList (str_list, string(""));
  str_list.PrintToStream (os);
}
//-------------------------------------------------------------------------------------------

void TNGnet::calc_mu_trans (void) const
{
  const int M (size());
  ColumnVector mean (cnf->XDIM(), 0.0l); // mean vector
  for (const_iterator itr(begin());  itr!=end(); ++itr)
    mean += itr->mu();
  mean = mean * (1.0l/static_cast<double>(M));  // `data size' in this context
  Matrix Cov (cnf->XDIM(),cnf->XDIM(),0.0l); // covariance matrix
  for (int c(0); c<Cov.cols(); ++c)
    for (int r(c); r<Cov.rows(); ++r)
    {
      for (const_iterator itr(begin());  itr!=end(); ++itr)
        Cov (r,c) += (itr->mu()(r)-mean(r)) * (itr->mu()(c)-mean(c));
      Cov (r,c) = Cov (r,c) * 1.0l/(static_cast<double>(M)-1.0l);
      if (r!=c)  Cov (c,r) = Cov (r,c);
    }
  EIG eig (Cov);
  mu_trans.resize (2,cnf->XDIM());
  mu_trans.insert (real(eig.eigenvectors().column(eig.eigenvalues().dim1()-1)).transpose(),0,0);
  mu_trans.insert (real(eig.eigenvectors().column(eig.eigenvalues().dim1()-2)).transpose(),1,0);
}
//-------------------------------------------------------------------------------------------

void TNGnet::print_unit_area (ostream &os, bool update_trans_matrix) const
{
  if (update_trans_matrix)
    calc_mu_trans();
  for (const_iterator itr(begin()); itr!=end(); ++itr)
  {
    itr->print_unit_area (os, mu_trans);
    os << endl << endl;
  }
}
//-------------------------------------------------------------------------------------------

void TNGnet::print_unit_label_gp (ostream &os, bool update_trans_matrix) const
{
  if (update_trans_matrix)
    calc_mu_trans();
  int i(0);
  ColumnVector tmu (2);
  for (const_iterator itr(begin()); itr!=end(); ++itr, ++i)
  {
    tmu = mu_trans * itr->mu();
    os << "set label "<<i+1<<" \""<<i<<"\" at "<<tmu(0)<<", "<<tmu(1)<<",  0 center nopoint front" << endl;
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// TNGnet EM-implementation
//===========================================================================================

void TNGnet::Estep( Matrix &Pit )
{
  const int M(size()), T(data->size());
  // E-step
  TReal sumGval, maxGval_exp;
  data->goFirst();
  for(int t(0); t<T; ++t,data->increment())
  {
    const TNGnetModelBase &val(data->current());
    maxGval_exp = -REAL_MAX;
    int i(0);
    for( iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i )
    {
      itr->cached_prob_e = itr->prob_exp(val,M);
      cachedPti(i,t) = itr->cached_prob_e;
      if(itr->cached_prob_e > maxGval_exp) maxGval_exp = itr->cached_prob_e;
    }
    sumGval = 0.0;
    for( iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i )
      sumGval += real_exp(itr->cached_prob_e-maxGval_exp);
    i = 0;
    sumGval = real_log(sumGval);
    for( iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i )
    {
      Pit(i,t) = real_exp(itr->cached_prob_e-maxGval_exp - sumGval);
      if (Pit(i,t)<DBL_TINY)  Pit(i,t)=DBL_TINY;
    }
  }
};
//-------------------------------------------------------------------------------------------

void TNGnet::singleMstep (const Matrix &Pit, const iterator &itr, int i)
{
  //*dbg*/SaveToFile (ngnetresult.dir+string("ngnet-dbg.dat"));
  const int T(data->size());
  static const ColumnVector one(1,1.0);

  TReal val_1, tmp1, tmp2;
  ColumnVector tmp_x (cnf->XDIM());
  Matrix       tmp_Sigma (cnf->XDIM(),cnf->XDIM());
  Matrix       tmp_Wx (cnf->XTDIM(), cnf->XTDIM()), tmp_W (cnf->YDIM(), cnf->XTDIM());
  #define WSUM(_r,_f) {\
    SetZero(_r); data->goFirst(); \
    for(int t(0);t<T;++t,data->increment()) {\
      _r += (_f)*Pit(i,t); } \
    _r = _r / static_cast<TReal>(T);}
  #define Dt data->current()
  WSUM(val_1, 1.0);
  //*!test*/if (val_1<1.0e-200) { cerr<<"faital "<<val_1<<endl; itr->SetRandomParam(); return; }
  /*!test*/if (val_1<DBL_TINY) {
      ngnetresult.dbg << "TNGnet::singleMstep, val_1 is invalid; unit " << i << ", val_1= " << val_1 << endl;
      LERROR("TNGnet::singleMstep, val_1 is invalid; unit " << i << ", val_1= " << val_1);
      // exit(1);
      /*if(size()<2)
        itr->SetRandomParam(*data);
      else
      {
        int p1 = Rand(0,size()-1), p2 = Rand(0,size()-1);
        while(p1==p2)  p2 = Rand(0,size()-1);
        itr->SetParamFromParent(unit(p1),unit(p2),*data);
      }
      return;*/ } //*/

  WSUM(tmp_x, Dt.x());
  itr->setmu (tmp_x / val_1);

  WSUM(tmp_Sigma, Dt.x()*Dt.x().transpose());
  {
    //*dbg*/ngnetresult.dbg<<"estimate invSigma..."<<endl;
    //*dbg*/ngnetresult.dbg<<Pit<<endl;
    //*dbg*/ngnetresult.dbg<<"T= "<<T<<endl;
    //*dbg*/ngnetresult.dbg<<"tmp_x="<<tmp_x.transpose()<<endl;
    //*dbg*/ngnetresult.dbg<<"itr->mu="<<itr->mu.transpose()<<endl;
    //*dbg*/ngnetresult.dbg<<tmp_Sigma<<val_1<<endl<<itr->mu * itr->mu.transpose();
    //*dbg*/ngnetresult.dbg<<"EIG..."<<endl<< tmp_Sigma / val_1 - itr->mu * itr->mu.transpose() << endl;
    EIG eig(tmp_Sigma / val_1 - itr->mu() * itr->mu().transpose());
    int dim(eig.eigenvalues().dim1());
    DiagMatrix ev(dim,dim,0.0);
    for(int r(0);r<dim;++r)
    {
      TReal dev (real(eig.eigenvalues()(r)));
      if (dev<Square(cnf->VAR_COMPONENT_MIN))       dev=Square(cnf->VAR_COMPONENT_MIN);
      else if (dev>Square(cnf->VAR_COMPONENT_MAX))  dev=Square(cnf->VAR_COMPONENT_MAX);
      ev(r,r) = 1.0/dev;
    }
    itr->setinvSigma (Matrix(real(eig.eigenvectors())*ev*OCT_INVERSE(real(eig.eigenvectors()))));
  }//*/

  WSUM(tmp_Wx, Dt.xt()*Dt.xt().transpose());
  WSUM(tmp_W, Dt.y() * Dt.xt().transpose());
  tmp_W = tmp_W * OCT_INVERSE(tmp_Wx+cnf->WEIGHT_PENALTY);
  itr->setWt (tmp_W);

  WSUM(tmp1, GetNormSq(Dt.y()));
  WSUM(tmp2, Trace( itr->out(Dt.xt()) * Dt.y().transpose() ));
  itr->setsig2 ((tmp1-tmp2)/static_cast<TReal>(cnf->YDIM())/val_1);
  #undef WSUM
}
//-------------------------------------------------------------------------------------------

void TNGnet::Mstep( const Matrix &Pit )
{
   // M-step
   int i(0);
   for( iterator itr(uset.begin()); itr!=uset.end(); ++itr,++i )
     singleMstep (Pit, itr, i);
};
//-------------------------------------------------------------------------------------------

void TNGnet::initEM (void)
{
   const int M(size()), T(data->size());
   Pit1.resize(M,T);
   Pit2.resize(M,T);
   cachedPti.resize(M,T);
   currentPit = &Pit1;
   Estep(*currentPit);
};
//-------------------------------------------------------------------------------------------

TReal TNGnet::updateParamsByEM(void)
{
   const int M(size()), T(data->size());
   Mstep(*currentPit);
   // calc Q
   Matrix *oldPit(currentPit);
   currentPit = (currentPit==&Pit1)?(&Pit2):(&Pit1);
   Estep(*currentPit);
   TReal Q(0.0);
   for(int t(0); t<T; ++t)
     for(int i(0); i<M; ++i)
       Q += (*oldPit)(i,t) * cachedPti(i,t);  //itr->prob_exp(val,M);
   return Q;
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
// TNGnet partial-EM-implementation
//===========================================================================================

void TNGnet::pEstep( Matrix &Pit )
{
   const int M(size()), T(data->size());
   // E-step
   TReal sumGval, maxGval_exp;
   data->goFirst();
   for(int t(0); t<T; ++t,data->increment())
   {
     const TNGnetModelBase &val(data->current());
     maxGval_exp = -REAL_MAX;
     for (list<int>::iterator itr(partial.begin()); itr!=partial.end(); ++itr)
     {
       funit[*itr]->cached_prob_e = funit[*itr]->prob_exp(val,M);
       cachedPti(*itr,t) = funit[*itr]->cached_prob_e;
       if(funit[*itr]->cached_prob_e > maxGval_exp) maxGval_exp = funit[*itr]->cached_prob_e;
     }
     sumGval = 0.0;
     for (list<int>::iterator itr(partial.begin()); itr!=partial.end(); ++itr)
       sumGval += real_exp(funit[*itr]->cached_prob_e-maxGval_exp);
     sumGval = real_log(sumGval);
     for (list<int>::iterator itr(partial.begin()); itr!=partial.end(); ++itr)
     {
       Pit(*itr,t) = real_exp(funit[*itr]->cached_prob_e-maxGval_exp - sumGval)*poldPt(t);
       if (Pit(*itr,t)<DBL_TINY)  Pit(*itr,t)=DBL_TINY;
     }
   }
};
//-------------------------------------------------------------------------------------------

void TNGnet::pMstep( const Matrix &Pit )
{
   // M-step
   ColumnVector one(1,1.0);
   for (list<int>::iterator itr(partial.begin()); itr!=partial.end(); ++itr)
     singleMstep (Pit, funit[*itr], *itr);;
};
//-------------------------------------------------------------------------------------------

void TNGnet::initpEM( list<int> &partialOld, list<int> &partialNew )
  /*! \param[in] partialOld  partial set that are old units (Pit is calculated)
      \param[in] partialNew  partial set that are new units (Pit is not calculated)
      \note posterior muse be calculated over old-params, before initpEM is called
      \note all elements of partialOld and partialNew are removed after initpEM */
{
   if(partialOld.empty())  { cerr<<"FATAL in pEM: partialOld is empty!"<<endl; lexit(qfail); }
   if(currentPit==NULL)
     initEM ();
   const int M(size()), T(data->size());
   const int deltaT (T-currentPit->cols());
   // currentPit = &Pit1;
   // Estep(*currentPit);  // calculate posterior
   // Pit2 = Pit1;
   int i(0);
   funit.resize(M); // update funit
   for (iterator itr(uset.begin());itr!=uset.end();++itr,++i)
     funit[i] = itr;
   poldPt.resize(T);
   for (int t(0);t<T-deltaT;++t)
   {
     poldPt(t) = 0.0;
     for (list<int>::const_iterator itr(partialOld.begin());itr!=partialOld.end();++itr)
       poldPt(t) += (*currentPit)(*itr,t);
   }
   for (int t(T-deltaT);t<T;++t)
     poldPt(t) = 1.0;

   partial.clear();
   partial.splice (partial.end(),partialOld);
   partial.splice (partial.end(),partialNew);

   cachedQ = 0.0;
   for(int t(0); t<T-deltaT; ++t)
     for (i=0; i<M; ++i)
       if (!is_included(i,partial))
         cachedQ += (*currentPit)(i,t) * cachedPti(i,t);

   Pit1.resize(M,T);
   Pit2.resize(M,T);
   cachedPti.resize(M,T);

   currentPit = &Pit1;
   pEstep(*currentPit);
};
//-------------------------------------------------------------------------------------------

TReal TNGnet::updateParamsBypEM(void)
{
   const int T(data->size());
   pMstep(*currentPit);
   // calc Q
   Matrix *oldPit(currentPit);
   currentPit = (currentPit==&Pit1)?(&Pit2):(&Pit1);
   pEstep(*currentPit);

   TReal Q(cachedQ);
   for(int t(0); t<T; ++t)
     for (list<int>::iterator itr(partial.begin()); itr!=partial.end(); ++itr)
       Q += (*oldPit)(*itr,t) * cachedPti(*itr,t);  //itr->prob_exp(val,M);
   return Q;
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
// tools
//===========================================================================================

extern "C" void fullNGnetEM (TNGnet &ngnet, TNGnet::dataset_type &data, const TReal &precision, int mvf_size,
    const std::string &index, const int &MAX_EM_ITERATIONS, bool (*call_back)(TNGnet &,const TReal &Q))
{
   ofstream &os (ngnetresult.fullEM);
   os << "#EM-main-"<< index <<":" << endl;
   os << "#  M,|data|= ("<<ngnet.size()<<", "<<data.size()<<")" << endl;
   TReal t1(GetUserTime());
   ngnet.setData(&data);
   ngnet.initEM(); // current posterior is calculated
   if (mvf_size<=0)
   {
     // os << "  full-EM is not executed" << endl;
     os << "#  update of full-EM is executed only one time" << endl;
     os << "Q= " << (ngnet.updateParamsByEM()) << endl;
     return;
   }
   TReal Q1(-REAL_MAX), Q2(0.0);
   static TMovingAverageFilter<TReal> MVF1(mvf_size), MVF2(mvf_size);
   int counter(0);
   int count (MAX_EM_ITERATIONS);
   do
   {
     Q2=Q1;
     os << "Q= " << (Q1=ngnet.updateParamsByEM()) << endl;
     // os << "logLikelihood= " << (ngnet.logLikelihood()) << endl;
     if(call_back && !call_back(ngnet,Q1)) break;
//      if (unit_area)
//      {
//        ofstream uos ((ngnetresult.uadir+*unit_area+lexical_cast<string>(counter)+string(".dat")).c_str());
//        //uos << endl << "proc-unit-area-EM" << lexical_cast<string>(counter) << ":" << endl;
//        ngnet.print_unit_area (uos);
//        uos.close();
//      }
     --count;
     ++counter;
   } while(MVF1(real_fabs(Q1-Q2))>precision*MVF2(real_fabs(Q1)) && count>0);
   os << "  full-EM done (" << (GetUserTime()-t1) << " sec)" << endl << endl;
}
//-------------------------------------------------------------------------------------------

void partialNGnetEM (TNGnet &ngnet, list<int> &partialOld, list<int> &partialNew, const TReal &precision, int mvf_size,
    const string &index, const int &MAX_PEM_ITERATIONS=25)
{
   ofstream &os (ngnetresult.pEM);
   os << "#partialEM-main-"<< index <<":" << endl;
   os << "#  M,|data|= ("<<(partialOld.size()+partialNew.size())<<", "<<ngnet.getData().size()<<")" << endl;
   TReal t1(GetUserTime());
   ngnet.initpEM (partialOld,partialNew);
   if (mvf_size<=0)
   {
     os << "  update of partial-EM is executed only one time" << endl;
     os << "  Q= " << (ngnet.updateParamsBypEM()) << endl;
     return;
   }
   TReal Q1(-REAL_MAX), Q2(0.0);
   static TMovingAverageFilter<TReal> MVF1(mvf_size), MVF2(mvf_size);
   int count (MAX_PEM_ITERATIONS);
   do
   {
     Q2=Q1;
     os << "Q= " << (Q1=ngnet.updateParamsBypEM()) << endl;
     // os << "logLikelihood= " << (ngnet.logLikelihood()) << endl;
     --count;
   } while (MVF1(real_fabs(Q1-Q2))>precision*MVF2(real_fabs(Q1)) && count>0);
   os << "  partial-EM done (" << (GetUserTime()-t1) << " sec)" << endl << endl;
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

