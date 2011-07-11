//-------------------------------------------------------------------------------------------
/*! \file    oldngnet.h
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
#ifndef loco_rabbits_oldngnet_h
#define loco_rabbits_oldngnet_h
//-------------------------------------------------------------------------------------------
#include <lora/math.h>
#include <lora/stl_ext.h>
#include <lora/stl_math.h>
#include <lora/type_gen_oct.h>
#include <lora/octave.h>
#include <lora/octave_str.h>
#include <lora/setting_file.h> // for TNGnetConfiguration
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
struct TNGnetResult
//===========================================================================================
{
  std::ostream &msg;
  std::string dir; //, uadir;
  std::ofstream  misc, dbg, fullEM, pEM;
  std::ofstream  sample;
  // std::ofstream  intervals, merge_pair, step_ssgm;
  // std::ofstream  no_unit_generated, split_and_generate_failed, merge_failed;
  void open(const std::string &_dir);

  TNGnetResult (void) : msg(std::cout) {};
  TNGnetResult (const std::string &_dir, bool _open=true)
      : msg(std::cout), dir(_dir)  {if (_open) open(dir);};

  ~TNGnetResult (void);
};
extern TNGnetResult ngnetresult;
//-------------------------------------------------------------------------------------------

#define GENT_API_ITR template <typename T, typename InItr> inline
#define GENT_API_CNT template <typename T, typename Container> inline
//-------------------------------------------------------------------------------------------
enum TWeightConstraint {wcNone=0, wcInertiaSymmetry};
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
struct TError
//-------------------------------------------------------------------------------------------
{
   TReal rmse;
   TReal avr;
   TReal max;
   TReal min;
   void SetZero (void)  { rmse=0.0; avr=0.0; max=0.0; min=0.0; };
   TError (void)  {SetZero();};
   TError (const TReal &_rmse, const TReal &_avr,const TReal &_max,const TReal &_min)
     : rmse(_rmse), avr(_avr), max(_max), min(_min)  {};
   bool operator<  (const TError &rhs) const { return rmse <  rhs.rmse; };
   bool operator<= (const TError &rhs) const { return rmse <= rhs.rmse; };
   bool operator>  (const TError &rhs) const { return rmse >  rhs.rmse; };
   bool operator>= (const TError &rhs) const { return rmse >= rhs.rmse; };
   bool operator== (const TError &rhs) const { return rmse == rhs.rmse; };
   void print (std::ostream &os, const std::string &prefix="") const
     {
       os << prefix << "rmse =\t"    << rmse    << std::endl;
       os << prefix << "avr =\t"     << avr     << std::endl;
       os << prefix << "max =\t"     << max     << std::endl;
       os << prefix << "min =\t"     << min     << std::endl;
     };
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TNGnetConfiguration
//===========================================================================================
{
private:
  // TNGnetModelBase *model;
  int xtdim, ydim, xdim, udim;
  TReal pi2_n2;
  TReal pi2_d2;
  TReal pi2_nd2;
  TReal log_pi2_n2;
  TReal log_pi2_d2;
  TReal log_pi2_nd2;

  bool nodim(void) const
    {return xtdim<0||ydim<0||xdim<0||udim<0;};
  void calc_const(void)
    {
      if(nodim())
        {
          pi2_n2=pi2_d2=pi2_nd2=0.0l;
          log_pi2_n2=log_pi2_d2=log_pi2_nd2=0.0l;
          return;
        };
      pi2_n2  = 1.0l / real_pow(2.0l*REAL_PI,static_cast<TReal>(xdim)*0.5l);
      pi2_d2  = 1.0l / real_pow(2.0l*REAL_PI,static_cast<TReal>(ydim)*0.5l);
      pi2_nd2 = 1.0l / real_pow(2.0l*REAL_PI,static_cast<TReal>(xdim+ydim)*0.5l);

      log_pi2_n2  = real_log(pi2_n2 );
      log_pi2_d2  = real_log(pi2_d2 );
      log_pi2_nd2 = real_log(pi2_nd2);
    };
public:
  const int&     XTDIM(void) const {return xtdim;};
  const int&     XDIM(void)  const {return xdim;};
  const int&     YDIM(void)  const {return ydim;};
  const int&     UDIM(void)  const {return udim;};
  const TReal    rXTDIM(void) const {return static_cast<TReal>(xtdim);};
  const TReal    rXDIM(void)  const {return static_cast<TReal>(xdim);};
  const TReal    rYDIM(void)  const {return static_cast<TReal>(ydim);};
  const TReal    rUDIM(void)  const {return static_cast<TReal>(udim);};
  const TReal&   PI2_N2 (void) const {return pi2_n2;};
  const TReal&   PI2_D2 (void) const {return pi2_d2;};
  const TReal&   PI2_ND2(void) const {return pi2_nd2;};
  const TReal&   log_PI2_N2 (void) const {return log_pi2_n2;};
  const TReal&   log_PI2_D2 (void) const {return log_pi2_d2;};
  const TReal&   log_PI2_ND2(void) const {return log_pi2_nd2;};

  int                 seed;
  TWeightConstraint   WEIGHT_CONST;
  TReal               VAR_COMPONENT_MIN;
  TReal               VAR_COMPONENT_MAX;
  TReal               SIG_COMPONENT_MIN;
  TReal               SIG_COMPONENT_MAX;
  TReal               ERASE_UNIT_THRESHOLD;
  TReal               UNIT_DIVIDING_PROB;
  TReal               UNIT_DIVIDING_SIG;
  TReal               UNIT_DIVIDING_RATE;
  Matrix              WEIGHT_PENALTY;

  void InitParam (int _xtdim, int _ydim, int _xdim, int _udim=0)
    {
      xtdim=_xtdim;
      ydim=_ydim;
      xdim=_xdim;
      udim=_udim;
      calc_const();

      seed                = (unsigned)time(NULL);
      VAR_COMPONENT_MIN   = 1.0e-6;
      VAR_COMPONENT_MAX   = 1.0e+6;
      SIG_COMPONENT_MIN   = 1.0e-6;
      SIG_COMPONENT_MAX   = 1.0e+6;
      ERASE_UNIT_THRESHOLD= 1.0e-100;
      UNIT_DIVIDING_PROB  = 0.1;
      UNIT_DIVIDING_SIG   = 1.0;
      UNIT_DIVIDING_RATE  = 0.5;
      if(nodim())
        WEIGHT_PENALTY    = Matrix(0,0);
      else
        WEIGHT_PENALTY    = Matrix(DiagMatrix(XTDIM(),XTDIM(),1.0e-7));
    };
  void SET_WEIGHT_PENALTY (const TReal &val)
    {for(int r(0); r<WEIGHT_PENALTY.rows(); ++r) WEIGHT_PENALTY(r,r)=val;};

  TNGnetConfiguration(int _xtdim, int _ydim, int _xdim, TWeightConstraint wc=wcNone)
      : WEIGHT_CONST(wc)
    {
      InitParam(_xtdim, _ydim, _xdim);
    };
  TNGnetConfiguration(int _xtdim, int _ydim, int _xdim, int _udim, TWeightConstraint wc=wcNone)
      : WEIGHT_CONST(wc)
    {
      InitParam(_xtdim, _ydim, _xdim, _udim);
    };

  void constrainWt(Matrix Wt) const
    {
      switch(WEIGHT_CONST)
      {
        case wcNone:  break;
        case wcInertiaSymmetry:
          FIXME("implement a constraint on parameters w.r.t. A");
          break;
        default:
          LERROR("invalid WEIGHT_CONST: "<<WEIGHT_CONST); lexit(df);
      }
    };

  #define PARAMETER_LIST \
    { \
      const std::string gname ("TNGnetConfiguration"); \
      SETTING_FILE_IO(   ,  xtdim                   ); \
      SETTING_FILE_IO(   ,  ydim                    ); \
      SETTING_FILE_IO(   ,  xdim                    ); \
      SETTING_FILE_IO(   ,  udim                    ); \
      SETTING_FILE_IO(   ,  seed                    ); \
      SETTING_FILE_IO(I  ,  WEIGHT_CONST            ); \
      SETTING_FILE_IO(   ,  VAR_COMPONENT_MIN       ); \
      SETTING_FILE_IO(   ,  VAR_COMPONENT_MAX       ); \
      SETTING_FILE_IO(   ,  SIG_COMPONENT_MIN       ); \
      SETTING_FILE_IO(   ,  SIG_COMPONENT_MAX       ); \
      SETTING_FILE_IO(   ,  ERASE_UNIT_THRESHOLD    ); \
      SETTING_FILE_IO(   ,  UNIT_DIVIDING_PROB      ); \
      SETTING_FILE_IO(   ,  UNIT_DIVIDING_SIG       ); \
      SETTING_FILE_IO(   ,  UNIT_DIVIDING_RATE      ); \
      SETTING_FILE_IO(SL2,  WEIGHT_PENALTY          ); \
    }
  void save(TSettingFile &sf) const
    {
      #define SETTING_FILE_IO(type,param)  saveToSettingFile##type(sf,gname,param)
      PARAMETER_LIST
      #undef SETTING_FILE_IO
    };
  void load(TSettingFile &sf)
    {
      #define SETTING_FILE_IO(type,param)  loadFromSettingFile##type(sf,gname,param)
      PARAMETER_LIST
      #undef SETTING_FILE_IO
      calc_const();
    };
  #undef PARAMETER_LIST
  void print(std::ostream &os) const
    {
      TSettingFile sf;
      save (sf);
      sf.PrintToStream (os);
    };
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief interface of data-model
class TNGnetModelBase
//===========================================================================================
{
protected:
  // const TNGnetConfiguration& cnf;
  ColumnVector _xt, _y;
  ColumnVector _x;
public:
  TNGnetModelBase(void) {};
  virtual ~TNGnetModelBase(void) {};
  virtual const TNGnetModelBase& operator=(const TNGnetModelBase& rhs)
    {_xt=rhs._xt; _y=rhs._y; _x=rhs._x; return *this;};

  const ColumnVector& xt(void) const {return _xt;};
  const ColumnVector& x(void)  const {return _x;};
  const ColumnVector& y(void)  const {return _y;};

  virtual void setX(const ColumnVector &__x)  = 0;
  virtual void setY(const ColumnVector &__y)  {_y=__y;};

  virtual void clear(void) {_xt=ColumnVector(0); _x=_xt; _y=_xt;};
  virtual void print (std::ostream &os) const
    {os<<"xt= "<<_xt.transpose()<<",  y= "<<_y.transpose()<<std::endl;};

  static ColumnVector extractX(const ColumnVector &__xt, int _xdim) {return __xt.extract_n(0,_xdim);};

  // static void constrainWt(Matrix Wt) {LERROR("invalid call of TNGnetModelBase::constrainWt"); lexit(df);};
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief data model of a generic i/o system (x,y)=(input,output); y=Wx+b
class TNGnetModelIO : public TNGnetModelBase
//===========================================================================================
{
protected:
public:
  TNGnetModelIO(void) {};
  TNGnetModelIO(int xdim, int ydim)
    {
      if(xdim>0) _x=ColumnVector(xdim,0.0l);
      if(ydim>0) _y=ColumnVector(ydim,0.0l);
      if(xdim>0) _xt=ColumnVector(xdim+1,0.0l);
    };
  TNGnetModelIO(const ColumnVector &__x, const ColumnVector &__y)
    {
      //static const ColumnVector one(1,1.0);
      _x=__x;
      _y=__y;
      _xt.resize(getXtDim(_x.length()));
      _xt.insert(_x,0);
      _xt(_x.length())=1.0;
      //_xt=_x.stack(one);
    };
  /*override*/const TNGnetModelIO& operator=(const TNGnetModelIO& rhs)
    {TNGnetModelBase::operator=(rhs); return *this;};

  static ColumnVector getXt(const ColumnVector &__x)
    {
      ColumnVector __xt(getXtDim(__x.length()));
      __xt.insert(__x,0);
      __xt(__x.length())=1.0;
      return __xt;
    };
  static int getXtDim(int _xdim)
    {return _xdim+1;};

  /*override*/void setX(const ColumnVector &__x)
    {
      _x=__x;
      _xt.resize(getXtDim(_x.length()));
      _xt.insert(_x,0);
      _xt(_x.length())=1.0;
    };

  // clear(void) {TNGnetModelBase::clear();};
  // static void constrainWt(Matrix Wt)  {/*do nothing*/};
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief data model of a dynamical system (x,u,y)=(state,input,output); y=Ax+Bu+d
class TNGnetModelDyn : public TNGnetModelBase
//===========================================================================================
{
protected:
  ColumnVector _u;
public:
  TNGnetModelDyn(void) {};
  TNGnetModelDyn(const TNGnetModelDyn &val) : TNGnetModelBase(val), _u(val._u) {};
  TNGnetModelDyn(int xdim, int udim, int ydim)
    {
      if(xdim>0) _x=ColumnVector(xdim,0.0l);
      if(udim>0) _u=ColumnVector(udim,0.0l);
      if(ydim>0) _y=ColumnVector(ydim,0.0l);
      if(xdim+udim>0) _xt=ColumnVector(xdim+udim+1,0.0l);
    };
  TNGnetModelDyn(const ColumnVector &__x, const ColumnVector &__u, const ColumnVector &__y)
    {
      //static const ColumnVector one(1,1.0);
      _x=__x;
      _u=__u;
      _y=__y;
      _xt.resize(getXtDim(_x.length(),_u.length()));
      _xt.insert(_x,0);
      _xt.insert(_u,_x.length());
      _xt(_x.length()+_u.length())=1.0;
      //_xt=_x.stack(_u.stack(one));
    };
  /*override*/const TNGnetModelDyn& operator=(const TNGnetModelDyn& rhs)
    {_u=rhs._u; TNGnetModelBase::operator=(rhs); return *this;};

  const ColumnVector& u(void) const {return _u;};

  static ColumnVector getXt(const ColumnVector &__x, const ColumnVector &__u)
    {
      ColumnVector __xt(getXtDim(__x.length(),__u.length()));
      __xt.insert(__x,0);
      __xt.insert(__u,__x.length());
      __xt(__x.length()+__u.length())=1.0;
      return __xt;
    };
  static int getXtDim(int _xdim, int _udim)
    {return _xdim+_udim+1;};

  /*override*/void setX(const ColumnVector &__x)
    {
      _x=__x;
      _xt.resize(getXtDim(_x.length(),_u.length()));
      _xt.insert(_x,0);
      _xt(_x.length()+_u.length())=1.0;
    };
  virtual void setU(const ColumnVector &__u)
    {
      _u=__u;
      _xt.resize(getXtDim(_x.length(),_u.length()));
      _xt.insert(_u,_x.length());
      _xt(_x.length()+_u.length())=1.0;
    };

  /*override*/void clear(void) {TNGnetModelBase::clear(); _u=_xt;};
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
//! \brief interface of data-set classes
class TNGnetDataSetBase
//===========================================================================================
{
private:
public:
  virtual void                    goFirst (void) const = 0;
  virtual bool                    isEnd (void) const = 0;
  virtual void                    increment (void) const = 0;
  virtual const TNGnetModelBase&  current (void) const = 0;
  virtual int                     size (void) const = 0;
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! \brief template class of the data-set
template <typename TModel, typename container=std::list<TModel> >
class TNGnetDataSet : public TNGnetDataSetBase, public container
//===========================================================================================
{
public:
  typedef TNGnetModelBase TDatum;
private:
  mutable typename container::const_iterator citr;
public:
  /*override*/void            goFirst (void) const {citr=container::begin();};
  /*override*/bool            isEnd (void) const {return citr==container::end();};
  /*override*/void            increment (void) const {++citr;};
  /*override*/const TDatum&   current (void) const {return *citr;};
  /*override*/int             size (void) const {return container::size();};
};
//-------------------------------------------------------------------------------------------



//===========================================================================================
//! \brief Gaussian Unit
class TGaussianUnit
//===========================================================================================
{
friend  class TNGnet;
protected:
  const TNGnetConfiguration& cnf;

  Matrix                _Wt;             //!< regression weight
  ColumnVector          _mu;             //!< average vector
  Matrix                _invSigma;       //!< inverse of covariance matrix
  TReal                 _sig2;           //!< variance

  TReal                 _detInvSigma;
  TReal                 _logDetInvSigma;
  Matrix                _Sigma;

  mutable TReal cached_prob_e;   //!< temporary val of prob_exp, used in EM
  mutable TReal tmpGval;         //!< temporary val of gaussian
  mutable TReal cached_Gval;     //!< cached val of gaussian
  mutable TReal pred_likelihood; //!< P(y|y_pred)

  void updateWt       (void)
    {
      //FIXME EnsureSymmetry (_Wt);
//       FIXME("EnsureSymmetry(_Wt)");
      //TModel::constrainWt(_Wt);
      cnf.constrainWt(_Wt);
    };
  void updatemu       (void)
    {
    };
  void updateinvSigma (void);
  void updatesig2     (void)
    {
      if(_sig2 < Square(cnf.SIG_COMPONENT_MIN))  _sig2=Square(cnf.SIG_COMPONENT_MIN);
      if(_sig2 > Square(cnf.SIG_COMPONENT_MAX))  _sig2=Square(cnf.SIG_COMPONENT_MAX);
    };
  void update (void)
    {
      updateWt      ();
      updatemu      ();
      updateinvSigma();
      updatesig2    ();
    };

public:
  TGaussianUnit(const TNGnetConfiguration &__cnf) : cnf(__cnf)  {clear();};

  void clear (void)
    {
      #define ZD_(x_n,x_i) x_n((x_i)>0?(x_i):0)
      int ZD_(xdim,cnf.XDIM()), ZD_(xtdim,cnf.XTDIM()), ZD_(ydim,cnf.YDIM());
      #undef ZD_
      #define _SETZERO_M(x,r,c)  x.resize(r,c); x.fill(0.0l);
      #define _SETZERO_V(x,r)    x.resize(r); x.fill(0.0l);
      _SETZERO_M      (_Wt, ydim, xtdim);
      _SETZERO_V      (_mu, xdim);
      _SETZERO_M      (_invSigma, xdim, xdim);
      _sig2           =0.0l;
      _detInvSigma    =0.0l;
      _logDetInvSigma =0.0l;
      _SETZERO_M      (_Sigma, xdim, xdim);
      #undef _SETZERO_M
      #undef _SETZERO_V
    };

  const Matrix&       Wt             (void) const {return _Wt;};
  const ColumnVector& mu             (void) const {return _mu;};
  const Matrix&       invSigma       (void) const {return _invSigma;};
  const TReal&        sig2           (void) const {return _sig2;};
  const TReal&        detInvSigma    (void) const {return _detInvSigma;};
  const TReal&        logDetInvSigma (void) const {return _logDetInvSigma;};
  const Matrix&       Sigma          (void) const {return _Sigma;};

  void  setWt       (const Matrix&       __Wt      )  {_Wt       = __Wt      ; updateWt      ();};
  void  setmu       (const ColumnVector& __mu      )  {_mu       = __mu      ; updatemu      ();};
  void  setinvSigma (const Matrix&       __invSigma)  {_invSigma = __invSigma; updateinvSigma();};
  void  setsig2     (const TReal&        __sig2    )  {_sig2     = __sig2    ; updatesig2    ();};

  // inline void SetRandomParam (void);
  // template<class _container>
  // void SetRandomParam (const _container &data);
  // template<class _container>
  // void SetParamFromParent (const TGaussianUnit &parent1, const TGaussianUnit &parent2, const _container &data);

protected:
  const TGaussianUnit& operator+= (const TGaussianUnit &rhs)
    {
      _Wt       += rhs._Wt        ;
      _mu       += rhs._mu        ;
      _invSigma += rhs._invSigma  ;
      _sig2     += rhs._sig2      ;
      update();
      return *this;
    };
  const TGaussianUnit& operator*= (const double &rhs)
    {
      _Wt       = _Wt       * rhs  ;
      _mu       = _mu       * rhs  ;
      _invSigma = _invSigma * rhs  ;
      _sig2     = _sig2     * rhs  ;
      update();
      return *this;
    };
public:
  void afterUpdate (void)  {update();};
  const TGaussianUnit& __weight_add (const double &w, const TGaussianUnit &rhs)
      //! \note After executing this function,  afterUpdate must be called!
    {
      _Wt       += w * rhs._Wt        ;
      _mu       += w * rhs._mu        ;
      _invSigma += w * rhs._invSigma  ;
      _sig2     += w * rhs._sig2      ;
      // update();
      return *this;
    };
public:
  ColumnVector out (const ColumnVector &__xt) const  {return _Wt*__xt;};
  ColumnVector out (const TNGnetModelBase &val) const { return out (val.xt()); };
  TReal prob_exp (const TNGnetModelBase &val, int M) const //! \param M: number of unit of NGnet
    {
      ColumnVector Wtx = out(val);
      // TReal tmp1 = PI2_ND2 / real_pow(sig2,0.5*static_cast<TReal>(YDIM)) * real_sqrt(invSigma.determinant().value())
      //              / static_cast<TReal>(M);
      TReal tmp1 = cnf.log_PI2_ND2() - 0.5*cnf.rYDIM()*real_log(_sig2)
                   + 0.5*_logDetInvSigma - real_log(static_cast<TReal>(M));
      TReal tmp2 = -0.5*Quadratic(val.x()-_mu, _invSigma)
                   -0.5/_sig2*SquareSum(val.y()-Wtx);

      return tmp1+tmp2; // real_log(tmp1)+tmp2;
    };
  TReal prob( const TNGnetModelBase &val, int M ) const //! P(xt,y,i|theta) \param M: unit count of NGnet
    { return real_exp(prob_exp(val,M)); };
  TReal lnG (const ColumnVector &x) const //! ln(P(x|i,theta)) = ln(Gi(x))
    { return cnf.log_PI2_N2() + 0.5l*_logDetInvSigma + (-0.5l*Quadratic(x-_mu,_invSigma)); };  // real_log(PI2_N2 * real_sqrt(invSigma.determinant().value()))
  TReal G (const ColumnVector &x) const //! P(x|i,theta) = Gi(x)
    { return (cnf.PI2_N2() * real_sqrt(_detInvSigma))
             * real_exp(-0.5*Quadratic(x-_mu, _invSigma)); };
  TReal lnPy_xt (const ColumnVector &y, const ColumnVector &__xt) const  //! ln(P(y|__xt,i,theta))
    { return real_log(cnf.PI2_D2() / real_pow(_sig2,0.5*cnf.rYDIM()))  // TODO make efficient computation
             + (-0.5/_sig2*SquareSum(y-out(__xt))); };
  TReal Py_xt (const ColumnVector &y, const ColumnVector &__xt) const  //! P(y|__xt,i,theta)
    { return cnf.PI2_D2() / real_pow(_sig2,0.5*cnf.rYDIM())
             * real_exp(-0.5/_sig2*SquareSum(y-out(__xt))); };

  void CalculateParam (const TNGnetDataSetBase &data);  //!< calculate params from data
  void calcMu         (const TNGnetDataSetBase &data);
  void calcInvSigma   (const TNGnetDataSetBase &data);
  void calcWt         (const TNGnetDataSetBase &data);
  void calcSig2       (const TNGnetDataSetBase &data);

//   template<class _container>
//   void CalculateParam (const _container &data);  //!< calculate params from data
//   template<class _container>
//   void calcMu (const _container &data);
//   template<class _container>
//   void calcW (const _container &data);
//   template<class _container>
//   void calcInvSigma (const _container &data);
//   template<class _container>
//   void calcSig2 (const _container &data);

  void SaveToStringList (TStringListEx &str_list, const std::string &prefix) const;
  void LoadFromStringList (TStringListEx &str_list);
  void print_unit_area (std::ostream &os, const Matrix &mu_trans) const;
    //!< \param [in] mu_trans : R(2x(dim(mu))) matrix to reduce the dimension of mu to 2
};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
class TNGnetCacheBase
//-------------------------------------------------------------------------------------------
{
protected:
  struct type_cache
  {
    int    index; //!< cache index, such as time-index, which must be greater-eq than 0, if 0, cache is initialized
    bool   cached;
    int    symbol;
    TReal  sumGval, maxGval_exp;
    ColumnVector Pi;
    TNGnetModelBase  *data;
    // TReal  time_step;  // used to calculate predicted y
    void reset (int ucount)
      {
        index=-1;
        cached=false;
        symbol=0;
        sumGval=0.0l; maxGval_exp=0.0l;
        Pi.resize(ucount); SetZero(Pi);
        if(data)  data->clear();
        // time_step=0.0l;
      };
    type_cache (void) : data(NULL)  {reset(0);};
    void iec (int i) const //!< index error check
      {if (index!=i){
          LERROR("cache-index is not consistent! (cache->index="<<index
            <<", given="<<i<<"); execute setCurrentState before using cache");
          lexit(df);}};
    void print (std::ostream &os) const  //!< print cached data
      {
        os<<"index= "<<index<<std::endl;
        os<<"symbol= "<<symbol<<std::endl;
        os<<"sumGval= "<<sumGval<<std::endl;
        os<<"maxGval_exp= "<<maxGval_exp<<std::endl;
        os<<"Pi= "<<Pi.transpose()<<std::endl;
        // os<<"time_step= "<<time_step<<std::endl;
        data->print (os);
      };
  };
  void iec(int cindex) {cache->iec(cindex);}
  type_cache cache1, cache2, *cache, *oldcache;
  TNGnetCacheBase(void) {reset(0);};
  TNGnetCacheBase(int ucount) {reset(ucount);};

  void setCurrentState (const TNGnetModelBase &d, int cindex, bool enforce=false)
    {
      if (cindex>=0 && cindex==cache->index && !enforce)
      {
        // if (d.x != cache->data.x)
        //   cerr << "cache is not consistent!" << endl;
        // cerr << "cache is used at " << cindex << endl;
        return;
      };
      if (cindex<=0)
        reset(cache->Pi.length());
      else
      {
        if (cache->index != cindex-1)
          LERROR("in TNGnet::setCurrentState(), too many steps between "<<cache->index<<" and "<<cindex);
        std::swap (cache,oldcache);
      }
      // if (d.x == oldcache->data.x)
      //   cerr << "the same state is cached agein " << d.x.transpose() << endl;
      cache->index = cindex;
      cache->cached = false;
      // cache->time_step = dt;

      cache->sumGval = 0.0l;
      cache->maxGval_exp = -REAL_MAX;
      cache->symbol = -1;
    };

public:
  const int&    index()        const {return cache->index;       };
  const bool&   cached()       const {return cache->cached;      };
  const int&    symbol()       const {return cache->symbol;      };
  const TReal&  sumGval()      const {return cache->sumGval;     };
  const TReal&  maxGval_exp()  const {return cache->maxGval_exp; };
  const ColumnVector& Pi()     const {return cache->Pi;          };
  const TNGnetModelBase&  data() const {return *(cache->data);   };

  const int&    oldIndex()        const {return oldcache->index;       };
  const bool&   oldCached()       const {return oldcache->cached;      };
  const int&    oldSymbol()       const {return oldcache->symbol;      };
  const TReal&  oldSumGval()      const {return oldcache->sumGval;     };
  const TReal&  oldMaxGval_exp()  const {return oldcache->maxGval_exp; };
  const ColumnVector& oldPi()     const {return oldcache->Pi;          };
  const TNGnetModelBase&  oldData() const {return *(oldcache->data);   };

  const int&    setIndex       (const int&   x)  {cache->index      =x; return cache->index;       };
  const bool&   setCached      (const bool&  x)  {cache->cached     =x; return cache->cached;      };
  const int&    setSymbol      (const int&   x)  {cache->symbol     =x; return cache->symbol;      };
  const TReal&  setSumGval     (const TReal& x)  {cache->sumGval    =x; return cache->sumGval;     };
  const TReal&  setMaxGval_exp (const TReal& x)  {cache->maxGval_exp=x; return cache->maxGval_exp; };
  ColumnVector& setPi          (void)            {return cache->Pi;};

  TNGnetCacheBase& operator()(int cindex)  {iec(cindex); return *this;};

  void print (std::ostream &os) const {cache->print (os);};
  void reset(int ucount) {cache1.reset(ucount); cache2.reset(ucount); cache=&cache1; oldcache=&cache2;};
};
//-------------------------------------------------------------------------------------------

/*! \page TNGnetCache How to use TNGnetCache
\code
  TNGnet ngnet;
  TNGnetCache <TNGnetModelDyn> ngnet_cache (ngnet.size());
  ...
  ngnet_cache.setCurrentState(data, cindex)  // cindex : discrete time, etc.
    // assign the current (x, u)=(state, control input) into data (or use setU, if u is not available)
  ..
  ngnet_cache.setData(cindex).setU(u);
  ..
  pix = ngnet.Pi_x(i, ngnet_cache(cindex))  // if cindex is different from the value specified by setCurrentState, an error arises
  ..
\endcode
  \warning Using ngnet.Pi_x(i, ngnet_cache) is dangerous because the index error is not checked.
          Use ngnet.Pi_x(i, ngnet_cache(cindex)).
*/
template <typename _type_model>  // _type_model must be a subclass of TNGnetModelBase
class TNGnetCache : public TNGnetCacheBase
{
public:
  typedef _type_model TModel;
  typedef TNGnetCacheBase TBase;
private:
  TModel data1, data2;
public:
  TNGnetCache(void) : TNGnetCacheBase() {cache1.data=&data1; cache2.data=&data2;};
  TModel& setData(int cindex) {iec(cindex); return *(static_cast<TModel*>(cache->data));}
  void setCurrentState (const TModel &_data, int cindex, bool enforce=false)
    {
      TNGnetCacheBase::setCurrentState(_data,cindex,enforce);
      if(cache==&cache1) data1=_data;
      else if(cache==&cache2) data2=_data;
    };
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TNGnet
//===========================================================================================
{
public:
  typedef TGaussianUnit                   unit_type;
  typedef std::list<TGaussianUnit>        uset_type;
  typedef TNGnetDataSetBase               dataset_type;
  typedef uset_type::iterator             iterator;
  typedef uset_type::const_iterator       const_iterator;
private:
  const TNGnet& operator= (const TNGnet &ngnet); // prevent to copy

protected:
  const TNGnetConfiguration *cnf;  //! configuration
  uset_type  uset;           //!< unit set

  dataset_type *data;      //!< pointer to learning data
  Matrix Pit1, Pit2;    //!< posterior P(i|x(t),y(t),theta) matrix, used in EM
  Matrix *currentPit;   //!< pointer to current Pit, used in EM
  Matrix cachedPti;     //!< P(x(t),y(t),i|theta) matrix used for calculate Q

  std::vector<iterator> funit;  //!< fast unit(i) to iterator mapping, used in pEM
  ColumnVector poldPt;          //!< , used in pEM
  TReal cachedQ;                //!< , used in pEM
  std::list<int> partial;       //!< , used in pEM

  mutable Matrix mu_trans;  //!< reduce the dimension of mu to 2


  void updateCache (TNGnetCacheBase &cache) const;
  // void cacheIEC (int cindex) const {cache->iec(cindex);};
  void cacheIECU (TNGnetCacheBase &cache) const
    {
      if (!cache.cached())  updateCache(cache);
    };

public:
  TNGnet (const TNGnetConfiguration &__cnf) : cnf(&__cnf), uset(), data(NULL), currentPit(NULL) {}; //cache_index(XDIM,-1.0)  {};
  TNGnet (void) : cnf(NULL), uset(), data(NULL), currentPit(NULL) {}; //cache_index(XDIM,-1.0)  {};

  void setConfig (const TNGnetConfiguration &__cnf)  {cnf=&__cnf;};

  const TNGnetConfiguration& getCnf(void) const {return *cnf;};
  const int&                 XDIM(void)   const {return cnf->XDIM();};
  const int&                 UDIM(void)   const {return cnf->UDIM();};
  const int&                 YDIM(void)   const {return cnf->YDIM();};
  const ColumnVector         getZeroX(void) const {return ColumnVector(cnf->XDIM(),0.0);};
  const ColumnVector         getZeroU(void) const {return ColumnVector(cnf->UDIM(),0.0);};
  const ColumnVector         getZeroY(void) const {return ColumnVector(cnf->YDIM(),0.0);};

  //! uset operation
  int                   size(void) const   { return uset.size(); };
  void                  clear (void)       { uset.clear(); };
  bool                  empty(void) const  { return uset.empty(); };
  unit_type&            unit(int i)        { return *list_itr_at(uset,i); };
  const unit_type&      unit(int i) const  { return *list_itr_at(uset,i); };
  unit_type&            back(void)         { return uset.back(); };
  const unit_type&      back(void) const   { return uset.back(); };
  unit_type&            front(void)        { return uset.front(); };
  const unit_type&      front(void) const  { return uset.front(); };
  iterator              iunit(int i)       { return list_itr_at(uset,i); };
  const_iterator        iunit(int i) const { return list_itr_at(uset,i); };
  iterator              begin(void)        { return uset.begin(); };
  const_iterator        begin(void) const  { return uset.begin(); };
  iterator              end(void)          { return uset.end(); };
  const_iterator        end(void) const    { return uset.end(); };
  iterator              erase( iterator itr )  { return uset.erase(itr); };
  void                  add(const unit_type &u)  { uset.push_back(u); };
  void                  splice(TNGnet &ngnet)  { uset.splice(uset.end(),ngnet.uset); };  //!< move ngnet.uset to *this.uset

  void                  setData (dataset_type *_data)  { data=_data; };
  dataset_type&            getData (void) { return *data; };
  const dataset_type&      getData (void) const { return *data; };

  //! basic calculation
  TReal           Pi_x (int i, const ColumnVector &x) const;  //!< return P(i|x)
  ColumnVector    Pi_x (const ColumnVector &x) const;  //!< return {P(i|x) | i=0..M-1}
  TReal           Pi_x (int i, TNGnetCacheBase &cache) const  //!< return P(i|x); using cache
                    {cacheIECU (cache); return  cache.Pi()(i);};
  const ColumnVector&  Pi_x (TNGnetCacheBase &cache) const  //!< return {P(i|x) | i=0..M-1}; using cache
                    {cacheIECU (cache); return cache.Pi();};
  TReal           Pi (int i, const ColumnVector &x) const  //! return P(i|*)
                    {return Pi_x(i,x);};
  ColumnVector    Pi (const ColumnVector &x) const  //! return {P(i|*) | i=0..M-1}
                    {return Pi_x(x);};
  TReal           Pi (int i, TNGnetCacheBase &cache) const  //! return P(i|*); using cache
                    {return Pi_x(i,cache);};
  const ColumnVector& Pi (TNGnetCacheBase &cache) const  //! return {P(i|*) | i=0..M-1}; using cache
                    {return Pi_x(cache);};

  int             symbol (const ColumnVector &x) const;  //!< get symbol index at state x
  int             symbol (TNGnetCacheBase &cache) const  //!< get symbol index at state x; using cache
                    {cacheIECU (cache); return cache.symbol();};
  ColumnVector    out (const ColumnVector &xt) const;  //!< calculate output y for state x and input u
  ColumnVector    out (TNGnetCacheBase &cache) const;  //!< calculate output y for state x and input u; using cache
  unit_type       genUnit (const ColumnVector &x) const;  //!< calculate generalized param of gaussian unit at state x
  unit_type       genUnit (TNGnetCacheBase &cache) const;  //!< calculate generalized param of gaussian unit at state x; using cache
  //! calculate generalized value of set at state x, non-cache version (see tngnet_implement.h, for detail):
    GENT_API_ITR  const T& genT (const ColumnVector &x, InItr first, InItr last, T &out) const;
    GENT_API_CNT  T genT (const ColumnVector &x, const Container &set) const;
    GENT_API_ITR  T genT (const ColumnVector &x, InItr first, InItr last) const;
    GENT_API_CNT  const T& genT (const ColumnVector &x, const Container &set, T &out) const;
  //! calculate generalized value of set at state x, using-cache version (see tngnet_implement.h, for detail):
    GENT_API_ITR  const T& genT (InItr first, InItr last, T &out, TNGnetCacheBase &cache) const;
    GENT_API_CNT  T genT (const Container &set, TNGnetCacheBase &cache) const;
    GENT_API_ITR  T genT (InItr first, InItr last, TNGnetCacheBase &cache) const;
    GENT_API_CNT  const T& genT (Container &set, T &out, TNGnetCacheBase &cache) const;
  TReal           logLikelihood (void) const;
  TReal           Px (const ColumnVector &x) const;  //!< calculate P(x|theta)
  TReal           Py_xt (const ColumnVector &y, const ColumnVector &xt) const;
    //!< calculate P(y|x,u,theta)
  TReal           Py_xt (const ColumnVector &y, TNGnetCacheBase &cache) const;
    //!< calculate P(y|x,u,theta); using cache
  double&         posterior (int i,int t) { return (*currentPit)(i,t); };
  const double&   posterior (int i,int t) const { return (*currentPit)(i,t); };
  Matrix&         posterior (void)  { return *currentPit; };
  const Matrix&   posterior (void) const { return *currentPit; };
  bool erase_small_posterior (void);
    //! erase units of small posterior; \note posterior will no longer be valid after this routine (run initEM)
  bool            divide_unit (void);
    //! divide units of large sig2; \note posterior will no longer be valid after this routine (run initEM)

  //! KL-divergence and distance between units
  //! \note check the validity!!
  TReal KLdiv (const unit_type &ui, const unit_type &uj) const // return KL divergence KL(ui|uj)
    {
      TReal res(0.0);
      const int M(uset.size());
      ColumnVector bi(ui.Wt().column(cnf->XTDIM()-1)), bj(uj.Wt().column(cnf->XTDIM()-1));
      Matrix Wi(ui.Wt().extract_n(0,0,cnf->YDIM(),cnf->XTDIM()-1)), Wj(uj.Wt().extract_n(0,0,cnf->YDIM(),cnf->XTDIM()-1));
      // res = real_log( uj.getsig2()/ui.getsig2() * ui.getdetInvSigma()/uj.getdetInvSigma() );
      res = real_log(uj.sig2()/ui.sig2()) + ui.logDetInvSigma() - uj.logDetInvSigma();
      res -= static_cast<TReal>(cnf->XDIM()+cnf->YDIM());
      res += Trace(uj.invSigma() * ui.Sigma());
      res += (uj.mu()-ui.mu()).transpose() * uj.invSigma() * (uj.mu()-ui.mu());
      res += static_cast<TReal>(cnf->YDIM()) * ui.sig2() / uj.sig2();
      res += 1.0/uj.sig2() * Trace((Wj-Wi).transpose()*(Wj-Wi)*ui.Sigma());
      res += 1.0/uj.sig2() * GetNorm((Wj-Wi)*ui.mu() + (bj-bi));
      res /= static_cast<TReal>(2*M);
      return res;
    };
  TReal KLdiv(int i, int j) const //! return KL divergence KL(i|j)
    { return KLdiv(unit(i),unit(j)); };
  TReal distanceKLdiv( const unit_type &ui, const unit_type &uj ) const //! return distance between ui and uj
    { return 0.5 * (KLdiv(ui,uj)+KLdiv(uj,ui)); }; // distanceKLdiv(ui,uj)=distanceKLdiv(uj,ui) is assured
  TReal distanceKLdiv(int i, int j) const //! return distance between unit i and j
    { return distanceKLdiv(unit(i),unit(j)); };
  TReal distanceEuclid( const unit_type &ui, const unit_type &uj ) const //! return distance between ui and uj
    { return GetNorm(ui.mu() - uj.mu()); };

  //! full-EM
  void  Estep (Matrix &Pit);
  void  singleMstep (const Matrix &Pit, const iterator &itr, int i);
  void  Mstep (const Matrix &Pit);
  void  initEM (void);
  TReal updateParamsByEM(void);
  //! partial-EM
  void  pEstep (Matrix &Pit);
  void  pMstep (const Matrix &Pit);
  void  initpEM (std::list<int> &partialOld, std::list<int> &partialNew);
  TReal updateParamsBypEM(void);
  // SMEM
  // TReal  execSplitAndMerge (void)  // execute split and merge EM; return Q after SMEM
  //   {
  //     int split, merge[2];
  //     split=...
  //     merge=...
  //     initpEM({split,merge},{});
  //     partialNGnetEM (*this, {split,merge}, {}, recision, mvf_size, const string &index, MAX_PEM_ITERATIONS);
  //     Q=fullNGnetEM (...);
  //     return Q;
  //   };

  //! utility
  void SaveToStringList (TStringListEx &str_list, const std::string &prefix) const;
  void LoadFromStringList (TStringListEx &str_list);
  bool SaveToFile (const std::string &filename) const;
  bool LoadFromFile (const std::string &filename);
  void print_info (std::ostream &os) const;
  void print_unit_area (std::ostream &os, bool update_trans_matrix=true) const;
  void print_unit_label_gp (std::ostream &os, bool update_trans_matrix=true) const;
  void calc_mu_trans (void) const;
  void set_mu_trans (const Matrix &mat) const { mu_trans = mat; };
  const Matrix& get_mu_trans (void) const { return mu_trans; };
};
//-------------------------------------------------------------------------------------------

extern "C" void fullNGnetEM (TNGnet &ngnet, TNGnet::dataset_type &data, const TReal &precision, int mvf_size,
    const std::string &index, const int &MAX_EM_ITERATIONS=1000, bool (*call_back)(TNGnet &,const TReal &Q)=NULL);


//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------


GENT_API_ITR  const T& TNGnet::genT (const ColumnVector &x, InItr first, InItr last, T &out) const
  //! calculate generalized param of set at state x
  //! \param T  must define WeightedAdd(TReal w, T x) member function.
  //! \param InItr must be an iterator of list &lt; T &gt; or vector &lt; T &gt;
  //! \param [in,out] out  must be initialized (as zero)
{
  if(size() == 0)  return out;
  //if(static_cast<size_t>(size()) < set.size())
  //  { std::cerr<<"faital in TNGnet::genT.  invalid set.size()"<<std::endl; lexit(df); }
  TReal sumGval(0.0), maxGval_exp(-REAL_MAX);  // weight
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
  {
    itr->tmpGval = itr->lnG(x);
    if(itr->tmpGval > maxGval_exp) maxGval_exp = itr->tmpGval;
  }
  for( const_iterator itr(uset.begin()); itr!=uset.end(); ++itr )
    sumGval += real_exp(itr->tmpGval-maxGval_exp);
  TReal tmpdiv (maxGval_exp + real_log(sumGval));
  const_iterator itr (uset.begin());
  for(; itr!=uset.end()&&first!=last; ++itr, ++first )
    WeightedAdd (out, real_exp(itr->tmpGval-tmpdiv), *first);
  if (itr!=uset.end()||first!=last)
    {std::cerr<<"faital in TNGnet::genT. invalid first or last"<<std::endl; lexit(df);}
  return out;
}
//-------------------------------------------------------------------------------------------
GENT_API_CNT  T TNGnet::genT (const ColumnVector &x, const Container &set) const
  //! calculate generalized param of set at state x
  //! \param T must define constructor (set zero) and WeightedAdd(TReal w, T x) member function.
{
  T  out;
  genT (x, set.begin(), set.end(), out);
  return out;
}
//-------------------------------------------------------------------------------------------
GENT_API_ITR  T TNGnet::genT (const ColumnVector &x, InItr first, InItr last) const
  //! calculate generalized param of set at state x
  //! \param T must define constructor (set zero) and WeightedAdd(TReal w, T x) member function.
{
  T  out;
  genT (x, first, last, out);
  return out;
}
//-------------------------------------------------------------------------------------------
GENT_API_CNT  const T& TNGnet::genT (const ColumnVector &x, const Container &set, T &out) const
  //! calculate generalized param of set at state x
  //! \param T  must define WeightedAdd(TReal w, T x) member function.
  //! \param [in,out] out  must be initialized (as zero)
{
  return genT (x, set.begin(), set.end(), out);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// using cache
//-------------------------------------------------------------------------------------------

GENT_API_ITR  const T& TNGnet::genT (InItr first, InItr last, T &out, TNGnetCacheBase &cache) const
  //! calculate generalized param of set at state x; using cache
  //! \param T  must define WeightedAdd(TReal w, T x) member function.
  //! \param [in,out] out  must be initialized (as zero)
{
  if(size() == 0)  return out;
  // if(static_cast<size_t>(size()) < set.size())
  //   { std::cerr<<"faital in TNGnet::genT.  invalid set.size()"<<std::endl; lexit(df); }
  cacheIECU (cache);
  int i(0);
  const_iterator itr (uset.begin());
  for(; itr!=uset.end()&&first!=last; ++itr, ++first,++i )
    WeightedAdd (out, cache.Pi()(i), *first);
  if (itr!=uset.end()||first!=last)
    {std::cerr<<"faital in TNGnet::genT. invalid first or last"<<std::endl; lexit(df);}
  return out;
}
//-------------------------------------------------------------------------------------------
GENT_API_CNT  T TNGnet::genT (const Container &set, TNGnetCacheBase &cache) const
  //! calculate generalized param of set at state x; using cache
  //! \param T must define constructor (set zero) and WeightedAdd(TReal w, T x) member function.
{
  T  out;
  genT (set.begin(), set.end(), out, cache);
  return out;
}
//-------------------------------------------------------------------------------------------
GENT_API_ITR  T TNGnet::genT (InItr first, InItr last, TNGnetCacheBase &cache) const
  //! calculate generalized param of set at state x; using cache
  //! \param T must define constructor (set zero) and WeightedAdd(TReal w, T x) member function.
{
  T  out;
  genT (first, last, out, cache);
  return out;
}
//-------------------------------------------------------------------------------------------
GENT_API_CNT  const T& TNGnet::genT (Container &set, T &out, TNGnetCacheBase &cache) const
  //! calculate generalized param of set at state x; using cache
  //! \param T  must define WeightedAdd(TReal w, T x) member function.
  //! \param [in,out] out  must be initialized (as zero)
{
  return genT (set.begin(), set.end(), out, cache);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------


inline void calcRMSE (const TNGnet &ngnet, TError &er)
  /*! calculate root mean square error of ngnet, for its data
      \param [in] ngnet ngnet
      \param [out] er.rmse    root mean square error
      \param [out] er.avr     average of ||error(t)||
      \param [out] er.max     max of ||error(t)||
      \param [out] er.min     min of ||error(t)|| */
{
   const TNGnet::dataset_type  &data(ngnet.getData());
   er.SetZero();
   // { rmse=0.0l; avr=0.0l; max=0.0l; min=0.0l; };
   er.min = REAL_MAX;
   std::vector<TReal> et(data.size(), 0.0l);
   std::vector<TReal>::iterator eitr(et.begin());
   for (data.goFirst();!data.isEnd();++eitr,data.increment())
   {
     *eitr = SquareSum (data.current().y()-ngnet.out(data.current().xt()));
     er.rmse += *eitr;
     *eitr = real_sqrt(*eitr);
     er.avr += *eitr;
     er.max =  std::max(er.max,*eitr);
     er.min =  std::min(er.min,*eitr);
   }
   er.rmse = real_sqrt(er.rmse / static_cast<TReal>(et.size()));
   er.avr  = er.avr / static_cast<TReal>(et.size());
   // for (eitr=et.begin();eitr!=et.end();++eitr)
   //   er.std_dev += Square(*eitr-er.avr);
   // er.std_dev = real_sqrt(er.std_dev / static_cast<TReal>(et.size())-1);
   return;
}
//-------------------------------------------------------------------------------------------
inline void calcRMSE (const TNGnet &ngnet, TError &er, const std::vector<double> &weight)
  /*! calculate root mean square error of ngnet, for its data
      \param [in] ngnet ngnet
      \param [out] er.rmse    root mean square error
      \param [out] er.avr     average of ||error(t)||
      \param [out] er.max     max of ||error(t)||
      \param [out] er.min     min of ||error(t)|| */
{
   const TNGnet::dataset_type  &data(ngnet.getData());
   er.SetZero();
   // { rmse=0.0l; avr=0.0l; max=0.0l; min=0.0l; };
   er.min = REAL_MAX;
   std::vector<TReal> et(data.size(), 0.0l);
   std::vector<TReal>::iterator eitr (et.begin());
   std::vector<double>::const_iterator witr (weight.begin());
   double sumweight(0.0l);
   for (data.goFirst();!data.isEnd();++eitr,++witr,data.increment())
   {
     *eitr = *witr * SquareSum (data.current().y()-ngnet.out(data.current().xt()));
     er.rmse += *eitr;
     *eitr = real_sqrt(*eitr);
     er.avr += *eitr;
     er.max =  std::max(er.max,*eitr);
     er.min =  std::min(er.min,*eitr);
     sumweight += *witr;
   }
   er.rmse = real_sqrt(er.rmse / sumweight);
   er.avr  = er.avr / sumweight;
   return;
}
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_oldngnet_h
//-------------------------------------------------------------------------------------------
