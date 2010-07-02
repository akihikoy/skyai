//-------------------------------------------------------------------------------------------
/*! \file   small_classes.h
    \brief   liblora - class utility  (header)
    \author Akihiko Yamaguchi
    \date   2008
    \date   2009
    \date   2010

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
#ifndef loco_rabbits_small_classes_h
#define loco_rabbits_small_classes_h
//-------------------------------------------------------------------------------------------
#include <lora/math.h>
#include <lora/stl_ext.h>
#include <vector>
#include <list>
#include <map>
#include <iostream>
#include <cstring>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \brief TStatisticsFilter calculates total, mean, variance, std-deviation,
            max, min, etc. for a sequential data */
template <typename t_real>
struct TStatisticsFilter
//===========================================================================================
{
  t_real  Total;
  t_real  NumberOfSamples;
  t_real  Mean;
  t_real  SqMean;
  t_real  Maximum;
  t_real  Minimum;

  t_real  Variance() const {t_real v(SqMean - Mean*Mean); return (v>=0.0l)?v:0.0l;}
  t_real  StdDeviation() const {return std::sqrt(Variance());}

  TStatisticsFilter (void)
    {
      Clear();
    }
  void Clear ()
    {
      Total           = 0.0l;
      NumberOfSamples = 0;
      Mean            = 0.0l;
      SqMean          = 0.0l;
      Maximum         = -RealMax<t_real>();
      Minimum         =  RealMax<t_real>();
    }

  void Add (const t_real &x)
    {
      Total+= x;
      ++NumberOfSamples;
      t_real N(static_cast<t_real>(NumberOfSamples));
      Mean= x/N + Mean*(N-1.0l)/N;
      SqMean= x*x/N + SqMean*(N-1.0l)/N;
      Maximum= std::max(Maximum,x);
      Minimum= std::min(Minimum,x);
    }
};
//-------------------------------------------------------------------------------------------

template <typename t_real>
std::ostream& operator<< (std::ostream &os, const TStatisticsFilter<t_real> &rhs)
{
#define TSTATISTICSFILTER_PRINT(elem)  os<<"  "#elem"  :  "<<rhs.elem<<std::endl;
  TSTATISTICSFILTER_PRINT(Total           )
  TSTATISTICSFILTER_PRINT(NumberOfSamples )
  TSTATISTICSFILTER_PRINT(Mean            )
  TSTATISTICSFILTER_PRINT(StdDeviation()  )
  TSTATISTICSFILTER_PRINT(Variance()      )
  TSTATISTICSFILTER_PRINT(Maximum         )
  TSTATISTICSFILTER_PRINT(Minimum         )
#undef TSTATISTICSFILTER_PRINT
  return os;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
template<class T>
class TMovingAverageFilter
//===========================================================================================
{
private:
  std::vector<T>  buffer;
  int index;
  T   current;
  T   clear_val;
  bool use_clear_val;
public:
  TMovingAverageFilter()  : index(0), use_clear_val(false) {}
  TMovingAverageFilter(int n)  : index(0), use_clear_val(false) { Initialize(n); }
  TMovingAverageFilter(int n, const T &clv) : index(0), clear_val(&clv), use_clear_val(true) { Initialize(n); }
  void SetClearVal (const T &clv)  { clear_val=clv; use_clear_val=true; }
  void Initialize (int n)
    {
      buffer.resize(n);
      index = 0;
      if (!use_clear_val)
      {
        for(typename std::vector<T>::iterator itr(buffer.begin());itr!=buffer.end();++itr)
          SetZero(*itr);
        SetZero(current);
      }
      else
      {
        for(typename std::vector<T>::iterator itr(buffer.begin());itr!=buffer.end();++itr)
          *itr = clear_val;
        current = clear_val;
      }
    }
  void Initialize (int n, const T &clv)  { SetClearVal(clv); Initialize(n); }
  void Initialize (int n, const T &clv, const T &init_val)
    {
      Initialize (n, init_val);
      SetClearVal (clv);
    }
  T Step (const T &val)
    {
      buffer[index] = val;
      index = (index+1<static_cast<int>(buffer.size())) ? (index+1) : 0;
      T res; if(!use_clear_val) SetZero(res); else res=clear_val;
      for(typename std::vector<T>::iterator itr(buffer.begin());itr!=buffer.end();++itr)
        res += *itr;
      return (current = res / static_cast<TReal>(buffer.size()));
    }
  T Value (void) const
    {
      // T res; if(!use_clear_val) SetZero(res); else res=clear_val;
      // for(typename std::vector<T>::iterator itr(buffer.begin());itr!=buffer.end();++itr)
      //   res += *itr;
      // return res / static_cast<T>(buffer.size());
      return current;
    }
  T operator()(const T &val)  { return Step(val); }
  T operator()(void) const { return Value(); }
  T LastLowData (void) const { return buffer[(index==0)?(buffer.size()-1):(index-1)]; }
  T PrevLowData (int n) const  //! return previous low data n < 0.  \note PrevLowData(0)==LastLowData()
      { while (index-1+n < 0) n+=buffer.size(); return buffer[index-1+n]; }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*! \brief low, high, and band path filters
    a part of source code is based on
    http://www.d4.dion.ne.jp/~sh_okada/tec/digital_filter.html  */
template<class T>
class TLHBPFilters
//===========================================================================================
{
public:
  enum TMode {LPF1,HPF1,LPF2,HPF2,BPF2};
private:
  TMode  mode;
  TReal  a, rq;
  T val1, val2;
  T   current;
  T   clear_val;
  bool use_clear_val;
  bool initialized;
public:
  TLHBPFilters(void)  : use_clear_val(false), initialized(false) {}
  TLHBPFilters(const T &clv) : clear_val(&clv), use_clear_val(true), initialized(false) {}
  void SetClearVal (const T &clv)  { clear_val=clv; use_clear_val=true; }
  bool isInitialized (void) const {return initialized;}
  void Initialize (const TMode &m, const TReal &dt, const TReal &f, const TReal &q)
    {
      mode = m;
      a  = 1.0l - real_exp(-2.0l*REAL_PI*f*dt);
      rq = 1.0l / q;
      if (!use_clear_val)
      {
        SetZero(val1);
        SetZero(val2);
        SetZero(current);
      }
      else
      {
        val1 = clear_val;
        val2  = clear_val;
        current = clear_val;
      }
      initialized= true;
    }
  void Initialize (const TMode &m, const TReal &dt, const TReal &f, const TReal &q, const T &clv)
    {
      SetClearVal(clv);
      Initialize(m,dt,f,q);
    }
  void Initialize (const TMode &m, const TReal &dt, const TReal &f, const TReal &q, const T &clv, const T &init_val)
    {
      // Initialize (m,dt,f,q, init_val);
      // SetClearVal (clv);
      Initialize (m,dt,f,q, clv);
      for(int i(0), iend(static_cast<int>(1.0l/f/dt*2.0l)); i<iend; ++i)  Step(init_val);
    }
  const T& Step (const T &val)
    {
      if (mode==LPF1 || mode==HPF1)
      {
        current = val - val1;
        val1 += current * a;
        if (mode==LPF1) current = val1;
        else if (mode==HPF1) current = val - val1;
      }
      else
      {
        current = val - val2 - val1 * rq;
        val1 += current * a;
        val2 += val1 * a;
        if (mode==LPF2) current = val2;
        else if (mode==HPF2) current = val - val2 - val1 * rq;
        else if (mode==BPF2) current = val1 * rq;
      }
      return current;
    }
  const T& Value (void) const
    {
      return current;
    }
  const T& operator()(const T &val)  { return Step(val); }
  const T& operator()(void) const { return Value(); }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//! "double out" class.  use this object to output the same value to two stream
class dout
//===========================================================================================
{
private:
  std::ostream &os1, &os2;
public:
  explicit dout (std::ostream &v_os1, std::ostream &v_os2) : os1(v_os1), os2(v_os2) {}
  template <typename T>
  dout& operator<< (const T     &rhs)  { os1 << rhs;  os2 << rhs; return *this; }
  dout& operator<< (std::ostream& (*pf)(std::ostream&))  { pf(os1); pf(os2); return *this; }
    /*!<  Interface for manipulators, such as \c std::endl and \c std::setw
      For more information, see ostream header */
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
//!\brief simple option parser class
class TOptionParser
//===========================================================================================
{
private:
  struct t_cell
    {
      std::string val;
      bool accessed;
      t_cell (void) : val(""), accessed(false) {}
      t_cell (const std::string &s) : val(s), accessed(false) {}
    };
  std::map <std::string, t_cell> opt;  //!< options whose format is '-optname optvalue'
  std::list <std::string>  used_opt;  //!< accessed options
  void parse_from_arglist (int argc, const char *const*const argv); //!< parse from arguments

public:
  std::list <std::string>  Floating;  /*!< options whose format is 'optname' (i.e. singular)
                                          if using such options and PrintNotAccessed, clear Floating
                                          before executing PrintNotAccessed */
  TOptionParser (int argc, const char *const*const argv)  {parse_from_arglist(argc-1,argv+1);}

  /*!\brief parse from a string
    \note option are separated by white spaces;
          strings like "x x" or 'y y' are recognized as a single word
    \note backslash \\ escapes a following letter */
  TOptionParser (const std::string &argline);

  //!\brief return the value of an option whose key is equal to the given key
  const std::string& operator[] (const std::string& key)
    {
      if(std::find(used_opt.begin(),used_opt.end(),key)==used_opt.end())
        used_opt.push_back(key);
      opt[key].accessed=true;
      return opt[key].val;
    }

  //!\brief return the value of an option whose key is equal to the given key.  if the key does not exist, return sdefault
  const std::string& operator() (const std::string& key, const std::string& sdefault=std::string(""))
    {
      if(std::find(used_opt.begin(),used_opt.end(),key)==used_opt.end())
        used_opt.push_back(key);
      if (opt.find(key)==opt.end()) return sdefault;
      opt[key].accessed=true;
      return opt[key].val;
    }

  //! \brief print unaccessed options, \return if there is no unaccessed options, return false, else true
  bool PrintNotAccessed (std::ostream &os=std::cerr, const std::string &prefix=std::string("  "), bool including_floating=true);

  void PrintUsed (std::ostream &os=std::cerr, const std::string &prefix=std::string("  "));
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief initializer: use this class to execute several routines just after every event.

  example:
  \code
  TInitializer Init;

  void funcA (void)
  {
    static int icode(-1);
    if (Init(icode))
    {
      INIT_CODE_A
    }
    ...
  }

  void funcB (void)
  {
    static int icode(-1);
    if (Init(icode))
    {
      INIT_CODE_B
    }
    ...
  }

  in a routine
  {
    Init.Init();
    funcA();  // INIT_CODE_A is executed
    funcB();  // INIT_CODE_B is executed
    ...
    Init.Init();
    funcB();  // INIT_CODE_B is executed
    funcA();  // INIT_CODE_A is executed
  }
  \endcode
*/
class TInitializer
//===========================================================================================
{
private:
  std::vector<bool>  table;
  void set_all (bool a)
    {
      for (std::vector<bool>::iterator itr(table.begin()); itr!=table.end(); ++itr)
        *itr = a;
    }
public:
  TInitializer(void)  {}
  void Clear (void)  { table.clear(); }
  void Init (void)  { set_all(true); }
  void NotInit (void)  { set_all(false); }

  //! \note user_code must be initialized by -1 at first time
  bool operator() (int &user_code, bool unset_init=true)
    {
      bool res;
      if (user_code<0)
      {
        table.push_back (!unset_init);
        user_code = table.size()-1;
        res = true;  // note: at first time, this function must return true
      }
      else
      {
        res = table[user_code];
        table[user_code] = !unset_init;
      }
      return res;
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \class TNBase
    \brief class to treat N-base number

  usage:
  \code
    TNBase nbase(10,3);
    int dmax(nbase.DecimalMax());
    for (int i(0); i<=dmax; ++i,++nbase)
      cout<<(int)nbase<<"\t"<<nbase<<endl;
    cout<<"---"<<endl;
    cout<<(int)nbase<<"\t"<<nbase<<endl;
    nbase= 128;
    cout<<(int)nbase<<"\t"<<nbase<<endl;
  \endcode
*/
class TNBase
//===========================================================================================
{
public:
  typedef  int                      ValueType;
  typedef  std::vector<ValueType>   NBaseType;
private:
  ValueType   N_, digits_;
  NBaseType   nbase_;
  ValueType   decimal_, decimal_max_;
  bool        err_;
  void assign (ValueType val);
  void increment (void)
    {
      ++decimal_;  if(decimal_>decimal_max_)  {decimal_= decimal_max_; err_=true;}
      for (NBaseType::iterator  idigit (nbase_.begin()); idigit!=nbase_.end(); ++idigit)
      {
        ++(*idigit);
        if (*idigit<N_)  return;
        *idigit= 0;
      }
    }
public:
  TNBase (void) :
      N_           (0),
      digits_      (0),
      nbase_       (0),
      decimal_     (0),
      decimal_max_ (0),
      err_         (true)
    {}
  TNBase (const ValueType &n, const ValueType &Ndigits) :
      N_           (0),
      digits_      (0),
      nbase_       (0),
      decimal_     (0),
      decimal_max_ (0),
      err_         (true)
    {
      Init(n, Ndigits);
    }
  void Init (const ValueType &n, const ValueType &Ndigits)
    {
      N_= n;
      digits_= Ndigits;
      err_= false;
      nbase_.resize(digits_);
      decimal_max_= ipow(N_,Ndigits)-1;
      assign(0);
    }

  const ValueType& N (void) const {return N_;}
  const ValueType& Digits (void) const {return digits_;}
  const ValueType& DecimalMax (void) const {return decimal_max_;}

  const TNBase& operator= (const ValueType& rhs)  {assign(rhs); return *this;}
  const TNBase& operator++ (void) {increment(); return *this;}
  const ValueType& operator[] (const ValueType &i) const {return nbase_[i];}
  operator ValueType() const {return decimal_;}
};
std::ostream& operator<< (std::ostream &lhs, const TNBase &rhs);
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*! \class TNxBase
    \brief class to treat Nx-base number where {N} of each dimension are different
  usage:
    \code
      const int A[] = {10,5,3};
      for (TNxBase nxbase(A,A+SIZE_OF_ARRAY(A)); nxbase.Cont(); ++nxbase)
        cout<<(int)nxbase<<"\t"<<nxbase<<endl;
      nxbase= 128;
      cout<<(int)nxbase<<"\t"<<nxbase<<endl;
    \endcode
*/
class TNxBase
//===========================================================================================
{
public:
  typedef int                               ValueType;
  typedef ValueType&                        Reference;
  typedef const ValueType&                  ConstReference;
  typedef std::vector<ValueType>            SetType;
  typedef SetType::iterator                 Iterator;
  typedef SetType::const_iterator           ConstIterator;
  typedef SetType::reverse_iterator         ReverseIterator;
  typedef SetType::const_reverse_iterator   ConstReverseIterator;
private:
  ValueType  decimal_, decimal_overflow_;
  SetType    d, size;
public:

  TNxBase (void) : decimal_(0), decimal_overflow_(0) {}

  template <typename InputIterator>
  TNxBase (InputIterator sizefirst, InputIterator sizelast)
      : decimal_(0), decimal_overflow_(0)
    {
      Init<InputIterator>(sizefirst,sizelast);
    }

  template <typename InputIterator>
  void Init (InputIterator sizefirst, InputIterator sizelast)
    {
      size.clear();
      decimal_overflow_= 1;
      for(InputIterator i(sizefirst); i!=sizelast; ++i)
      {
        size.push_back(*i);
        decimal_overflow_*=*i;
      }
      d.resize (size.size());
      Init();
    }
  void Init (void)
    {decimal_=0; std::fill(Begin(),End(),0);}

  const SetType&          Nx (void) const {return size;}
  ValueType               Digits (void) const {return size.size();}
  ValueType               DecimalOverflow (void) const {return decimal_overflow_;}

  Iterator                Begin(void)       {return d.begin();}
  ConstIterator           Begin(void) const {return d.begin();}
  ReverseIterator         RBegin(void)       {return d.rbegin();}
  ConstReverseIterator    RBegin(void) const {return d.rbegin();}
  Iterator                End(void)       {return d.end();}
  ConstIterator           End(void) const {return d.end();}
  ReverseIterator         REnd(void)       {return d.rend();}
  ConstReverseIterator    REnd(void) const {return d.rend();}
  // Reference               operator[] (int i)       {return d[i];}
  ConstReference          operator[] (int i) const {return d[i];}

  Iterator                SizeBegin(void)       {return size.begin();}
  ConstIterator           SizeBegin(void) const {return size.begin();}
  Iterator                SizeEnd(void)       {return size.end();}
  ConstIterator           SizeEnd(void) const {return size.end();}

  bool Cont (void) const
    {
      if (decimal_overflow_==0)  return false;
      if (d.back()>=size.back())
        return false;
      else return true;
    }
  bool Increment (void)
    {
      ++decimal_; if(decimal_>=decimal_overflow_) {decimal_=decimal_overflow_;}
      ConstIterator sizeitr (size.begin());
      for(Iterator itr(Begin()); itr!=End(); ++sizeitr,++itr)
      {
        if (*itr+1<*sizeitr) {++(*itr); break;}
        if (itr==End()-1) {*itr=*sizeitr;}
        else *itr=0;
      }
      return Cont();
    }
  bool Step (void)  {return Increment();}

  bool Assign (ValueType val);

  const TNxBase& operator= (const ValueType& rhs)  {Assign(rhs); return *this;}
  const TNxBase& operator++ (void) {Increment(); return *this;}
  operator ValueType() const {return decimal_;}
};
std::ostream& operator<< (std::ostream &lhs, const TNxBase &rhs);
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*! \class TGridGenerator
  \brief generate a grid on a continuous vector space which conver an integer to continuous vector

  usage:
    \code
      int  levels[]= {3,3};
      double mins[]= { -1.0, -2.0};
      double maxs[]= {  1.0,  2.0};
      TGridGenerator grid(levels,levels+sizeof(levels)/sizeof(levels[0]),mins,maxs);
      vector<TReal> vec(grid.Digits(),0.0l);

      grid.IntToContVector(0,vec.begin());
      cout<<vec[0]<<", "<<vec[1]<<endl;     // = -1, -2
      grid.IntToContVector(4,vec.begin());
      cout<<vec[0]<<", "<<vec[1]<<endl;     // = 0, 0
      grid.IntToContVector(6,vec.begin());
      cout<<vec[0]<<", "<<vec[1]<<endl;     // = -1, 2
    \endcode
*/
class TGridGenerator
//===========================================================================================
{
public:

  TGridGenerator () {}

  template <typename t_levels_iterator, typename t_vector_iterator>
  TGridGenerator (t_levels_iterator levels_first, t_levels_iterator levels_last,
                  t_vector_iterator mins_first, t_vector_iterator maxs_first)
    {
      Init (levels_first, levels_last, mins_first, maxs_first);
    }

  /*!\brief initialize the class
      \param [in]levels_first : first iterator of `level vector' which denotes a vector of the number of partitions into which each dimension is divided.
      \param [in]levels_first : last iterator of `level vector'
      \param [in]mins_first   : first iterator of minimum vector
      \param [in]maxs_first   : first iterator of maximum vector */
  template <typename t_levels_iterator, typename t_vector_iterator>
  void Init (t_levels_iterator levels_first, t_levels_iterator levels_last,
              t_vector_iterator mins_first, t_vector_iterator maxs_first)
    {
      dim_mapper_.Init (levels_first, levels_last);
      units_.resize(dim_mapper_.Digits());
      mins_.resize(dim_mapper_.Digits());
      maxs_.resize(dim_mapper_.Digits());
      std::vector<TReal>::iterator units_itr(units_.begin()), mins_itr(mins_.begin()), maxs_itr(maxs_.begin());
      for (; levels_first!=levels_last; ++levels_first,
                                        ++mins_first,
                                        ++maxs_first,
                                        ++units_itr,++mins_itr,++maxs_itr)
      {
        *mins_itr= *mins_first;
        *maxs_itr= *maxs_first;
        if (*levels_first>1)
          *units_itr= (*maxs_first-*mins_first)/static_cast<TReal>(*levels_first-1);
        else
          *units_itr= (*maxs_first-*mins_first);
      }
    }

  int Size() const {return dim_mapper_.DecimalOverflow();}
  int Digits() const {return dim_mapper_.Digits();}

  /*!\brief convert an integer to corresponding continuous vector
      \param [in]val : input integer
      \param [out]res_first : first iterator of output vector whose size should be dim_mapper_.Digits() */
  template <typename t_vector_iterator>
  bool IntToContVector (int val, t_vector_iterator res_first) const
    {
      if(!dim_mapper_.Assign(val))
      {
        LERROR("TGridGenerator: cannot convert "<<val<<" to continuous vector. "
                "The input should be in [0,"<<Size()-1<<"].");
        return false;
      }
      CurrentContVector (res_first);
      return true;
    }

  /*!\brief get the current continuous vector (use with Init, Cont, Increment)
      \param [out]res_first : first iterator of output vector whose size should be dim_mapper_.Digits() */
  template <typename t_vector_iterator>
  void CurrentContVector (t_vector_iterator res_first) const
    {
      std::vector<TReal>::const_iterator units_itr(units_.begin()), mins_itr(mins_.begin()), maxs_itr(maxs_.begin());
      for(TNxBase::ConstIterator ditr(dim_mapper_.Begin()), sitr(dim_mapper_.SizeBegin());
            ditr!=dim_mapper_.End();
            ++ditr,++sitr,++units_itr,++mins_itr,++maxs_itr,++res_first)
      {
        if (*sitr<=1)
          *res_first=0.5l*(*mins_itr+*maxs_itr);
        else
          *res_first= (*mins_itr)+(*units_itr)*(*ditr);
      }
    }
  void Init (void) const  {dim_mapper_.Init();}
  bool Cont (void) const  {return dim_mapper_.Cont();}
  void Increment (void) const {++dim_mapper_;}

private:

  mutable TNxBase     dim_mapper_;
  std::vector<TReal>  units_, mins_, maxs_;       //!< {interval, min, max} vector of grid

};
//-------------------------------------------------------------------------------------------



//===========================================================================================
template <typename t_key, typename t_value>
class TListMap;
//===========================================================================================

//-------------------------------------------------------------------------------------------
namespace loco_rabbits_detail
{
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief iterator class of TListMap
  \note this class assumes that container_ (&TListMap::data_) does not include an empty list
*/
template<typename t_data, typename t_iterator1, typename t_iterator2, typename t_value_ref, typename t_value_ptr, typename t_key_ref>
class TListMapIterator
//===========================================================================================
{
private:
  typedef TListMapIterator<t_data, t_iterator1, t_iterator2, t_value_ref, t_value_ptr, t_key_ref>  TThis;

public:
  TListMapIterator () : container_(NULL) {}

  template<typename t2_iterator1, typename t2_iterator2, typename t2_value_ref, typename t2_value_ptr, typename t2_key_ref>
  TListMapIterator (const TListMapIterator<t_data,t2_iterator1,t2_iterator2,t2_value_ref,t2_value_ptr,t2_key_ref> &rhs)
      : container_(rhs.container_), iterator1_(rhs.iterator1_), iterator2_(rhs.iterator2_)
    {
    }

  TThis& operator++()
    {
      // if (iterator1_==container_->end())  {LERROR("invalid increment!"); lexit(df);}
      ++iterator2_;
      if (iterator2_==iterator1_->second.end())
      {
        ++iterator1_;
        if (iterator1_!=container_->end())
          iterator2_= iterator1_->second.begin();
      }
      return *this;
    }
  TThis& operator--()
    {
      // if (iterator1_==container_->end())  {LERROR("invalid increment!"); lexit(df);}
      if (iterator1_==container_->end())  // i.e. *this == end()
      {
        --iterator1_;
        iterator2_= iterator1_->second.end();
      }
      else if (iterator2_==iterator1_->second.begin())
      {
        bool is_1_begin(iterator1_==container_->begin());
        --iterator1_;
        if (!is_1_begin)
          iterator2_= iterator1_->second.end();
      }
      --iterator2_;
      return *this;
    }
  t_value_ref operator*()
    {
      return *iterator2_;
    }
  t_value_ptr operator->()
    {
      return &(*iterator2_);
    }
  bool operator==(const TThis &rhs)
    {
      if (iterator1_==container_->end() && rhs.iterator1_==container_->end())  return true;
      return iterator1_==rhs.iterator1_ && iterator2_==rhs.iterator2_;
    }
  bool operator!=(const TThis &rhs)
    {
      return !(operator==(rhs));
    }
  t_key_ref key()
    {
      return iterator1_->first;
    }

private:
  const t_data   *container_;
  t_iterator1    iterator1_;
  t_iterator2    iterator2_;
  friend class loco_rabbits::TListMap<typename t_data::key_type, typename t_data::mapped_type::value_type>;
  template<typename t2_data, typename t2_iterator1, typename t2_iterator2, typename t2_value_ref, typename t2_value_ptr, typename t2_key_ref>
  friend class TListMapIterator;

};  // TListMapIterator
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits_detail
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief map [key --\> list\<t_value\>] template class
    \warning This template class is highly experimental. Please test this class before to use.
*/
template <typename t_key, typename t_value>
class TListMap
//===========================================================================================
{
private:

  typedef std::map<t_key,std::list<t_value> >   TData;

public:

  typedef loco_rabbits_detail::TListMapIterator<
                                  TData,
                                  typename TData::iterator,
                                  typename std::list<t_value>::iterator,
                                  t_value&,
                                  t_value*,
                                  const t_key&>                        iterator;
  typedef loco_rabbits_detail::TListMapIterator<
                                  TData,
                                  typename TData::const_iterator,
                                  typename std::list<t_value>::const_iterator,
                                  const t_value&,
                                  t_value const*,
                                  const t_key&>                        const_iterator;

  TListMap() : total_elements_(0) {}

  iterator begin()
    {
      if(data_.empty())  return end();
      iterator i;
      i.container_= &data_;
      i.iterator1_= data_.begin();
      i.iterator2_= i.iterator1_->second.begin();
      return i;
    }
  const_iterator begin() const
    {
      if(data_.empty())  return end();
      const_iterator i;
      i.container_= &data_;
      i.iterator1_= data_.begin();
      i.iterator2_= i.iterator1_->second.begin();
      return i;
    }
  iterator end()
    {
      iterator i;
      i.container_= &data_;
      i.iterator1_= data_.end();
      return i;
    }
  const_iterator end() const
    {
      const_iterator i;
      i.container_= &data_;
      i.iterator1_= data_.end();
      return i;
    }
  //!\brief return an iterator where iterator's key is equal to key
  iterator begin(const t_key &key)
    {
      if(data_.empty())  return end();
      iterator i;
      i.container_= &data_;
      i.iterator1_= data_.find(key);
      if (i.iterator1_!=data_.end())  i.iterator2_= i.iterator1_->second.begin();
      return i;
    }
  //!\brief return an iterator where iterator's key is equal to key
  const_iterator begin(const t_key &key) const
    {
      if(data_.empty())  return end();
      const_iterator i;
      i.container_= &data_;
      i.iterator1_= data_.find(key);
      if (i.iterator1_!=data_.end())  i.iterator2_= i.iterator1_->second.begin();
      return i;
    }
  //!\brief return an iterator which is next to the last iterator whose key is equal to key
  iterator end(const t_key &key)
    {
      if(data_.empty())  return end();
      iterator i;
      i.container_= &data_;
      i.iterator1_= data_.find(key);
      if (i.iterator1_!=data_.end())  ++(i.iterator1_);
      if (i.iterator1_!=data_.end())  i.iterator2_= i.iterator1_->second.begin();
      return i;
    }
  //!\brief return an iterator which is next to the last iterator whose key is equal to key
  const_iterator end(const t_key &key) const
    {
      if(data_.empty())  return end();
      const_iterator i;
      i.container_= &data_;
      i.iterator1_= data_.find(key);
      if (i.iterator1_!=data_.end())  ++(i.iterator1_);
      if (i.iterator1_!=data_.end())  i.iterator2_= i.iterator1_->second.begin();
      return i;
    }

  size_t size () const {return total_elements_;}
  size_t size (const t_key &key) const
    {
      typename TData::const_iterator i= data_.find(key);
      if (i==data_.end())  return 0;
      else return i->second.size();
    }

  bool empty () const {return total_elements_==0;}

  void clear ()
    {
      data_.clear();
      total_elements_= 0;
    }

  void insert (const t_key &key, const t_value &value)
    {
      typename TData::iterator i= data_.find(key);
      if (i==data_.end())
      {
        std::pair<typename TData::iterator,bool> res= data_.insert(typename TData::value_type(key,std::list<t_value>()));
        res.first->second.push_back(value);
      }
      else
      {
        i->second.push_back(value);
      }
      ++total_elements_;
    }

  iterator erase (iterator i)
    {
      if (i.iterator1_==data_.end())  return i;
      iterator next(i);
      ++next;
      i.iterator1_->second.erase(i.iterator2_);
      if(i.iterator1_->second.empty())
        data_.erase(i.iterator1_);
      --total_elements_;
      return next;
    }

private:

  TData   data_;
  size_t  total_elements_;

};  // TListMap
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace loco_rabbits_detail
{
//===========================================================================================
/*!\brief iterator class of TListPQueue
*/
template <typename t_list_pqueue, typename t_iterator1, typename t_iterator2, typename t_value_ref, typename t_value_ptr, typename t_key_ref>
class TListPQueueIterator
//===========================================================================================
{
public:
  typedef TListPQueueIterator<t_list_pqueue,t_iterator1,t_iterator2,t_value_ref,t_value_ptr,t_key_ref>  TThis;

  TThis& operator++()
    {
      ++iterator2_;
      if (iterator2_==iterator1_->List.end())
      {
        ++iterator1_;
        if (iterator1_!=container_->end())
          iterator2_= iterator1_->List.begin();
      }
      return *this;
    }
  t_value_ref operator*()
    {
      return *iterator2_;
    }
  t_value_ptr operator->()
    {
      return &(*iterator2_);
    }
  bool operator==(const TThis &rhs)
    {
      if (iterator1_==container_->end() && rhs.iterator1_==container_->end())  return true;
      return iterator1_==rhs.iterator1_ && iterator2_==rhs.iterator2_;
    }
  bool operator!=(const TThis &rhs)
    {
      return !(operator==(rhs));
    }
  t_key_ref key()
    {
      return iterator1_->Key;
    }

  void set_container (const t_list_pqueue *c) {container_= c;}
  void set_iterator1 (t_iterator1 i1) {iterator1_= i1;}
  void set_iterator2 (t_iterator2 i2) {iterator2_= i2;}

private:
  const t_list_pqueue   *container_;
  t_iterator1    iterator1_;
  t_iterator2    iterator2_;

};  // TListPQueueIterator
//-------------------------------------------------------------------------------------------
}  // end of loco_rabbits_detail
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief priority queue {(key, list\<t_value\>)} template class
    \warning This template class is highly experimental. Please test this class before to use.
*/
template <typename t_key, typename t_value>
class TListPQueue
//===========================================================================================
{
public:
  enum TSortOrder {soDescending=0, soAscending};

  typedef TListPQueue<t_key,t_value>  TThis;
  typedef t_key    TKey;
  typedef t_value  TValue;
  typedef std::list<TValue>  TList;

  struct TItem
    {
      TKey   Key;
      TList  List;
      bool operator> (const TItem &rhs) const {return Key> rhs.Key;};
      bool operator< (const TItem &rhs) const {return Key< rhs.Key;};
    };

  typedef std::list<TItem>  TData;

  typedef loco_rabbits_detail::TListPQueueIterator<
                TData,
                typename TData::iterator,
                typename TList::iterator,
                TValue&, TValue*, TKey&>                     i_iterator;
  typedef loco_rabbits_detail::TListPQueueIterator<
                TData,
                typename TData::const_iterator,
                typename TList::const_iterator,
                const TValue&, TValue const*, const TKey&>   const_i_iterator;

  TListPQueue (TSortOrder so=soDescending) : sort_order_(so) {}

  i_iterator ibegin()
    {
      if(data_.empty())  return iend();
      i_iterator i;
      i.set_container(&data_);
      i.set_iterator1(data_.begin());
      i.set_iterator2(data_.begin()->List.begin());
      return i;
    }
  const_i_iterator ibegin() const
    {
      if(data_.empty())  return iend();
      const_i_iterator i;
      i.set_container(&data_);
      i.set_iterator1(data_.begin());
      i.set_iterator2(data_.begin()->List.begin());
      return i;
    }
  /*!\brief return an iterator of the first element of index-th List.
            if index is negative, index-th List is counted from the end */
  i_iterator ibegin(int index)
    {
      if(data_.empty())  return iend();
      i_iterator i;
      i.set_container(&data_);
      typename TData::iterator i1(list_itr_at(data_,index));
      i.set_iterator1(i1);
      if(i1!=data_.end()) i.set_iterator2(i1->List.begin());
      return i;
    }
  /*!\brief return an iterator of the first element of index-th List.
            if index is negative, index-th List is counted from the end */
  const_i_iterator ibegin(int index) const
    {
      if(data_.empty())  return iend();
      i_iterator i;
      i.set_container(&data_);
      typename TData::const_iterator i1(list_itr_at(data_,index));
      i.set_iterator1(i1);
      if(i1!=data_.end()) i.set_iterator2(i1->List.begin());
      return i;
    }
  i_iterator iend()
    {
      i_iterator i;
      i.set_container(&data_);
      i.set_iterator1(data_.end());
      return i;
    }
  const_i_iterator iend() const
    {
      const_i_iterator i;
      i.set_container(&data_);
      i.set_iterator1(data_.end());
      return i;
    }

  typename TData::iterator Push (const TKey &key, TList &v_list)
    {
      TItem item;
      item.Key= key;
      typename TData::iterator itr;
      if      (sort_order_==soDescending)  itr= data_.insert(find_if(data_.begin(),data_.end(),std::bind2nd(std::less<TItem>(),item)),item);
      else if (sort_order_==soAscending)   itr= data_.insert(find_if(data_.begin(),data_.end(),std::bind2nd(std::greater<TItem>(),item)),item);
      else {LERROR("fatal: sort_order_= "<<(int)sort_order_);lexit(df);}
      itr->List.splice(itr->List.begin(), v_list);
      return itr;
    }

  TData& Data()  {return data_;}
  const TData& Data() const {return data_;}

protected:
  TData       data_;
  TSortOrder  sort_order_;

};  // TListPQueue
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*!  allocate bubbles (hyper-shperes) of the same radius as widely and largely as possible

  sample:
  \code
  TBubbleSet bubbles;

  TBubbleSet::TRealVector scale(3, 1.0);  GenAt(scale,1)=2.0;
  bubbles.SetScale(scale);
  bubbles.SetCenterMin(TBubbleSet::TRealVector(3, -1.0));
  bubbles.SetCenterMax(TBubbleSet::TRealVector(3, 1.0));

  TReal time_step= 0.1l;
  bubbles.GenerateRandomly(20, 0.1l);
  TReal first_acc(bubbles.Step(time_step));
  TMovingAverageFilter<TReal>  acc_avr;
  acc_avr.Initialize(100,0.0l,first_acc);
  do
  {
    acc_avr(bubbles.Step(time_step));
  } while (acc_avr()>0.0002l*first_acc);

  bubbles.PrintCenters(cout);
  cout<<"Radius= "<< bubbles.Radius()<<endl;
  \endcode

*/
class TBubbleSet
//===========================================================================================
{
public:

  // typedef ColumnVector TRealVector;
  typedef std::vector<TReal> TRealVector;

  TBubbleSet()
    :
      margin_ratio_          (1.0l),
      spring_k_              (1.0l),
      bubble_dumping_        (2.0l),
      radius_dumping_        (10.0l),
      bubble_mass_           (5.0l),
      radius_mass_           (5.0l),
      radius_internal_force_ (0.1l)
    {}

  int Size() const {return bubble_set_.size();}
  const TReal& Radius() const {return radius_;}
  TRealVector Center(int index) const;

  // accessors:
  const TRealVector& CenterMax () const {return cmax_;}
  const TRealVector& CenterMin () const {return cmin_;}
  const TRealVector& Scale     () const {return scale_;}

  void SetCenterMax (const TRealVector &cmax);
  void SetCenterMin (const TRealVector &cmin);
  void SetScale (const TRealVector &scale);

  const TReal& MarginRatio         () const {return margin_ratio_          ;}
  const TReal& SpringK             () const {return spring_k_              ;}
  const TReal& BubbleDumping       () const {return bubble_dumping_        ;}
  const TReal& RadiusDumping       () const {return radius_dumping_        ;}
  const TReal& BubbleMass          () const {return bubble_mass_           ;}
  const TReal& RadiusMass          () const {return radius_mass_           ;}
  const TReal& RadiusInternalForce () const {return radius_internal_force_ ;}

  //! margin-ratio is in [0,1].  if 1: each sphere cannot overlap the boundary, 0: each center can be on the boundary
  void SetMarginRatio         (const TReal &v)  {margin_ratio_          = v;}
  void SetSpringK             (const TReal &v)  {spring_k_              = v;}
  void SetBubbleDumping       (const TReal &v)  {bubble_dumping_        = v;}
  void SetRadiusDumping       (const TReal &v)  {radius_dumping_        = v;}
  void SetBubbleMass          (const TReal &v)  {bubble_mass_           = v;}
  void SetRadiusMass          (const TReal &v)  {radius_mass_           = v;}
  void SetRadiusInternalForce (const TReal &v)  {radius_internal_force_ = v;}

  void GenerateRandomly (int N, const TReal &init_radius);
  TReal Step (const TReal &time_step=0.1l);

  void PrintCenters (std::ostream &os=std::cout) const;
  void PrintRadiusCenters (std::ostream &os=std::cout) const;

private:

  struct TBubble
    {
      TRealVector Center;
      TRealVector Velocity;
      TRealVector TotalForce;
    };

  TRealVector cmin_, cmax_, scale_;
  TRealVector scaled_cmin_, scaled_cmax_;
  TReal margin_ratio_;
  TReal spring_k_, bubble_dumping_, radius_dumping_;
  TReal bubble_mass_, radius_mass_;
  TReal radius_internal_force_;

  std::vector<TBubble>  bubble_set_;
  TReal radius_, radius_spd_, radius_total_force_;

};
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_small_classes_h
//-------------------------------------------------------------------------------------------

