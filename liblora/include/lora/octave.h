//-------------------------------------------------------------------------------------------
/*! \file    octave.h
    \brief   liblora - liboctave extension (header)
    \author  Akihiko Yamaguchi
    \date    2008
    \date    2009

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

    -----------------------------------------------------------------------------------------

    \note   Matrix(r,c) in liboctave is allocated as
            (11,21,..,r1 ; 12,22,..,r2 ; ... ; 1c,2c,..,rc)  on the memory
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_octave_h
#define loco_rabbits_octave_h
//-------------------------------------------------------------------------------------------
#include <lora/octave_fwd.h>
#include <cmath>
#include <lora/math.h>
#include <lora/rand.h>
#include <cstring> // for memcpy
#include <boost/function.hpp>  // TODO it is better not to include it (heavy)
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

// void initialize_liboctave (void);
//-------------------------------------------------------------------------------------------

//!\brief Generator of ColumnVector, RowVector
//!\note oct_type is instantiated with ColumnVector, RowVector
template <typename oct_type>
oct_type  OctGen1 (int size, ...);
//!\brief Generate Matrix
//!\note  ... = m(0,0),m(0,1),..,m(0,cols-1), m(1,0),..., m(rows-1,0),..,m(rows-1,cols-1)
//!\note oct_type is instantiated with Matrix
template <typename oct_type>
oct_type  OctGen2 (int rows, int cols, ...);
//-------------------------------------------------------------------------------------------

inline const ColumnVector& operator*= (ColumnVector &lhs, const double &rhs)
{
  for (double *ptr=OctBegin(lhs),*last=OctEnd(lhs); ptr!=last; ++ptr)
    *ptr*= rhs;
  return lhs;
}
inline const RowVector& operator*= (RowVector &lhs, const double &rhs)
{
  for (double *ptr=OctBegin(lhs),*last=OctEnd(lhs); ptr!=last; ++ptr)
    *ptr*= rhs;
  return lhs;
}
//-------------------------------------------------------------------------------------------

inline const ColumnVector& operator/= (ColumnVector &lhs, const double &rhs)
{
  for (double *ptr=OctBegin(lhs),*last=OctEnd(lhs); ptr!=last; ++ptr)
    *ptr/= rhs;
  return lhs;
}
inline const RowVector& operator/= (RowVector &lhs, const double &rhs)
{
  for (double *ptr=OctBegin(lhs),*last=OctEnd(lhs); ptr!=last; ++ptr)
    *ptr/= rhs;
  return lhs;
}
//-------------------------------------------------------------------------------------------

/*!\brief Copy an octave array
  \note The operator= for the liboctave array is implemented using a reference counting.
      So, when an array is often modified after a copy, memory allocations often arise.
      In such a case, direct copy (implemented in this function) is efficient.
  \note lhs_type and rhs_type should be either {ColumnVector, RowVector}
*/
template <typename lhs_type, typename rhs_type>
inline const lhs_type& CopyOctArray (lhs_type &lhs, const rhs_type &rhs)
{
  if(OctBegin(lhs)==OctBegin(rhs))  return lhs;
  lhs.resize(rhs.length());
  const int N (lhs.length());
  // if(N!=rhs.length())
  //   {LERROR("size nonconformant in vector operation: lhs.length()= "
  //     <<N<<", rhs.size()= "<<rhs.length()); lexit(df);}
  std::memcpy(OctBegin(lhs),OctBegin(rhs),sizeof(*OctBegin(lhs))*N);
  return lhs;
}
//-------------------------------------------------------------------------------------------

#define OCT_INVERSE(mat) OctErrorInverse(mat,__LINE__,__FILE__)
inline Matrix OctErrorInverse(const Matrix &mat, int line_num, const char* file_name)
{
  int ie;
  Matrix res = mat.inverse(ie);
  if (ie != 0)
    {LERROR("matrix error in "<<file_name<<":"<<line_num<<": "
      "mat.inverse() have returned a error-code "<<ie);}
  return res;
}
//-------------------------------------------------------------------------------------------

template<>  // specialization of the template
inline void SetZero (ColumnVector &val)
{
  // // for(int i(val.dim1()-1);i>=0;--i)   // this code is slow
  // //   val(i) = 0.0;
  std::fill(OctBegin(val),OctEnd(val),0.0l);  // fast
  // val=ColumnVector(val.length(),0.0l);  // for huge dimensional val, this is faster
}
template<>  // specialization of the template
inline void SetZero (RowVector &val)
{
  std::fill(OctBegin(val),OctEnd(val),0.0l);  // fast
  // val=ColumnVector(val.length(),0.0l);  // for huge dimensional val, this is faster
}
template<>
inline void SetZero (Matrix &val)
{
  // // for(int c(val.cols()-1);c>=0;--c)   // this code is slow
  // //   for(int r(val.rows()-1);r>=0;--r)
  // //     val(r,c) = 0.0;
  std::fill(OctBegin(val),OctEnd(val),0.0l);  // fast
  // val=Matrix(val.rows(),val.cols(),0.0l);  // for huge dimensional val, this is faster
}

template<>  // specialization of the template
inline void SetOne (ColumnVector &val)
{
  std::fill(OctBegin(val),OctEnd(val),1.0l);  // fast
}
template<>  // specialization of the template
inline void SetOne (RowVector &val)
{
  std::fill(OctBegin(val),OctEnd(val),1.0l);  // fast
}
template<>
inline void SetOne (Matrix &val)
{
  std::fill(OctBegin(val),OctEnd(val),1.0l);  // fast
}

inline Matrix GetEye (const int i)
{
  Matrix m(i,i,0.0);
  for (int r(0);r<i;++r) m(r,r)=1.0;
  return m;
}
//-------------------------------------------------------------------------------------------


inline double Quadratic (const ColumnVector &x, const Matrix &m)
{
  if(x.length()!=m.rows()||x.length()!=m.cols())
    {LERROR("in _quadratic, invalid dimensions:"
      <<"x:"<<x.length()<<" m:"<<m.rows()<<"x"<<m.cols()); lexit(df);}
  double sum(0.0l), subsum;
  const double *xfirst(OctBegin(x));
  const double *xlast(OctEnd(x));
  const double *mitr(OctBegin(m));
  const double *x1itr(NULL), *x2itr(NULL);
  for(x1itr=xfirst; x1itr!=xlast; ++x1itr)
  {
    subsum= 0.0l;
    for(x2itr=xfirst; x2itr!=xlast; ++x2itr, ++mitr)
      subsum+= (*x2itr)*(*mitr);
    sum+= (*x1itr)*(subsum);
  }
  return sum;
}
//-------------------------------------------------------------------------------------------

inline double Trace (const Matrix &M)
{
  if (M.cols()!=M.rows())
    {std::cerr<<"can't calculate trace of the Matrix M that cols()!=rows()"<<std::endl; lexit(df);}
  double res(0.0);
  for(int c(M.cols()-1);c>=0;--c)
    res += M(c,c);
  return res;
}
//-------------------------------------------------------------------------------------------

inline double TraceSq( const Matrix &M )
{
  double res(0.0);
  for(int c(M.cols()-1);c>=0;--c)
    res += Square(M(c,c));
  return res;
}
//-------------------------------------------------------------------------------------------

inline ColumnVector CrossCV3( const ColumnVector &a, const ColumnVector &b )
{
  ColumnVector ret(3);
  ret(0) = a(1)*b(2) - b(1)*a(2);
  ret(1) = a(2)*b(0) - b(2)*a(0);
  ret(2) = a(0)*b(1) - b(0)*a(1);
  return ret;
}
//-------------------------------------------------------------------------------------------

template <typename t_oct_vec>
inline Matrix GetWedge (const  t_oct_vec &w)
{
  Matrix wedge(3,3);
  wedge(0,0)=0.0;    wedge(0,1)=-w(2);  wedge(0,2)=w(1);
  wedge(1,0)=w(2);   wedge(1,1)=0.0;    wedge(1,2)=-w(0);
  wedge(2,0)=-w(1);  wedge(2,1)=w(0);   wedge(2,2)=0.0;
  return wedge;
}
//-------------------------------------------------------------------------------------------

//! w should be ColumnVector(3) or RowVector(3)
//! include lora/stl_math.h to use
template <typename t_oct_vec>
inline Matrix Rodrigues (const t_oct_vec &w, const double &dt)
{
  double norm_w = GetNorm(w), th = norm_w * dt;
  Matrix w_wedge(3,3);
  w_wedge = GetWedge( w / norm_w );
  return GetEye(3) + w_wedge * sin(th) + w_wedge * w_wedge * (1.0-cos(th));
}
//-------------------------------------------------------------------------------------------

//!\brief return rotation matrix that rotates theta around a
//! \note if a is already normalized, use Rodrigues rather than this function
//! include lora/stl_math.h to use
template <typename t_oct_vec>
inline Matrix RotateAroundAxis (const t_oct_vec &a, const double &theta)
{
  return Rodrigues (GetNormalized(a), theta);
}
//-------------------------------------------------------------------------------------------

//! \brief ensure perfect symmetry
inline void EnsureSymmetry (Matrix &m)
{
  m = 0.5*(m+m.transpose());
}
//-------------------------------------------------------------------------------------------

Matrix EigInverse (const Matrix &m, const double &mineigv, const double &maxeigv, int &info);

inline Matrix EigInverse (const Matrix &m, const double &mineigv=DBL_MIN, const double &maxeigv=DBL_MAX)
{
  int info;
  return EigInverse (m, mineigv, maxeigv, info);
}
inline Matrix EigInverse (const Matrix &m, int &info)
{
  return EigInverse (m, DBL_MIN, DBL_MAX, info);
}
//-------------------------------------------------------------------------------------------

Matrix EigRevision (const Matrix &m, const double &mineigv, const double &maxeigv, int &info);

inline Matrix EigRevision (const Matrix &m, const double &mineigv, const double &maxeigv)
{
  int info;
  return EigRevision (m, mineigv, maxeigv, info);
}
//-------------------------------------------------------------------------------------------

/*! \brief calculate log-determinant of given Matrix */
double LogDeterminant (const Matrix &m, int &info);

inline double LogDeterminant (const Matrix &m)
{
  int info;
  double det (LogDeterminant(m,info));
  if (info!=0) lexit(df);
  return det;
}
//-------------------------------------------------------------------------------------------

/*!\brief return affine transform matrix[4][4] from R[3][3] and p[3]
    \param p ColumnVector(3); position
    \param R Matrix(3,3); pose[ex,ey,ez]
    \return Matrix(4,4);
    \note p, R & mat should have dim, and that is not checked for.  */
inline Matrix GetAffineMatrix (const ColumnVector &p, const Matrix &R)
{
  Matrix res(4,4,0.0);
  res.insert (R,0,0);
  res.insert (p,0,3);
  res(3,3) = 0.0;
  return res;
}
//-------------------------------------------------------------------------------------------

/*!\brief  Calculate a covariance matrix destSigma that is converted from a covariance matrix srcSigma
          by a linear mapping fLinearMapper
    \note  That is, when x is normally distributed with covariance srcSigma, this function calculates
          the covariance destSigma of q which is converted by the linear mapping fLinearMapper(x,q)
    \note q is also normally distributed */
void LinearMapSigma (const Matrix &srcSigma, Matrix &destSigma,
      const boost::function <void(const ColumnVector &src, ColumnVector &dest)>  &fLinearMapper);
//-------------------------------------------------------------------------------------------



//===========================================================================================
/*! \brief generate random variables of multi-dimensional normal distribution */
class MultiDimNormalDistribution
//===========================================================================================
{
private:
  Matrix Sigma, L;
public:
  MultiDimNormalDistribution (void) {};
  MultiDimNormalDistribution (const Matrix &s) : Sigma (s)  {SetCov(Sigma);};
  void SetCov (const Matrix &s);
  const Matrix& GetSigma (void) const {return Sigma;};
  const Matrix& GetChol (void) const {return L;};
  ColumnVector operator() (void) const
    {
      ColumnVector r(L.dim1(), 0.0);
      for (double *ptr(OctBegin(r)); ptr!=OctEnd(r); ++ptr)  *ptr= NDRand();
      return L*r;
    };
};
//-------------------------------------------------------------------------------------------


}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_octave_h
//-------------------------------------------------------------------------------------------
