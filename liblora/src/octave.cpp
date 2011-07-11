//-------------------------------------------------------------------------------------------
/*! \file    octave.cpp
    \brief   liblora - liboctave extension (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

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
#include <lora/octave.h>
//-------------------------------------------------------------------------------------------
#include <lora/string_list.h>
#include <octave/config.h>
#include <octave/EIG.h>
#include <octave/dbleCHOL.h>
#include <octave/dbleSVD.h>
#include <cstdarg>
//-------------------------------------------------------------------------------------------
#ifdef PACKAGE_BUGREPORT
  #undef PACKAGE_BUGREPORT
  #undef PACKAGE_NAME
  #undef PACKAGE_STRING
  #undef PACKAGE_TARNAME
  #undef PACKAGE_VERSION
#endif
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

static void verror (const char *name, const char *fmt, va_list args)
{
  if (name)
    fprintf (stderr, "%s: ", name);
  vfprintf (stderr, fmt, args);
  fprintf (stderr, "\n");
  fflush (stderr);
}
void liboctave_error (const char *fmt, ...)
{
  va_list args;
  va_start (args, fmt);
  std::cerr<<ioscc::red;
  verror ("liboctave fatal", fmt, args);
  std::cerr<<ioscc::none;
  va_end (args);
  lexit(df);
}

namespace loco_rabbits_octave_detail
{
  struct TInitializer
  {
    TInitializer();
  } initializer;
  TInitializer::TInitializer()
  {
    set_liboctave_error_handler (&liboctave_error);
  }
}
//-------------------------------------------------------------------------------------------


//!\brief Generator of ColumnVector, RowVector
//!\note oct_type is instantiated with ColumnVector, RowVector
template <typename oct_type>
oct_type  OctGen1 (int size, ...)
{
  va_list argptr;
  va_start (argptr,size);

  oct_type res(size,0.0);
  typename oct_type::element_type *pres (OctBegin(res));
  for (int i(0);i<size;++i,++pres)
    *pres= va_arg (argptr,typename oct_type::element_type);

  va_end (argptr);
  return res;
}
template ColumnVector OctGen1<ColumnVector> (int size, ...);
template RowVector    OctGen1<RowVector>    (int size, ...);
//-------------------------------------------------------------------------------------------
//!\brief Generate Matrix
//!\note  ... = m(0,0),m(0,1),..,m(0,cols-1), m(1,0),..., m(rows-1,0),..,m(rows-1,cols-1)
//!\note oct_type is instantiated with Matrix
template <typename oct_type>
oct_type  OctGen2 (int rows, int cols, ...)
{
  va_list argptr;
  va_start (argptr,cols);

  oct_type res(rows,cols,0.0);
  for (int r(0);r<rows;++r)
    for (int c(0);c<cols;++c)
      res(r,c)= va_arg (argptr,typename oct_type::element_type);

  va_end (argptr);
  return res;
}
template Matrix  OctGen2<Matrix> (int rows, int cols, ...);
//-------------------------------------------------------------------------------------------


Matrix EigInverse (const Matrix &m, const double &mineigv, const double &maxeigv, int &info)
{
  EIG eig(m);
  int dim(eig.eigenvalues().length());
  DiagMatrix ev(dim,dim,0.0);
  for(int r(0);r<dim;++r)
  {
    double div (real(eig.eigenvalues()(r)));
    if (div<mineigv)        div=mineigv;
    else if (div>maxeigv)   div=maxeigv;
    ev(r,r) = 1.0/div;
  }
  return Matrix(real(eig.eigenvectors())*ev*real(eig.eigenvectors()).inverse(info));
}
//-------------------------------------------------------------------------------------------

Matrix EigRevision (const Matrix &m, const double &mineigv, const double &maxeigv, int &info)
{
  EIG eig(m);
  int dim(eig.eigenvalues().length());
  DiagMatrix ev(dim,dim,0.0);
  for(int r(0);r<dim;++r)
  {
    double evi (real(eig.eigenvalues()(r)));
    if (evi<mineigv)        evi=mineigv;
    else if (evi>maxeigv)   evi=maxeigv;
    ev(r,r) = evi;
  }
  return Matrix(real(eig.eigenvectors())*ev*real(eig.eigenvectors()).inverse(info));
}
//-------------------------------------------------------------------------------------------

/*! \brief calculate log-determinant of given Matrix */
double LogDeterminant (const Matrix &m, int &info)
{
  CHOL chol(m,info);  //!< \note octave decomposes Sigma so that Sigma=chol'*chol
  if (info!=0)
  {
    LERROR("error in LogDeterminant(): CHOL failed.");
    SVD svd(m);
    double ld(0.0);
    for (int r(0); r<m.rows(); ++r)  ld+=std::log(svd.singular_values()(r,r));
    return ld;
  }
  double ld(0.0);
  for (int r(0); r<m.rows(); ++r)  ld+=std::log(chol.chol_matrix()(r,r));
  return ld*2.0;
}
//-------------------------------------------------------------------------------------------

/*!\brief  Calculate a covariance matrix destSigma that is converted from a covariance matrix srcSigma
          by a linear mapping fLinearMapper
    \note  That is, when x is normally distributed with covariance srcSigma, this function calculates
          the covariance destSigma of q which is converted by the linear mapping fLinearMapper(x,q)
    \note q is also normally distributed */
void LinearMapSigma (const Matrix &srcSigma, Matrix &destSigma,
      const boost::function <void(const ColumnVector &src, ColumnVector &dest)>  &fLinearMapper)
{
  ColumnVector ext;
  fLinearMapper (srcSigma.column(0), ext);
  Matrix tmp (ext.length(), srcSigma.cols());
  for (int c(0); c<srcSigma.cols(); ++c)
  {
    fLinearMapper (srcSigma.column(c), ext);
    for (int r(0); r<ext.length(); ++r)  tmp(r,c)= ext(r);
  }
  destSigma.resize (ext.length(),ext.length());
  for (int r(0); r<tmp.rows(); ++r)
  {
    fLinearMapper (tmp.row(r).transpose(), ext);
    for (int c(0); c<ext.length(); ++c)  destSigma(r,c)= ext(c);
  }
  EnsureSymmetry(destSigma);
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MultiDimNormalDistribution
//===========================================================================================
void MultiDimNormalDistribution::SetCov (const Matrix &s)
{
  int info;
  Sigma = s;
  CHOL chol(Sigma,info);  //!< \note octave decomposes Sigma so that Sigma=chol'*chol
  if (info!=0)  {std::cerr<<"error in MultiDimNormalDistribution::SetCov(): CHOL failed."<<std::endl; lexit(df);}
  L = chol.chol_matrix().transpose();
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
