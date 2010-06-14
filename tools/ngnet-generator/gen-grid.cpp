//-------------------------------------------------------------------------------------------
/*! \file   gen-grid.cpp
    \brief  tools - generate a `grid-NGnet' where Gaussians are allocated on a grid
    \author Akihiko Yamaguchi
    \date   Nov.22,2008
    \date   Nov.23,2009

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
#include <lora/small_classes.h>
#include <lora/stl_ext.h>
#include <lora/file.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
typedef TNGnetModelDyn TData;
typedef TNGnetDataSet<TData> TDataSet;

// ColumnVector random_cvector (const ColumnVector &mean, const TReal &var)
//   //! generate random column-vector
// {
//   ColumnVector vec(mean);
//   for(int r(vec.dim1()-1); r>=0; --r)
//     vec(r) += Rand(-var,var);
//   return vec;
// }
//-------------------------------------------------------------------------------------------

// Matrix random_matrix(int r, int c, const TReal &stddev, const TReal &var)
//   //! generate random matrix
// {
//   Matrix mat(r,c,0.0);
//   for(int c(mat.cols()-1); c>=0; --c)
//     for(int r(mat.rows()-1); r>=0; --r)
//     {
//       if(r==c) mat(r,c) = Rand(stddev-var,stddev+var);
//       else mat(r,c) = Rand(-var,+var);
//     }
//   return mat;
// }
//-------------------------------------------------------------------------------------------

// invSigma= Matrix(DiagMatrix(unit.invSigma().rows(),unit.invSigma().cols(), Square(isigmaSD)))

void gen_grid_ngnet (TNGnet &ngnet,
  TNxBase &unit_grid,
  ColumnVector xmin, ColumnVector xmax,
  const Matrix &invSigma )
{
  LASSERT1op1(ngnet.getCnf().XDIM(),==,unit_grid.Digits());
  LASSERT1op1(unit_grid.Digits(),==,xmin.length());
  LASSERT1op1(unit_grid.Digits(),==,xmax.length());

  ColumnVector mumean (unit_grid.Digits(),0.0);
  ColumnVector muunit (unit_grid.Digits(),0.0);

  for (int r(0); r<muunit.length(); ++r)
  {
    int divs= unit_grid.Nx()[r];
    if(divs>1)
      muunit(r)= (xmax(r)-xmin(r))/static_cast<TReal>(divs-1);
    else
      muunit(r)= xmax(r)-xmin(r);
  }

  for(unit_grid.Init(); unit_grid.Cont(); unit_grid.Step())
  {
    for(int r(0); r<mumean.length(); ++r)
      if (unit_grid.Nx()[r]==1)
        mumean(r)=0.5*(xmin(r)+xmax(r));
      else
        mumean(r)=xmin(r)+muunit(r)*unit_grid[r];
    ngnet.add (TNGnet::unit_type(ngnet.getCnf()));
    TNGnet::unit_type  &unit(ngnet.back());
    unit.setWt(Matrix(unit.Wt().rows(),unit.Wt().cols(),0.0));
    unit.setmu(mumean);
    unit.setinvSigma(invSigma);
    unit.setsig2(0.0l);
  }
}
//-------------------------------------------------------------------------------------------

};
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int main( int argc, char **argv )
{
  srand ((unsigned)time(NULL));
  TOptionParser OPTION (argc,argv);
  bool fatal_error(false);

  //---------
  string outfilename;
  int xdim(-1),udim(1),ydim(1);

  if (OPTION("out")=="")
    {LERROR("option -out outfilename must be specified"); fatal_error=true;}
  else
    outfilename = OPTION("out");
  if (FileExists(outfilename))
    {LWARNING("output file "<<outfilename<<" already exists. overwrite?"); fatal_error= !AskYesNo();}
  if (OPTION("xdim")=="")
    {OPTION["xdim"]; LERROR("option -xdim must be specified"); fatal_error=true;}
  else
  {
    xdim=StrToInt(OPTION("xdim"));
    if(OPTION("udim")!="")  udim=StrToInt(OPTION("udim"));
    if(OPTION("ydim")!="")  ydim=StrToInt(OPTION("ydim"));
  }


  TNxBase  unit_grid;
  ColumnVector xmax(xdim), xmin(xdim);
  Matrix invSigma;

  vector<int> grid_levels;
  if (OPTION("unit_grid")!="")
    grid_levels= ConvertFromStr<vector<int> > (OPTION("unit_grid"));
  else
    {LERROR("option -unit_grid '[0-9] [0-9] ...' must be specified"); fatal_error=true;}
  unit_grid.Init (grid_levels.begin(), grid_levels.end());

  if (OPTION("xmin")!="")
    xmin=StringToColumnVector(OPTION("xmin"));
  else
    {LERROR("option -xmin '-1 -1 ...' must be specified"); fatal_error=true;}
  if (OPTION("xmax")!="")
    xmax=StringToColumnVector(OPTION("xmax"));
  else
    {LERROR("option -xmax '1 1 ...' must be specified"); fatal_error=true;}

  if (OPTION("invSigma")!="")
    invSigma= Matrix(DiagMatrix(StringToColumnVector(OPTION("invSigma"))));
  else
    {LERROR("option -invSigma '1 1 ...' must be specified"); fatal_error=true;}

  //---------
  stringstream optss;
  if (OPTION("help")!="" || fatal_error)
    {cerr<<"valid options:"<<endl; OPTION.PrintUsed(); return fatal_error?1:0;}
  if (OPTION.PrintNotAccessed(optss))
    {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}

  //---------
  TNGnetConfiguration ngnetcnf(TData::getXtDim(xdim,udim)/*xt*/,ydim/*y*/,xdim/*x*/,udim/*u*/,wcNone);
  TNGnet ngnet(ngnetcnf);

  //gen_init (ngnet, unitPerDim, xmin, xmax, /*muvar*/, isigmaSD, /*isigmavar*/);
  gen_grid_ngnet (ngnet, unit_grid, xmin, xmax, invSigma);

  ngnet.SaveToFile(outfilename);
  LMESSAGE(outfilename<<" is generated");
  return 0;
}
//-------------------------------------------------------------------------------------------

