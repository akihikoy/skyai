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
#include <lora/type_gen_oct.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
typedef TNGnetModelDyn TData;
typedef TNGnetDataSet<TData> TDataSet;


void GenGridNGnet (
    TNGnet &ngnet,
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

void GetInvSigma(const ColumnVector &xmax, const ColumnVector &xmin, const std::vector<int> &grid_levels, Matrix &invSigma)
{
  // xmin=;xmax=;Ngrid=; (1/(0.5*(xmax-xmin)/(Ngrid-1)/2))**2, (1/(0.9*(xmax-xmin)/(Ngrid-1)/2))**2
  LASSERT1op1(GenSize(xmax),==,GenSize(xmin));
  LASSERT1op1(GenSize(xmax),==,GenSize(grid_levels));
  int dim(GenSize(xmax));
  invSigma.resize(dim,dim);
  invSigma.fill(0.0);
  TypeExt<ColumnVector>::const_iterator xmax_itr(GenBegin(xmax));
  TypeExt<ColumnVector>::const_iterator xmin_itr(GenBegin(xmin));
  TypeExt<std::vector<int> >::const_iterator grid_itr(GenBegin(grid_levels));
  for(int r(0); r<dim; ++r,++xmax_itr,++xmin_itr,++grid_itr)
  {
    if(*xmax_itr!=*xmin_itr)
      invSigma(r,r)= Square(1.0/(0.75*(*xmax_itr-*xmin_itr)/static_cast<double>(*grid_itr-1)/2.0));
    else
      invSigma(r,r)= 1.0e-6;
  }
}
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------

int main( int argc, char **argv )
{
  srand ((unsigned)time(NULL));
  TOptionParser option (argc,argv);
  bool fatal_error(false);

  //---------
  string outfilename;
  int xdim(1),udim(1),ydim(1);

  if (option("out")=="")
    {LERROR("option -out outfilename must be specified"); fatal_error=true;}
  else
    outfilename = option("out");
  if (FileExists(outfilename))
    {LWARNING("output file "<<outfilename<<" already exists. overwrite?"); fatal_error= !AskYesNo();}

  if(option("xdim")!="")  xdim=StrToInt(option("xdim"));
  if(option("udim")!="")  udim=StrToInt(option("udim"));
  if(option("ydim")!="")  ydim=StrToInt(option("ydim"));


  TNxBase  unit_grid;
  ColumnVector xmax, xmin;
  Matrix invSigma;

  vector<int> grid_levels;
  if (option("unit_grid")!="")
    grid_levels= ConvertFromStr<vector<int> > (option("unit_grid"));
  else
    {LERROR("option -unit_grid '[0-9] [0-9] ...' must be specified"); fatal_error=true;}
  unit_grid.Init (grid_levels.begin(), grid_levels.end());

  if (option("xmin")!="")
    xmin=StringToColumnVector(option("xmin"));
  else
    {LERROR("option -xmin '-1 -1 ...' must be specified"); fatal_error=true;}
  if (option("xmax")!="")
    xmax=StringToColumnVector(option("xmax"));
  else
    {LERROR("option -xmax '1 1 ...' must be specified"); fatal_error=true;}

  if (option("invSigma")!="")
  {
    if (option("invSigma")=="auto")
      GetInvSigma(xmax, xmin, grid_levels, invSigma);
    else
      invSigma= Matrix(DiagMatrix(StringToColumnVector(option("invSigma"))));
  }
  else
    {LERROR("option -invSigma '1 1 ...' must be specified"); fatal_error=true;}

  //---------
  stringstream optss;
  if (option("help")!="" || fatal_error)
    {cerr<<"valid options:"<<endl; option.PrintUsed(); return fatal_error?1:0;}
  if (option.PrintNotAccessed(optss))
    {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}

  LASSERT1op1(GenSize(xmax),==,GenSize(xmin));
  LASSERT1op1(GenSize(xmax),==,GenSize(grid_levels));
  xdim= GenSize(xmax);

  //---------
  TNGnetConfiguration ngnetcnf(TData::getXtDim(xdim,udim)/*xt*/,ydim/*y*/,xdim/*x*/,udim/*u*/,wcNone);
  TNGnet ngnet(ngnetcnf);

  //gen_init (ngnet, unitPerDim, xmin, xmax, /*muvar*/, isigmaSD, /*isigmavar*/);
  GenGridNGnet (ngnet, unit_grid, xmin, xmax, invSigma);

  ngnet.SaveToFile(outfilename);
  LMESSAGE(outfilename<<" is generated");
  return 0;
}
//-------------------------------------------------------------------------------------------

