//-------------------------------------------------------------------------------------------
/*! \file    gen-bubble.cpp
    \brief   tools - generate a NGnet where Gaussians are allocated by `bubble-allocation method'
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Jun.23, 2010
*/
//-------------------------------------------------------------------------------------------
#include <lora/oldngnet.h>
#include <lora/small_classes.h>
#include <lora/file.h>
#include <numeric>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
typedef TNGnetModelDyn TData;
typedef TNGnetDataSet<TData> TDataSet;


void Copy (const std::vector<TReal> &src, ColumnVector &dest)
{
  LASSERT1op1(GenSize(src),==,GenSize(dest));
  std::vector<TReal>::const_iterator src_itr(GenBegin(src)), src_last(GenEnd(src));
  for (TypeExt<ColumnVector>::iterator dest_itr(GenBegin(dest)); src_itr!=src_last; ++dest_itr,++src_itr)
    *dest_itr= *src_itr;
}

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace loco_rabbits;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
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


  int N(0);
  std::vector<TReal> xmax, xmin, scale;
  TReal sigma_ratio(0.6l);
  TReal margin_ratio(0.2l);
  TReal time_step(0.1l);

  vector<int> grid_levels;
  if (option("N")!="")
    N= ConvertFromStr<int>(option("N"));
  else
    {LERROR("option -N [0-9]+ must be specified (number of Gaussians)"); fatal_error=true;}

  if (option("xmin")!="")
    xmin= ConvertFromStr<std::vector<TReal> >(option("xmin"));
  else
    {LERROR("option -xmin '-1 -1 ...' must be specified"); fatal_error=true;}
  if (option("xmax")!="")
    xmax= ConvertFromStr<std::vector<TReal> >(option("xmax"));
  else
    {LERROR("option -xmax '1 1 ...' must be specified"); fatal_error=true;}

  if (option("scale")!="")
    scale= ConvertFromStr<std::vector<TReal> >(option("scale"));

  if (option("sratio")!="")
    sigma_ratio= ConvertFromStr<TReal>(option("sratio"));
  if (option("mratio")!="")
    margin_ratio= ConvertFromStr<TReal>(option("mratio"));
  if (option("step")!="")
    time_step= ConvertFromStr<TReal>(option("step"));

  //---------
  stringstream optss;
  if (option("help")!="" || fatal_error)
    {cerr<<"valid options:"<<endl; option.PrintUsed(); return fatal_error?1:0;}
  if (option.PrintNotAccessed(optss))
    {cerr<<"invalid options:"<<endl<<optss.str(); return 1;}

  LASSERT1op1(GenSize(xmax),==,GenSize(xmin));
  xdim= GenSize(xmax);

  //---------
  TBubbleSet bubbles;

  bubbles.SetScale(scale);
  bubbles.SetCenterMin(xmin);
  bubbles.SetCenterMax(xmax);
  bubbles.SetMarginRatio(margin_ratio);
  vector<TReal> diff= (xmax-xmin);
  TReal avr_len= accumulate(GenBegin(diff),GenEnd(diff),0.0l)/static_cast<TReal>(GenSize(diff));

  bubbles.GenerateRandomly(N, 0.5l*avr_len/real_pow(static_cast<TReal>(N),1.0l/static_cast<TReal>(xdim)));
  TReal first_acc(bubbles.Step(time_step));
  TMovingAverageFilter<TReal>  acc_avr;
  acc_avr.Initialize(100,0.0l,first_acc);
  // int ni(0);
  do
  {
    cout<<(acc_avr(bubbles.Step(time_step)))<<" \t "<<bubbles.Radius()<<endl;
    ofstream ofs("hoge.dat");
    // ofstream ofs(("bbl/frame"+IntToStr(ni++,4)+".dat").c_str());
    bubbles.PrintCenters(ofs);
    // for(int i(0);i<bubbles.Size();++i){ofs<<GenPrint(bubbles.Center(i))<<endl;}
    usleep(10000);
  } while (acc_avr()>0.0002l*first_acc);


  //---------
  TNGnetConfiguration ngnetcnf(TData::getXtDim(xdim,udim)/*xt*/,ydim/*y*/,xdim/*x*/,udim/*u*/,wcNone);
  TNGnet ngnet(ngnetcnf);

  Matrix invSigma(xdim,xdim,0.0);
  if(GenSize(scale)==xdim)
  {
    for(int r(0);r<xdim;++r)
    {
      if(xmin[r]!=xmax[r])
        invSigma(r,r)= Square(1.0l/(sigma_ratio*bubbles.Radius()*scale[r]));
      else
        invSigma(r,r)= 1.0e-6;
    }
  }
  else
  {
    for(int r(0);r<xdim;++r)
    {
      if(xmin[r]!=xmax[r])
        invSigma(r,r)= Square(1.0l/(sigma_ratio*bubbles.Radius()));
      else
        invSigma(r,r)= 1.0e-6;
    }
  }
  ColumnVector mumean(xdim,-1.0);
  for(int i(0);i<bubbles.Size();++i)
  {
    Copy(bubbles.Center(i), mumean);
    ngnet.add (TNGnet::unit_type(ngnet.getCnf()));
    TNGnet::unit_type  &unit(ngnet.back());
    unit.setWt(Matrix(unit.Wt().rows(),unit.Wt().cols(),0.0));
    unit.setmu(mumean);
    unit.setinvSigma(invSigma);
    unit.setsig2(0.0l);
  }

  ngnet.SaveToFile(outfilename);
  LMESSAGE(outfilename<<" is generated");
  return 0;
}
//-------------------------------------------------------------------------------------------
