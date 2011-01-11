//-------------------------------------------------------------------------------------------
/*! \file    data_logger.cpp
    \brief   libskyai - data logger module (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.29, 2009-

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
#include <skyai/modules_core/data_logger.h>
#include <fstream>
#include <boost/bind.hpp>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
using namespace std;
// using namespace boost;

static bool open_log_file (ofstream &log_file, const string &filename, TFileOverwritePolicy file_overwrite_policy)
{
  if (filename!="")
  {
    if (CanOpenFile(filename,file_overwrite_policy))
      {log_file.open(filename.c_str());}
  }
  if (log_file.is_open())  return true;
  else  return false;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class MSimpleDataLoggerInterface
//===========================================================================================

/*virtual*/void MSimpleDataLoggerInterface::slot_initialize_exec (void)
{
  bool already_opened;
  log_file_= TSharedFileStream::Open(Agent().GetDataFileName(conf_.FileName), conf_.FileOverwritePolicy, &already_opened);
  if (!conf_.FileSharable && already_opened)
  {
    LWARNING(conf_.FileName<<" is already opened by another module");
    log_file_= NULL;
  }
}
//-------------------------------------------------------------------------------------------

/*virtual*/void MSimpleDataLoggerInterface::slot_newline_exec (void)
{
  if (log_file_==NULL || !log_file_->is_open())  return;
  *log_file_<<std::endl;
}
//-------------------------------------------------------------------------------------------

#define LOG(x_idx)                                        \
  if (in_data##x_idx.ConnectionSize()>0)                  \
    *log_file_<<ConvertToStr(in_data##x_idx.GetFirst());  \
  else *log_file_<<conf_.NoDataMark;

//===========================================================================================
// class MSimpleDataLogger1
//===========================================================================================

template <typename t_data1>
override void MSimpleDataLogger1<t_data1>::slot_log_exec (void)
{
  if (log_file_==NULL || !log_file_->is_open())  return;
  LOG(1);
  *log_file_<<std::endl;
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class MSimpleDataLogger2
//===========================================================================================

template <typename t_data1, typename t_data2>
override void MSimpleDataLogger2<t_data1,t_data2>::slot_log_exec (void)
{
  if (log_file_==NULL || !log_file_->is_open())  return;
  LOG(1);
  *log_file_<<conf_.Delim;
  LOG(2);
  *log_file_<<std::endl;
}
//-------------------------------------------------------------------------------------------

#undef LOG

//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TRealVector)
SKYAI_ADD_MODULE(MSimpleDataLogger1_TInt)
SKYAI_ADD_MODULE(MSimpleDataLogger1_TReal)
SKYAI_ADD_MODULE(MSimpleDataLogger1_TIntVector)
SKYAI_ADD_MODULE(MSimpleDataLogger1_TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TRealVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TInt)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TReal)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TIntVector)
SKYAI_INSTANTIATE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TRealVector)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TInt_TInt)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TInt_TReal)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TInt_TIntVector)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TInt_TRealVector)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TReal_TInt)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TReal_TReal)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TReal_TIntVector)
SKYAI_ADD_MODULE(MSimpleDataLogger2_TReal_TRealVector)
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class MUniversalDataLogger
//===========================================================================================

override void MUniversalDataLogger::slot_initialize_exec (void)
{
  TParent::slot_initialize_exec();
  make_data_list();
}
//-------------------------------------------------------------------------------------------

override void MUniversalDataLogger::slot_log_exec (void)
{
  if (log_file_==NULL || !log_file_->is_open())  return;

  std::string  delim ("");
  int i(1);
  for (TObserverMap::const_iterator itr(data_map_.begin()); itr!=data_map_.end(); ++itr, ++i)
  {
    while (i<itr->first)
    {
      *log_file_<<delim<<conf_.NoDataMark;
      delim= conf_.Delim;
      ++i;
    }
    *log_file_<<delim<<itr->second.Observe();
    delim= conf_.Delim;
  }
  *log_file_<<endl;
}
//-------------------------------------------------------------------------------------------

template <typename t_port>
inline TString observe_data (t_port *pport, typename t_port::TConnectedPortIterator itr)
{
  return ConvertToStr(pport->GetCurrent(itr));
}

void MUniversalDataLogger::make_data_list (void)
{
  // check conf2_.OrderOfColumns...
  int max_order(0);
  for (TOrderMap::const_iterator itr(conf2_.OrderOfColumns.begin());
        itr!=conf2_.OrderOfColumns.end(); ++itr)
  {
    if (itr->second<=0) {LWARNING(ModuleUniqueCode()<<" allow only positive numbers as the order."); continue;}
    if (itr->second>max_order)  max_order= itr->second;
  }

  if (max_order>0 && conf2_.PutBlankData)  ++max_order;

  // construct data_map_
  std::map<TString, TInt>::const_iterator  order_itr;
  data_map_.clear();
  #define CONSTRUCT_DATA_MAP(x_in_data)                                                               \
    for (GET_PORT_TYPE(x_in_data)::TConnectedPortIterator itr(x_in_data.ConnectedPortBegin());        \
            itr!=x_in_data.ConnectedPortEnd(); ++itr)                                                 \
    {                                                                                                 \
      std::string unique_code= ParentCModule().SearchSubPortUniqueCode((*itr));                       \
      order_itr= conf2_.OrderOfColumns.find(unique_code);                                             \
      TObserver  observer (unique_code,                                                               \
                            boost::bind (observe_data<GET_PORT_TYPE(x_in_data)>, &x_in_data, itr));   \
      if (order_itr != conf2_.OrderOfColumns.end() && order_itr->second>0)                            \
      {                                                                                               \
        data_map_[order_itr->second]= observer;                                                       \
      }                                                                                               \
      else                                                                                            \
      {                                                                                               \
        ++max_order;                                                                                  \
        data_map_[max_order]= observer;                                                               \
      }                                                                                               \
    }
  CONSTRUCT_DATA_MAP(in_data_int)
  CONSTRUCT_DATA_MAP(in_data_real)
  CONSTRUCT_DATA_MAP(in_data_string)
  CONSTRUCT_DATA_MAP(in_data_int_vector)
  CONSTRUCT_DATA_MAP(in_data_real_vector)
  CONSTRUCT_DATA_MAP(in_data_composite1)
  #undef CONSTRUCT_DATA_MAP

  if (log_file_==NULL || !log_file_->is_open())  return;
  if (conf2_.CommentOutMark=="")  return;
  // write header
  *log_file_<<conf2_.CommentOutMark;
  int i(1);
  for (TObserverMap::const_iterator itr(data_map_.begin()); itr!=data_map_.end(); ++itr, ++i)
  {
    while (i<itr->first)
    {
      *log_file_<<conf_.Delim<<conf_.NoDataMark;
      ++i;
    }
    *log_file_<<conf_.Delim<<itr->second.PortName/*<<"("<<i<<")"*/;
  }
  *log_file_<<endl;
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MUniversalDataLogger)
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class MRealMatrixLogger
//===========================================================================================

/*virtual*/void MRealMatrixLogger::slot_log_exec (void)
{
  TString filename;
  if (conf_.FileNumFillZero)  filename= conf_.FilePrefix + IntToStr(get_file_number(),conf_.FileNumDigits) + conf_.FileSuffix;
  else                        filename= conf_.FilePrefix + IntToStr(get_file_number()) + conf_.FileSuffix;
  std::ofstream  ofs;
  if (!open_log_file (ofs, Agent().GetDataFileName(filename), conf_.FileOverwritePolicy))  return;
  ofs<< get_real_matrix();
  ofs.close();
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
SKYAI_ADD_MODULE(MRealMatrixLogger)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

