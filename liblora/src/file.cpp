//-------------------------------------------------------------------------------------------
/*! \file    file.cpp
    \brief   liblora - file utility (source)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Nov.30, 2008-

    Copyright (C) 2008, 2010  Akihiko Yamaguchi

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
#include <lora/file.h>
#include <fstream>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
//-------------------------------------------------------------------------------------------

/*! \brief check the filename exists */
bool FileExists (const std::string &filename)
{
  bool res(false);
  std::ifstream ifs (filename.c_str());
  res = ifs.is_open();
  ifs.close();
  return res;
  // struct stat sb;
  // if (stat(filename.c_str(),&sb)==-1)  return false;
  // return true;
}
//-------------------------------------------------------------------------------------------

/*! \brief touch the filename */
void FileTouch (const std::string &filename)
{
  std::ofstream ofs (filename.c_str());
  ofs.close();
}
//-------------------------------------------------------------------------------------------

/*! \brief under the polity, check if it is allowable to open the file */
bool CanOpenFile (const std::string &filename, TFileOverwritePolicy polity)
{
  if (FileExists(filename))
  {
    if      (polity==fopStop)       lexit(qfail);
    else if (polity==fopOverwrite)  return true;
    else if (polity==fopAsk)
    {
      std::cerr<<filename<<" already exists. Will you overwrite?"<<std::endl;
      std::cerr<<"  answer Yes    : overwrite"<<std::endl;
      std::cerr<<"  answer No     : not overwrite (continue)"<<std::endl;
      std::cerr<<"  answer Cancel : stop running"<<std::endl;
      switch(AskYesNoCancel())
      {
        case ryncYes    :  return true;
        case ryncNo     :  return false;
        case ryncCancel :  lexit(qfail); break;
        default : lexit(abort);
      }
    }
  }
  else  {return true;}
  return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TSharedFileStream
//===========================================================================================

/*static*/std::ofstream* TSharedFileStream::Open(const std::string &filename, TFileOverwritePolicy polity, bool *already_opened)
{
  if (already_opened)  *already_opened=false;
  if (filename=="")  return NULL;

  TFileMap::iterator itr1(instance().files_.find(filename));
  if(itr1!=instance().files_.end())
  {
    if(itr1->second->is_open())
    {
      if (already_opened)  *already_opened=true;
      return itr1->second;
    }
    else
    {
      bool can_open(CanOpenFile(filename,polity));
      std::ofstream *ofs(itr1->second);
      if(can_open)  ofs->open(filename.c_str());
      if(ofs->is_open())  return ofs;
      else  return NULL;
    }
  }

  bool can_open(CanOpenFile(filename,polity));

  std::ofstream *ofs= new std::ofstream();
  std::pair<TFileMap::iterator,bool> itr2
      = instance().files_.insert(TFileMap::value_type(filename,ofs));
  LASSERT(itr2.second);
  if(can_open)  ofs->open(filename.c_str());
  if(ofs->is_open())  return ofs;
  else  return NULL;
}
//-------------------------------------------------------------------------------------------

/*static*/void TSharedFileStream::Close(const std::string &filename)
{
  TFileMap::iterator itr(instance().files_.find(filename));
  if(itr!=instance().files_.end())
    close_file(itr);
}
//-------------------------------------------------------------------------------------------

/*static*/void TSharedFileStream::close_file(TFileMap::iterator itr)
{
  if(itr->second)
  {
    itr->second->close();
    delete itr->second;
  }
  itr->second=NULL;
  instance().files_.erase(itr);
}
//-------------------------------------------------------------------------------------------

/*static*/void TSharedFileStream::close_all()
{
  for(TFileMap::iterator itr(instance().files_.begin()),last(instance().files_.end()); itr!=last; ++itr)
    close_file(itr);
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------
