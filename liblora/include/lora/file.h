//-------------------------------------------------------------------------------------------
/*! \file    file.h
    \brief   liblora - file utility (header)
    \author  Akihiko Yamaguchi
    \date    2008-
    \date    Sep.07,2010  added TFileOverwritePolicy, CanOpenFile

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
#ifndef loco_rabbits_file_h
#define loco_rabbits_file_h
//-------------------------------------------------------------------------------------------
#include <map>
#include <iosfwd>
#include <lora/string.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

enum TFileOverwritePolicy
{
  fopStop            =0,
  fopNotOverwrite    ,
  fopOverwrite       ,
  fopAsk
};
ENUM_STR_MAP_BEGIN(TFileOverwritePolicy)
  ENUM_STR_MAP_ADD(fopStop             )
  ENUM_STR_MAP_ADD(fopNotOverwrite     )
  ENUM_STR_MAP_ADD(fopOverwrite        )
  ENUM_STR_MAP_ADD(fopAsk              )
ENUM_STR_MAP_END  (TFileOverwritePolicy)
//-------------------------------------------------------------------------------------------

/*! \brief check the filename exists */
bool FileExists (const std::string &filename);

/*! \brief touch the filename */
void FileTouch (const std::string &filename);

/*! \brief under the polity, check if it is allowable to open the file */
bool CanOpenFile (const std::string &filename, TFileOverwritePolicy polity);

//-------------------------------------------------------------------------------------------

//===========================================================================================
/*! \brief shared file class that enables to open a file with the same filename from
            some routines simultaneously.
    \note the class is singleton  */
class TSharedFileStream
//===========================================================================================
{
public:
  static std::ofstream* Open(const std::string &filename, TFileOverwritePolicy polity=fopAsk, bool *already_opened=NULL);
  static void Close(const std::string &filename);

private:
  typedef std::map<std::string, std::ofstream*> TFileMap;
  TFileMap files_;

  TSharedFileStream() {};
  TSharedFileStream(const TSharedFileStream&);
  ~TSharedFileStream() {};

  static TSharedFileStream& instance()
    {
      static TSharedFileStream entity;
      return entity;
    }

  static void close_file(TFileMap::iterator itr);
  static void close_all();

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_file_h
//-------------------------------------------------------------------------------------------

