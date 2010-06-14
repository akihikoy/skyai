//-------------------------------------------------------------------------------------------
/*! \file    file.h
    \brief   liblora - file utility (header)
    \author  Akihiko Yamaguchi
    \date    2008-

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

    -----------------------------------------------------------------------------------------

    \warning  this library depends on system-calls
*/
//-------------------------------------------------------------------------------------------
#ifndef loco_rabbits_file_h
#define loco_rabbits_file_h
//-------------------------------------------------------------------------------------------
#include <lora/common.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------

/*! \brief check the filename exists */
bool FileExists (const std::string &filename);

/*! \brief touch the filename */
void FileTouch (const std::string &filename);


//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
#endif // loco_rabbits_file_h
//-------------------------------------------------------------------------------------------

