//  Copyright (c) 2015  Christos Bergeles and Imperial College London

//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "screwException.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace screws;

ScrewException::ScrewException(std::string msg, std::string filename, std::string function, int line)
{
  _msg = msg;
  _filename = filename;
  _function = function;
  _line = line;
  _buffer = (char*)malloc(500 * sizeof(char));
}

ScrewException::~ScrewException() {}

const char* ScrewException::what() const
{
  sprintf(_buffer, "Encountered: %s, in file %s, function %s, line %d.", _msg.c_str(), _filename.c_str(), _function.c_str(), _line);
  return _buffer;
}
