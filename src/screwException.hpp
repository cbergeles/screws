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

#ifndef SCREWEXCEPTION_H
#define SCREWEXCEPTION_H

#include "screwsInitLibrary.hpp"

#include <string>

namespace screws
{
  /*!
   * \class ScrewException
   * \ingroup libScrews
   * \brief This class handles different exception types for the screws library.
   * \date 27th April 2013
   */
  class SCREWS_EXPORT ScrewException
  {
  public:
    /*!
     * @brief Exception class for libScrews.
     * @param msg an std::string with the message to be displayed.
     * @param filename an std::string containing the filename where the error originated.
     * @param function an std::string containing the name of the function where the error originated from.
     * @param line an integer specifying the line of code in the file.
     */
    ScrewException(std::string msg, std::string filename, std::string function, int line);
    ~ScrewException();

    /// @return the reported error message.
    virtual const char* what() const;

  private:
    char* _buffer;
    std::string _msg;
    std::string _function;
    std::string _filename;
    int _line;
  };
};

#endif // SCREWEXCEPTION_H
