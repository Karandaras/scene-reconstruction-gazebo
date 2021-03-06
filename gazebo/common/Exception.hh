/*
 * Copyright 2012 Nate Koenig
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Author: Nathan Koenig
 * Date: 07 May 2007
 */

#ifndef _GAZEBO_EXCEPTION_HH_
#define _GAZEBO_EXCEPTION_HH_

#include <iostream>
#include <sstream>
#include <string>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \brief This macro logs an error to the throw stream and throws
    /// an exception that contains the file name and line number.
    #define gzthrow(msg) {std::ostringstream throwStream;\
      throwStream << msg << std::endl << std::flush;\
      throw gazebo::common::Exception(__FILE__, __LINE__, throwStream.str()); }

    /// \class Exception Exception.hh common/common.hh
    /// \brief Class for generating exceptions
    class Exception
    {
      /// \brief Constructor
      public: Exception();

      /// \brief Default constructor
      /// \param[in] _file File name
      /// \param[in] _line Line number where the error occurred
      /// \param[in] _msg Error message
      public: Exception(const char *_file,
                          int _line,
                          std::string _msg);

      /// \brief Destructor
      public: virtual ~Exception();

      /// \brief Return the error function
      /// \return The error function name
      public: std::string GetErrorFile() const;

      /// \brief Return the error string
      /// \return The error string
      public: std::string GetErrorStr() const;

      /// \brief Print the exception to std out.
      public: void Print() const;

      /// \brief The error function
      private: std::string file;

      /// \brief Line the error occured on
      private: int line;

      /// \brief The error string
      private: std::string str;

      /// \brief stream insertion operator for Gazebo Error
      /// \param[in] _out the output stream
      /// \param[in] _err the exception
      public: friend std::ostream &operator<<(std::ostream& _out,
                  const gazebo::common::Exception &_err)
              {
                return _out << _err.GetErrorStr();
              }
    };
    /// \}
  }
}

#endif
