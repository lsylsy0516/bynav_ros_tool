////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __OEM_DEBUG_FILE_HPP__
#define __OEM_DEBUG_FILE_HPP__

#include <ros/ros.h>

#include <string.h>
#include <fstream>

namespace bynav_ros_driver {
/**
 * File under .ros, for capturing debug output not suitable for console.
 */
class Oem7DebugFile {
 public:
  typedef struct FileOptions {
    int max_log_size;  // MB
    std::string file_name;
  } FileOptions;

  Oem7DebugFile();

  virtual bool initialize(const FileOptions &file_options);

  virtual bool write(const unsigned char* buf, size_t len);

 private:
  bool openLogFile();

  bool rotateLogFile();

 private:
  std::ofstream oem7_file_;  ///< output

  FileOptions file_options_;
};

}  // namespace bynav_ros_driver

#endif // __OEM_DEBUG_FILE_HPP__
