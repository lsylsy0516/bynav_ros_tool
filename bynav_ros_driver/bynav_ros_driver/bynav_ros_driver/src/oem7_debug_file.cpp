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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
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

#include "oem7_debug_file.hpp"

#include <ros/ros.h>
#include <sys/stat.h>

namespace {

double getFileSize(const std::string& file_path) {
  if (file_path.empty()) {
    return 0;
  }

  struct stat statbuf;
  stat(file_path.c_str(), &statbuf);
  double file_size = statbuf.st_size;

  return file_size / 1e6;
}

}  // namespace

namespace bynav_ros_driver {

Oem7DebugFile::Oem7DebugFile() {}

bool Oem7DebugFile::initialize(const FileOptions &file_options) {
if (file_options.file_name.empty()) {
    return true;  // Null initialization
  }

  file_options_ = file_options;
  if (!openLogFile()) {
    return false;
  }

  ROS_INFO_STREAM("OemDebugFile['" << file_options_.file_name << "'] opened.");

  return true;
}

/**
 * Reads input from file.
 *
 */
bool Oem7DebugFile::write(const unsigned char* buf, size_t len) {
  if (file_options_.file_name.empty()) return true;

  double file_size = getFileSize(file_options_.file_name);
  if (file_options_.max_log_size > 0 &&
      file_size - file_options_.max_log_size > 1e-6) {
    if (!rotateLogFile()) {
      return false;
    }
  }

  if (ros::isShuttingDown()) return false;

  oem7_file_.write(reinterpret_cast<const char*>(buf), len);
  int errno_value =
      errno;  // Cache errno locally, in case any ROS calls /macros affect it.

  if (!oem7_file_) {
    ROS_ERROR_STREAM("Oem7DebugFile[" << file_options_.file_name
                                      << "]: errno= " << errno_value << " '"
                                      << strerror(errno_value) << "'");
    return false;
  }

  return true;
}

bool Oem7DebugFile::openLogFile() {
  oem7_file_.open(file_options_.file_name,
                  std::ios::out | std::ios::binary | std::ios::app);
  int errno_value =
      errno;  // Cache errno locally, in case any ROS calls /macros affect it.
  if (!oem7_file_.is_open()) {
    ROS_ERROR_STREAM("OemDebugFile['"
                     << file_options_.file_name << "']: could not open; error= "
                     << errno_value << " '" << strerror(errno_value) << "'");
    return false;
  }

  return true;
}

bool Oem7DebugFile::rotateLogFile() {
  oem7_file_.close();

  std::string save_name =
      file_options_.file_name + "_" + std::to_string(time(NULL));
  ::rename(file_options_.file_name.c_str(), save_name.c_str());

  if (!openLogFile()) {
    return false;
  }

  return true;
}

}  // namespace bynav_ros_driver
