// Copyright 2019 ADLINK Technology Ltd. Advanced Robotic Platform Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMNIBOT_BASE__VISIBILITY_CONTROL_H_
#define OMNIBOT_BASE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OMNIBOT_BASE_EXPORT __attribute__ ((dllexport))
    #define OMNIBOT_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define OMNIBOT_BASE_EXPORT __declspec(dllexport)
    #define OMNIBOT_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef OMNIBOT_BASE_BUILDING_DLL
    #define OMNIBOT_BASE_PUBLIC OMNIBOT_BASE_EXPORT
  #else
    #define OMNIBOT_BASE_PUBLIC OMNIBOT_BASE_IMPORT
  #endif
  #define OMNIBOT_BASE_PUBLIC_TYPE OMNIBOT_BASE_PUBLIC
  #define OMNIBOT_BASE_LOCAL
#else
  #define OMNIBOT_BASE_EXPORT __attribute__ ((visibility("default")))
  #define OMNIBOT_BASE_IMPORT
  #if __GNUC__ >= 4
    #define OMNIBOT_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define OMNIBOT_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OMNIBOT_BASE_PUBLIC
    #define OMNIBOT_BASE_LOCAL
  #endif
  #define OMNIBOT_BASE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // OMNIBOT_BASE__VISIBILITY_CONTROL_H_
