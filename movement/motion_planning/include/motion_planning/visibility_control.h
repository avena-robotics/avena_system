#ifndef MOTION_PLANNING__VISIBILITY_CONTROL_H_
#define MOTION_PLANNING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_PLANNING_EXPORT __attribute__ ((dllexport))
    #define MOTION_PLANNING_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_PLANNING_EXPORT __declspec(dllexport)
    #define MOTION_PLANNING_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_PLANNING_BUILDING_LIBRARY
    #define MOTION_PLANNING_PUBLIC MOTION_PLANNING_EXPORT
  #else
    #define MOTION_PLANNING_PUBLIC MOTION_PLANNING_IMPORT
  #endif
  #define MOTION_PLANNING_PUBLIC_TYPE MOTION_PLANNING_PUBLIC
  #define MOTION_PLANNING_LOCAL
#else
  #define MOTION_PLANNING_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_PLANNING_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_PLANNING_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_PLANNING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_PLANNING_PUBLIC
    #define MOTION_PLANNING_LOCAL
  #endif
  #define MOTION_PLANNING_PUBLIC_TYPE
#endif

#endif  // MOTION_PLANNING__VISIBILITY_CONTROL_H_
