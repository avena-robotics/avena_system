#ifndef ESTIMATE_SHAPE_MANAGER__VISIBILITY_CONTROL_H_
#define ESTIMATE_SHAPE_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ESTIMATE_SHAPE_MANAGER_EXPORT __attribute__ ((dllexport))
    #define ESTIMATE_SHAPE_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define ESTIMATE_SHAPE_MANAGER_EXPORT __declspec(dllexport)
    #define ESTIMATE_SHAPE_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ESTIMATE_SHAPE_MANAGER_BUILDING_LIBRARY
    #define ESTIMATE_SHAPE_MANAGER_PUBLIC ESTIMATE_SHAPE_MANAGER_EXPORT
  #else
    #define ESTIMATE_SHAPE_MANAGER_PUBLIC ESTIMATE_SHAPE_MANAGER_IMPORT
  #endif
  #define ESTIMATE_SHAPE_MANAGER_PUBLIC_TYPE ESTIMATE_SHAPE_MANAGER_PUBLIC
  #define ESTIMATE_SHAPE_MANAGER_LOCAL
#else
  #define ESTIMATE_SHAPE_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define ESTIMATE_SHAPE_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define ESTIMATE_SHAPE_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define ESTIMATE_SHAPE_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ESTIMATE_SHAPE_MANAGER_PUBLIC
    #define ESTIMATE_SHAPE_MANAGER_LOCAL
  #endif
  #define ESTIMATE_SHAPE_MANAGER_PUBLIC_TYPE
#endif

#endif  // ESTIMATE_SHAPE_MANAGER__VISIBILITY_CONTROL_H_
