#ifndef ESTIMATE_SHAPE__VISIBILITY_CONTROL_H_
#define ESTIMATE_SHAPE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ESTIMATE_SHAPE_EXPORT __attribute__ ((dllexport))
    #define ESTIMATE_SHAPE_IMPORT __attribute__ ((dllimport))
  #else
    #define ESTIMATE_SHAPE_EXPORT __declspec(dllexport)
    #define ESTIMATE_SHAPE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ESTIMATE_SHAPE_BUILDING_LIBRARY
    #define ESTIMATE_SHAPE_PUBLIC ESTIMATE_SHAPE_EXPORT
  #else
    #define ESTIMATE_SHAPE_PUBLIC ESTIMATE_SHAPE_IMPORT
  #endif
  #define ESTIMATE_SHAPE_PUBLIC_TYPE ESTIMATE_SHAPE_PUBLIC
  #define ESTIMATE_SHAPE_LOCAL
#else
  #define ESTIMATE_SHAPE_EXPORT __attribute__ ((visibility("default")))
  #define ESTIMATE_SHAPE_IMPORT
  #if __GNUC__ >= 4
    #define ESTIMATE_SHAPE_PUBLIC __attribute__ ((visibility("default")))
    #define ESTIMATE_SHAPE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ESTIMATE_SHAPE_PUBLIC
    #define ESTIMATE_SHAPE_LOCAL
  #endif
  #define ESTIMATE_SHAPE_PUBLIC_TYPE
#endif

#endif  // ESTIMATE_SHAPE__VISIBILITY_CONTROL_H_
