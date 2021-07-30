#ifndef FIT_BOWL__VISIBILITY_CONTROL_H_
#define FIT_BOWL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_BOWL_EXPORT __attribute__ ((dllexport))
    #define FIT_BOWL_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_BOWL_EXPORT __declspec(dllexport)
    #define FIT_BOWL_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_BOWL_BUILDING_LIBRARY
    #define FIT_BOWL_PUBLIC FIT_BOWL_EXPORT
  #else
    #define FIT_BOWL_PUBLIC FIT_BOWL_IMPORT
  #endif
  #define FIT_BOWL_PUBLIC_TYPE FIT_BOWL_PUBLIC
  #define FIT_BOWL_LOCAL
#else
  #define FIT_BOWL_EXPORT __attribute__ ((visibility("default")))
  #define FIT_BOWL_IMPORT
  #if __GNUC__ >= 4
    #define FIT_BOWL_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_BOWL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_BOWL_PUBLIC
    #define FIT_BOWL_LOCAL
  #endif
  #define FIT_BOWL_PUBLIC_TYPE
#endif

#endif  // FIT_PLATE__VISIBILITY_CONTROL_H_
