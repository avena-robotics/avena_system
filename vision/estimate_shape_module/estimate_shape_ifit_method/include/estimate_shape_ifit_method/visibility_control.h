#ifndef IFIT_METHOD__VISIBILITY_CONTROL_H_
#define IFIT_METHOD__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IFIT_METHOD_EXPORT __attribute__ ((dllexport))
    #define IFIT_METHOD_IMPORT __attribute__ ((dllimport))
  #else
    #define IFIT_METHOD_EXPORT __declspec(dllexport)
    #define IFIT_METHOD_IMPORT __declspec(dllimport)
  #endif
  #ifdef IFIT_METHOD_BUILDING_LIBRARY
    #define IFIT_METHOD_PUBLIC IFIT_METHOD_EXPORT
  #else
    #define IFIT_METHOD_PUBLIC IFIT_METHOD_IMPORT
  #endif
  #define IFIT_METHOD_PUBLIC_TYPE IFIT_METHOD_PUBLIC
  #define IFIT_METHOD_LOCAL
#else
  #define IFIT_METHOD_EXPORT __attribute__ ((visibility("default")))
  #define IFIT_METHOD_IMPORT
  #if __GNUC__ >= 4
    #define IFIT_METHOD_PUBLIC __attribute__ ((visibility("default")))
    #define IFIT_METHOD_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IFIT_METHOD_PUBLIC
    #define IFIT_METHOD_LOCAL
  #endif
  #define IFIT_METHOD_PUBLIC_TYPE
#endif

#endif  // IFIT_METHOD__VISIBILITY_CONTROL_H_
