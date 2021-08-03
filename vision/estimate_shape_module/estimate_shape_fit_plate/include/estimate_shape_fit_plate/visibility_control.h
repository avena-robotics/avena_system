#ifndef FIT_PLATE__VISIBILITY_CONTROL_H_
#define FIT_PLATE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_PLATE_EXPORT __attribute__ ((dllexport))
    #define FIT_PLATE_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_PLATE_EXPORT __declspec(dllexport)
    #define FIT_PLATE_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_PLATE_BUILDING_LIBRARY
    #define FIT_PLATE_PUBLIC FIT_PLATE_EXPORT
  #else
    #define FIT_PLATE_PUBLIC FIT_PLATE_IMPORT
  #endif
  #define FIT_PLATE_PUBLIC_TYPE FIT_PLATE_PUBLIC
  #define FIT_PLATE_LOCAL
#else
  #define FIT_PLATE_EXPORT __attribute__ ((visibility("default")))
  #define FIT_PLATE_IMPORT
  #if __GNUC__ >= 4
    #define FIT_PLATE_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_PLATE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_PLATE_PUBLIC
    #define FIT_PLATE_LOCAL
  #endif
  #define FIT_PLATE_PUBLIC_TYPE
#endif

#endif  // FIT_PLATE__VISIBILITY_CONTROL_H_
