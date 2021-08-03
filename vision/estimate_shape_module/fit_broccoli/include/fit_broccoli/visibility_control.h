#ifndef FIT_BROCCOLI__VISIBILITY_CONTROL_H_
#define FIT_BROCCOLI__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_BROCCOLI_EXPORT __attribute__ ((dllexport))
    #define FIT_BROCCOLI_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_BROCCOLI_EXPORT __declspec(dllexport)
    #define FIT_BROCCOLI_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_BROCCOLI_BUILDING_LIBRARY
    #define FIT_BROCCOLI_PUBLIC FIT_BROCCOLI_EXPORT
  #else
    #define FIT_BROCCOLI_PUBLIC FIT_BROCCOLI_IMPORT
  #endif
  #define FIT_BROCCOLI_PUBLIC_TYPE FIT_BROCCOLI_PUBLIC
  #define FIT_BROCCOLI_LOCAL
#else
  #define FIT_BROCCOLI_EXPORT __attribute__ ((visibility("default")))
  #define FIT_BROCCOLI_IMPORT
  #if __GNUC__ >= 4
    #define FIT_BROCCOLI_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_BROCCOLI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_BROCCOLI_PUBLIC
    #define FIT_BROCCOLI_LOCAL
  #endif
  #define FIT_BROCCOLI_PUBLIC_TYPE
#endif

#endif  // FIT_BROCCOLI__VISIBILITY_CONTROL_H_
