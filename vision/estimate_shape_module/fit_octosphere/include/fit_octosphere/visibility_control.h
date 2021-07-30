#ifndef FIT_OCTOSPHERE__VISIBILITY_CONTROL_H_
#define FIT_OCTOSPHERE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_OCTOSPHERE_EXPORT __attribute__ ((dllexport))
    #define FIT_OCTOSPHERE_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_OCTOSPHERE_EXPORT __declspec(dllexport)
    #define FIT_OCTOSPHERE_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_OCTOSPHERE_BUILDING_LIBRARY
    #define FIT_OCTOSPHERE_PUBLIC FIT_OCTOSPHERE_EXPORT
  #else
    #define FIT_OCTOSPHERE_PUBLIC FIT_OCTOSPHERE_IMPORT
  #endif
  #define FIT_OCTOSPHERE_PUBLIC_TYPE FIT_OCTOSPHERE_PUBLIC
  #define FIT_OCTOSPHERE_LOCAL
#else
  #define FIT_OCTOSPHERE_EXPORT __attribute__ ((visibility("default")))
  #define FIT_OCTOSPHERE_IMPORT
  #if __GNUC__ >= 4
    #define FIT_OCTOSPHERE_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_OCTOSPHERE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_OCTOSPHERE_PUBLIC
    #define FIT_OCTOSPHERE_LOCAL
  #endif
  #define FIT_OCTOSPHERE_PUBLIC_TYPE
#endif

#endif  // FIT_OCTOSPHERE__VISIBILITY_CONTROL_H_
