#ifndef FIT_SPATULA__VISIBILITY_CONTROL_H_
#define FIT_SPATULA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_SPATULA_EXPORT __attribute__ ((dllexport))
    #define FIT_SPATULA_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_SPATULA_EXPORT __declspec(dllexport)
    #define FIT_SPATULA_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_SPATULA_BUILDING_LIBRARY
    #define FIT_SPATULA_PUBLIC FIT_SPATULA_EXPORT
  #else
    #define FIT_SPATULA_PUBLIC FIT_SPATULA_IMPORT
  #endif
  #define FIT_SPATULA_PUBLIC_TYPE FIT_SPATULA_PUBLIC
  #define FIT_SPATULA_LOCAL
#else
  #define FIT_SPATULA_EXPORT __attribute__ ((visibility("default")))
  #define FIT_SPATULA_IMPORT
  #if __GNUC__ >= 4
    #define FIT_SPATULA_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_SPATULA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_SPATULA_PUBLIC
    #define FIT_SPATULA_LOCAL
  #endif
  #define FIT_SPATULA_PUBLIC_TYPE
#endif

#endif  // FIT_SPATULA__VISIBILITY_CONTROL_H_
