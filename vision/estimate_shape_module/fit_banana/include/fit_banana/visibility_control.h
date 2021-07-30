#ifndef FIT_BANANA__VISIBILITY_CONTROL_H_
#define FIT_BANANA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_BANANA_EXPORT __attribute__ ((dllexport))
    #define FIT_BANANA_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_BANANA_EXPORT __declspec(dllexport)
    #define FIT_BANANA_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_BANANA_BUILDING_LIBRARY
    #define FIT_BANANA_PUBLIC FIT_BANANA_EXPORT
  #else
    #define FIT_BANANA_PUBLIC FIT_BANANA_IMPORT
  #endif
  #define FIT_BANANA_PUBLIC_TYPE FIT_BANANA_PUBLIC
  #define FIT_BANANA_LOCAL
#else
  #define FIT_BANANA_EXPORT __attribute__ ((visibility("default")))
  #define FIT_BANANA_IMPORT
  #if __GNUC__ >= 4
    #define FIT_BANANA_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_BANANA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_BANANA_PUBLIC
    #define FIT_BANANA_LOCAL
  #endif
  #define FIT_BANANA_PUBLIC_TYPE
#endif

#endif  // FIT_BANANA__VISIBILITY_CONTROL_H_
