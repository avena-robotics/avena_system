#ifndef HELPERS_VISION__VISIBILITY_CONTROL_H_
#define HELPERS_VISION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HELPERS_VISION_EXPORT __attribute__ ((dllexport))
    #define HELPERS_VISION_IMPORT __attribute__ ((dllimport))
  #else
    #define HELPERS_VISION_EXPORT __declspec(dllexport)
    #define HELPERS_VISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef HELPERS_VISION_BUILDING_LIBRARY
    #define HELPERS_VISION_PUBLIC HELPERS_VISION_EXPORT
  #else
    #define HELPERS_VISION_PUBLIC HELPERS_VISION_IMPORT
  #endif
  #define HELPERS_VISION_PUBLIC_TYPE HELPERS_VISION_PUBLIC
  #define HELPERS_VISION_LOCAL
#else
  #define HELPERS_VISION_EXPORT __attribute__ ((visibility("default")))
  #define HELPERS_VISION_IMPORT
  #if __GNUC__ >= 4
    #define HELPERS_VISION_PUBLIC __attribute__ ((visibility("default")))
    #define HELPERS_VISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HELPERS_VISION_PUBLIC
    #define HELPERS_VISION_LOCAL
  #endif
  #define HELPERS_VISION_PUBLIC_TYPE
#endif

#endif  // HELPERS_VISION__VISIBILITY_CONTROL_H_
