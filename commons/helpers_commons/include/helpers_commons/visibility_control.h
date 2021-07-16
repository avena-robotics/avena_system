#ifndef HELPERS_COMMONS__VISIBILITY_CONTROL_H_
#define HELPERS_COMMONS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HELPERS_COMMONS_EXPORT __attribute__ ((dllexport))
    #define HELPERS_COMMONS_IMPORT __attribute__ ((dllimport))
  #else
    #define HELPERS_COMMONS_EXPORT __declspec(dllexport)
    #define HELPERS_COMMONS_IMPORT __declspec(dllimport)
  #endif
  #ifdef HELPERS_COMMONS_BUILDING_LIBRARY
    #define HELPERS_COMMONS_PUBLIC HELPERS_COMMONS_EXPORT
  #else
    #define HELPERS_COMMONS_PUBLIC HELPERS_COMMONS_IMPORT
  #endif
  #define HELPERS_COMMONS_PUBLIC_TYPE HELPERS_COMMONS_PUBLIC
  #define HELPERS_COMMONS_LOCAL
#else
  #define HELPERS_COMMONS_EXPORT __attribute__ ((visibility("default")))
  #define HELPERS_COMMONS_IMPORT
  #if __GNUC__ >= 4
    #define HELPERS_COMMONS_PUBLIC __attribute__ ((visibility("default")))
    #define HELPERS_COMMONS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HELPERS_COMMONS_PUBLIC
    #define HELPERS_COMMONS_LOCAL
  #endif
  #define HELPERS_COMMONS_PUBLIC_TYPE
#endif

#endif  // HELPERS_COMMONS__VISIBILITY_CONTROL_H_
