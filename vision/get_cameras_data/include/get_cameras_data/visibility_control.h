#ifndef GET_CAMERAS_DATA__VISIBILITY_CONTROL_H_
#define GET_CAMERAS_DATA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GET_CAMERAS_DATA_EXPORT __attribute__ ((dllexport))
    #define GET_CAMERAS_DATA_IMPORT __attribute__ ((dllimport))
  #else
    #define GET_CAMERAS_DATA_EXPORT __declspec(dllexport)
    #define GET_CAMERAS_DATA_IMPORT __declspec(dllimport)
  #endif
  #ifdef GET_CAMERAS_DATA_BUILDING_LIBRARY
    #define GET_CAMERAS_DATA_PUBLIC GET_CAMERAS_DATA_EXPORT
  #else
    #define GET_CAMERAS_DATA_PUBLIC GET_CAMERAS_DATA_IMPORT
  #endif
  #define GET_CAMERAS_DATA_PUBLIC_TYPE GET_CAMERAS_DATA_PUBLIC
  #define GET_CAMERAS_DATA_LOCAL
#else
  #define GET_CAMERAS_DATA_EXPORT __attribute__ ((visibility("default")))
  #define GET_CAMERAS_DATA_IMPORT
  #if __GNUC__ >= 4
    #define GET_CAMERAS_DATA_PUBLIC __attribute__ ((visibility("default")))
    #define GET_CAMERAS_DATA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GET_CAMERAS_DATA_PUBLIC
    #define GET_CAMERAS_DATA_LOCAL
  #endif
  #define GET_CAMERAS_DATA_PUBLIC_TYPE
#endif

#endif  // GET_CAMERAS_DATA__VISIBILITY_CONTROL_H_
