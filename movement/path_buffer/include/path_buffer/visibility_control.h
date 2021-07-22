#ifndef PATH_BUFFER__VISIBILITY_CONTROL_H_
#define PATH_BUFFER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PATH_BUFFER_EXPORT __attribute__ ((dllexport))
    #define PATH_BUFFER_IMPORT __attribute__ ((dllimport))
  #else
    #define PATH_BUFFER_EXPORT __declspec(dllexport)
    #define PATH_BUFFER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PATH_BUFFER_BUILDING_DLL
    #define PATH_BUFFER_PUBLIC PATH_BUFFER_EXPORT
  #else
    #define PATH_BUFFER_PUBLIC PATH_BUFFER_IMPORT
  #endif
  #define PATH_BUFFER_PUBLIC_TYPE PATH_BUFFER_PUBLIC
  #define PATH_BUFFER_LOCAL
#else
  #define PATH_BUFFER_EXPORT __attribute__ ((visibility("default")))
  #define PATH_BUFFER_IMPORT
  #if __GNUC__ >= 4
    #define PATH_BUFFER_PUBLIC __attribute__ ((visibility("default")))
    #define PATH_BUFFER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PATH_BUFFER_PUBLIC
    #define PATH_BUFFER_LOCAL
  #endif
  #define PATH_BUFFER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // PATH_BUFFER__VISIBILITY_CONTROL_H_
