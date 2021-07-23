#ifndef DATA_STORE__VISIBILITY_CONTROL_H_
#define DATA_STORE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DATA_STORE_EXPORT __attribute__ ((dllexport))
    #define DATA_STORE_IMPORT __attribute__ ((dllimport))
  #else
    #define DATA_STORE_EXPORT __declspec(dllexport)
    #define DATA_STORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DATA_STORE_BUILDING_DLL
    #define DATA_STORE_PUBLIC DATA_STORE_EXPORT
  #else
    #define DATA_STORE_PUBLIC DATA_STORE_IMPORT
  #endif
  #define DATA_STORE_PUBLIC_TYPE DATA_STORE_PUBLIC
  #define DATA_STORE_LOCAL
#else
  #define DATA_STORE_EXPORT __attribute__ ((visibility("default")))
  #define DATA_STORE_IMPORT
  #if __GNUC__ >= 4
    #define DATA_STORE_PUBLIC __attribute__ ((visibility("default")))
    #define DATA_STORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DATA_STORE_PUBLIC
    #define DATA_STORE_LOCAL
  #endif
  #define DATA_STORE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DATA_STORE__VISIBILITY_CONTROL_H_
