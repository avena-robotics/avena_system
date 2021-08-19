#ifndef OCTOMAP_FILTER__VISIBILITY_CONTROL_H_
#define OCTOMAP_FILTER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OCTOMAP_FILTER_EXPORT __attribute__ ((dllexport))
    #define OCTOMAP_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define OCTOMAP_FILTER_EXPORT __declspec(dllexport)
    #define OCTOMAP_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef OCTOMAP_FILTER_BUILDING_DLL
    #define OCTOMAP_FILTER_PUBLIC OCTOMAP_FILTER_EXPORT
  #else
    #define OCTOMAP_FILTER_PUBLIC OCTOMAP_FILTER_IMPORT
  #endif
  #define OCTOMAP_FILTER_PUBLIC_TYPE OCTOMAP_FILTER_PUBLIC
  #define OCTOMAP_FILTER_LOCAL
#else
  #define OCTOMAP_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define OCTOMAP_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define OCTOMAP_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define OCTOMAP_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OCTOMAP_FILTER_PUBLIC
    #define OCTOMAP_FILTER_LOCAL
  #endif
  #define OCTOMAP_FILTER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // OCTOMAP_FILTER__VISIBILITY_CONTROL_H_
