#ifndef FIT_CUTTING_BOARD__VISIBILITY_CONTROL_H_
#define FIT_CUTTING_BOARD__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FIT_CUTTING_BOARD_EXPORT __attribute__ ((dllexport))
    #define FIT_CUTTING_BOARD_IMPORT __attribute__ ((dllimport))
  #else
    #define FIT_CUTTING_BOARD_EXPORT __declspec(dllexport)
    #define FIT_CUTTING_BOARD_IMPORT __declspec(dllimport)
  #endif
  #ifdef FIT_CUTTING_BOARD_BUILDING_LIBRARY
    #define FIT_CUTTING_BOARD_PUBLIC FIT_CUTTING_BOARD_EXPORT
  #else
    #define FIT_CUTTING_BOARD_PUBLIC FIT_CUTTING_BOARD_IMPORT
  #endif
  #define FIT_CUTTING_BOARD_PUBLIC_TYPE FIT_CUTTING_BOARD_PUBLIC
  #define FIT_CUTTING_BOARD_LOCAL
#else
  #define FIT_CUTTING_BOARD_EXPORT __attribute__ ((visibility("default")))
  #define FIT_CUTTING_BOARD_IMPORT
  #if __GNUC__ >= 4
    #define FIT_CUTTING_BOARD_PUBLIC __attribute__ ((visibility("default")))
    #define FIT_CUTTING_BOARD_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FIT_CUTTING_BOARD_PUBLIC
    #define FIT_CUTTING_BOARD_LOCAL
  #endif
  #define FIT_CUTTING_BOARD_PUBLIC_TYPE
#endif

#endif  // FIT_CUTTING_BOARD__VISIBILITY_CONTROL_H_
