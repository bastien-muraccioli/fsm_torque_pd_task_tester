#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define FSMTorquePDTaskTester_DLLIMPORT __declspec(dllimport)
#  define FSMTorquePDTaskTester_DLLEXPORT __declspec(dllexport)
#  define FSMTorquePDTaskTester_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define FSMTorquePDTaskTester_DLLIMPORT __attribute__((visibility("default")))
#    define FSMTorquePDTaskTester_DLLEXPORT __attribute__((visibility("default")))
#    define FSMTorquePDTaskTester_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define FSMTorquePDTaskTester_DLLIMPORT
#    define FSMTorquePDTaskTester_DLLEXPORT
#    define FSMTorquePDTaskTester_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef FSMTorquePDTaskTester_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define FSMTorquePDTaskTester_DLLAPI
#  define FSMTorquePDTaskTester_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef FSMTorquePDTaskTester_EXPORTS
#    define FSMTorquePDTaskTester_DLLAPI FSMTorquePDTaskTester_DLLEXPORT
#  else
#    define FSMTorquePDTaskTester_DLLAPI FSMTorquePDTaskTester_DLLIMPORT
#  endif // FSMTorquePDTaskTester_EXPORTS
#  define FSMTorquePDTaskTester_LOCAL FSMTorquePDTaskTester_DLLLOCAL
#endif // FSMTorquePDTaskTester_STATIC
