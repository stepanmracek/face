#ifndef SOFTKINETIC_H
#define SOFTKINETIC_H

#if (defined WIN32 || defined _WIN32 || defined WINCE)
#if (defined SKAPI_EXPORTS)
#  define SOFTKINETIC_EXPORTS __declspec(dllexport)
#else
#  define SOFTKINETIC_EXPORTS __declspec(dllimport)
#endif
#endif

#if !defined(SOFTKINETIC_EXPORTS)
#define SOFTKINETIC_EXPORTS
#endif

#endif // SOFTKINETIC_H
