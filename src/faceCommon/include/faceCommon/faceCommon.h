#ifndef FACECOMMON_H
#define FACECOMMON_H

#if (defined WIN32 || defined _WIN32 || defined WINCE)
	#if (defined FACECOMMON_COMPILING)
		#define FACECOMMON_EXPORTS __declspec(dllexport)
	#else
		#define FACECOMMON_EXPORTS __declspec(dllimport)
	#endif
#else
	#define FACECOMMON_EXPORTS
#endif

#endif // FACECOMMON_H
