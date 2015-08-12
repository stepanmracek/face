#ifndef REALSENSE_H
#define REALSENSE_H

#if (defined WIN32 || defined _WIN32 || defined WINCE)
	#if (defined REALSENSE_COMPILING)
		#define REALSENSE_EXPORTS __declspec(dllexport)
	#else
		#define REALSENSE_EXPORTS __declspec(dllimport)
	#endif
#else
	#define REALSENSE_EXPORTS
#endif

#endif // REALSENSE_H
