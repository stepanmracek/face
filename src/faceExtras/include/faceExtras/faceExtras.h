#ifndef FACEEXTRAS_H
#define FACEEXTRAS_H

#if (defined WIN32 || defined _WIN32 || defined WINCE)
	#if (defined FACEEXTRAS_COMPILING)
		#define FACEEXTRAS_EXPORTS __declspec(dllexport)
	#else
		#define FACEEXTRAS_EXPORTS __declspec(dllimport)
	#endif
#else
	#define FACEEXTRAS_EXPORTS
#endif

#endif // FACEEXTRAS_H
