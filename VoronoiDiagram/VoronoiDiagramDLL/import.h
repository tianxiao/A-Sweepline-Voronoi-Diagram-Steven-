#ifndef VORONOIDIAGRAM_IMPORT_H
#define VORONOIDIAGRAM_IMPORT_H 

#ifndef VORONOI_DLL_EXPORT
#define R_DECLDIR __declspec(dllimport)
#else
#define R_DECLDIR __declspec(dllexport)
#endif

#endif 