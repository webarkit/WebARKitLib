#ifndef MARKER_DECOMPRESS_H
#define MARKER_DECOMPRESS_H

#include <stdio.h>

/* Largest decompressed .zft accepted by decompressMarkers(); larger archives are rejected. */
#ifndef MARKER_DECOMPRESS_MAX_SIZE
#define MARKER_DECOMPRESS_MAX_SIZE (128u * 1024u * 1024u)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    char *iset_content;   
    char *fset_content;    
    char *fset3_content; 
} markerContentStruct;

char* nameConcat(const char *s1, const char *s2);
FILE *openZFT( const char *filename, const char *ext);
/*
 * Unpack <src>.zft into <outTemp>.iset, <outTemp>.fset and <outTemp>.fset3.
 * Returns 0 on success, -1 on a missing, malformed or oversized archive or a
 * write error. On failure no output file is left behind, so <outTemp> should
 * name new files: an existing marker set at that path is not preserved.
 */
int decompressMarkers(const char* src, const char* outTemp);
int extractDataAndSave(const char* str, const char* name);

#ifdef __cplusplus
}
#endif

#endif // MARKER_DECOMPRESS_H
