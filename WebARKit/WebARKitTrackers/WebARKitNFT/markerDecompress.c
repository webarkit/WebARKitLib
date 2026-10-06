#include <WebARKitTrackers/WebARKitNFT/markerDecompress.h>

#ifdef _WIN32
#  include <Windows.h>
#else
#  include <sys/stat.h>
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <AR/ar.h>
#include <zlib.h>

#if MARKER_DECOMPRESS_MAX_SIZE < 1
#  error "MARKER_DECOMPRESS_MAX_SIZE must be at least 1"
#endif

static const size_t inflate_chunk = 4*1024*1024;
static const size_t inflate_max = MARKER_DECOMPRESS_MAX_SIZE;

/*
 * Inflate a whole zlib stream. On success *outLen is the decompressed size and
 * the buffer has an extra NUL after it, so it can be searched as a string.
 * Streams that expand past inflate_max are rejected, so a small archive with
 * a huge expansion ratio cannot exhaust memory.
 */
static char *inflateAll(const unsigned char *in, size_t inLen, size_t *outLen)
{
    z_stream strm;
    size_t cap = inflate_chunk < inflate_max ? inflate_chunk : inflate_max;
    char *out = malloc(cap + 1);
    int ret;

    if (out == NULL) return NULL;
    memset(&strm, 0, sizeof(strm));
    if (inflateInit(&strm) != Z_OK) {
        free(out);
        return NULL;
    }
    strm.next_in = (Bytef *)in;
    strm.avail_in = (uInt)inLen;

    do {
        if (strm.total_out == cap) {
            size_t newCap;
            char *bigger;
            if (cap >= inflate_max) {
                // At the limit: let zlib finish the stream (empty final block,
                // trailer) with the one spare byte after the buffer. Any output
                // written there means the stream is larger than the limit.
                strm.next_out = (Bytef *)(out + cap);
                strm.avail_out = 1;
                ret = inflate(&strm, Z_NO_FLUSH);
                if (strm.total_out > cap) {
                    ARLOGe("Error: .zft data expands past %zu bytes\n", inflate_max);
                    ret = Z_MEM_ERROR;
                }
                continue;
            }
            newCap = cap > inflate_max / 2 ? inflate_max : cap * 2;
            bigger = realloc(out, newCap + 1);
            if (bigger == NULL) {
                ret = Z_MEM_ERROR;
                break;
            }
            out = bigger;
            cap = newCap;
        }
        strm.next_out = (Bytef *)(out + strm.total_out);
        strm.avail_out = (uInt)(cap - strm.total_out);
        ret = inflate(&strm, Z_NO_FLUSH);
    } while (ret == Z_OK);

    if (ret != Z_STREAM_END) {
        ARLOGe("Error inflating .zft data (zlib error %d)\n", ret);
        inflateEnd(&strm);
        free(out);
        return NULL;
    }
    *outLen = strm.total_out;
    out[*outLen] = 0;
    inflateEnd(&strm);
    return out;
}

int decompressMarkers(const char* src, const char* outTemp){
    FILE *fp;
    unsigned char *in;
    char *c;
    long filesize;
    size_t outLen;

    fp = openZFT(src, "zft");
    if ( fp == NULL )
    {
        ARLOGe("Error opening .zft file\n");
        return -1;
    }

    fseek (fp, 0, SEEK_END);
    filesize = ftell (fp);
    fseek (fp, 0, SEEK_SET);
    if (filesize <= 0)
    {
        ARLOGe("Error: empty or unreadable .zft file\n");
        fclose(fp);
        return -1;
    }

    in = malloc (filesize);
    if (in == NULL)
    {
        ARLOGe("Error mallocing %ld bytes for inflate\n", filesize);
        fclose(fp);
        return -1;
    }
    if (fread (in, 1, filesize, fp) != (size_t)filesize)
    {
        ARLOGe("Error reading .zft file\n");
        fclose(fp);
        free(in);
        return -1;
    }
    fclose (fp);

    c = inflateAll(in, (size_t)filesize, &outLen);
    free(in);
    if (c == NULL) return -1;

    int result = extractDataAndSave(c, outTemp);

    free(c);
    return result;
}

/* Write one extracted marker file in binary mode; 0 on success, -1 on error. */
static int saveMarkerFile(const char *name, const char *ext, const char *data, size_t size)
{
    char *fileName = nameConcat(name, ext);
    FILE *fp;
    int ok;

    if (fileName == NULL) return -1;
    fp = fopen(fileName, "wb");
    if (fp == NULL) {
        ARLOGe("Error: cannot create %s\n", fileName);
        free(fileName);
        return -1;
    }
    ok = fwrite(data, 1, size, fp) == size;
    if (fclose(fp) != 0) ok = 0;
    if (!ok) {
        ARLOGe("Error: cannot write %s\n", fileName);
        remove(fileName);
    }
    free(fileName);
    return ok ? 0 : -1;
}

static void removeMarkerFile(const char *name, const char *ext)
{
    char *fileName = nameConcat(name, ext);
    if (fileName == NULL) return;
    remove(fileName);
    free(fileName);
}

int extractDataAndSave(const char* str, const char* name){
    // The decompressed data is: {"iset":"<iset>","fset":"<fset>","fset3":"<fset3>"}
    static const char isetKey[]  = "{\"iset\":\"";
    static const char fsetKey[]  = "\",\"fset\":\"";
    static const char fset3Key[] = "\",\"fset3\":\"";
    static const char endKey[]   = "\"}";

    if (strncmp(str, isetKey, sizeof(isetKey) - 1) != 0) {
        ARLOGe("Error: 'iset' not found at the start of the string.\n");
        return -1;
    }
    const char *iset = str + sizeof(isetKey) - 1;

    const char *fsetKeyPos = strstr(iset, fsetKey);
    if (fsetKeyPos == NULL) {
        ARLOGe("Error: 'fset' not found in the string.\n");
        return -1;
    }
    const char *fset = fsetKeyPos + sizeof(fsetKey) - 1;

    const char *fset3KeyPos = strstr(fset, fset3Key);
    if (fset3KeyPos == NULL) {
        ARLOGe("Error: 'fset3' not found in the string.\n");
        return -1;
    }
    const char *fset3 = fset3KeyPos + sizeof(fset3Key) - 1;

    const char *end = strstr(fset3, endKey);
    if (end == NULL) {
        ARLOGe("Error: end of string not found.\n");
        return -1;
    }

    // Searching each key after the previous one keeps the fields ordered.
    size_t isetSize  = (size_t)(fsetKeyPos - iset);
    size_t fsetSize  = (size_t)(fset3KeyPos - fset);
    size_t fset3Size = (size_t)(end - fset3);
    if (isetSize == 0 || fsetSize == 0 || fset3Size == 0) {
        ARLOGe("Error: empty marker field (iset %zu, fset %zu, fset3 %zu bytes).\n", isetSize, fsetSize, fset3Size);
        return -1;
    }

    if (saveMarkerFile(name, ".iset", iset, isetSize) != 0) return -1;
    if (saveMarkerFile(name, ".fset", fset, fsetSize) != 0) {
        removeMarkerFile(name, ".iset");
        return -1;
    }
    if (saveMarkerFile(name, ".fset3", fset3, fset3Size) != 0) {
        removeMarkerFile(name, ".iset");
        removeMarkerFile(name, ".fset");
        return -1;
    }
    return 0;
}

FILE *openZFT( const char *filename, const char *ext)
{
    FILE   *fp;
    char   *buf;
    size_t  len;

    if (!filename) return (NULL);
    if (ext) {
        len = strlen(filename) + strlen(ext) + 2; // space for '.' and '\0'.
        arMalloc(buf, char, len);
        sprintf(buf, "%s.%s", filename, ext);
        fp = fopen(buf,"rb");
        free(buf);
    } else {
        fp = fopen(filename,"rb");
    }

    return fp;
}

char* nameConcat(const char *s1, const char *s2)
{
    const size_t len1 = strlen(s1);
    const size_t len2 = strlen(s2);
    char *result = malloc(len1 + len2 + 1); // +1 for the null-terminator
    if (result == NULL) return NULL;
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1); // +1 to copy the null-terminator
    return result;
}