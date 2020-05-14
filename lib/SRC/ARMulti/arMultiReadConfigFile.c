/*
 *  arMultiReadConfigFile.c
 *  ARToolKit5
 *
 *  This file is part of ARToolKit.
 *
 *  ARToolKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ARToolKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ARToolKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2002-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */
/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 1.0
 * Date: 01/09/05
 *
 *******************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <AR/ar.h>
#include <AR/arMulti.h>

static char *get_buff( char *buf, int n, FILE *fp );

ARMultiMarkerInfoT *arMultiReadConfigFile( const char *filename, ARPattHandle *pattHandle )
{
    FILE                   *fp;
    ARMultiEachMarkerInfoT *marker;
    ARMultiMarkerInfoT     *marker_info;
    ARdouble               wpos3d[4][2];
    char                   buf[256], pattPath[2048], dummy;
    int                    num;
    int                    patt_type = 0;
    int                    i, j;

    if ((fp = fopen(filename, "r")) == NULL) {
        ARLOGe("Error: unable to open multimarker config file '%s'.\n", filename);
        ARLOGperror(NULL);
        return NULL;
    }

    get_buff(buf, 256, fp);
    if( sscanf(buf, "%d", &num) != 1 ) {
        ARLOGe("Error processing multimarker config file '%s': First line must be number of marker configs to read.\n", filename);
        fclose(fp);
        return NULL;
    }
    ARLOGd("Reading %d markers from multimarker file '%s'\n", num, filename);

    arMalloc(marker, ARMultiEachMarkerInfoT, num);

    for( i = 0; i < num; i++ ) {
        get_buff(buf, 256, fp);
        if (sscanf(buf, 
#if defined(__LP64__) && !defined(__APPLE__)
                        "%lu%c",
#else
                        "%llu%c",
#endif
                         &(marker[i].globalID), &dummy) != 1) { // Try first as matrix code.
            
            if (!pattHandle) {
                ARLOGe("Error processing multimarker config file '%s': pattern '%s' specified in multimarker configuration while in barcode-only mode.\n", filename, buf);
                goto bail;
            }
            if (!arUtilGetDirectoryNameFromPath(pattPath, filename, sizeof(pattPath), 1)) { // Get directory prefix.
                ARLOGe("Error processing multimarker config file '%s': Unable to determine directory name.\n", filename);
                goto bail;
            }
            strncat(pattPath, buf, sizeof(pattPath) - strlen(pattPath) - 1); // Add name of file to open.
            if ((marker[i].patt_id = arPattLoad(pattHandle, pattPath)) < 0) {
                ARLOGe("Error processing multimarker config file '%s': Unable to load pattern '%s'.\n", filename, pattPath);
                goto bail;
            }
            marker[i].patt_type = AR_MULTI_PATTERN_TYPE_TEMPLATE;
            patt_type |= 0x01;
        } else {
            
            if ((marker[i].globalID & 0xffff8000ULL) == 0ULL) marker[i].patt_id = (int)(marker[i].globalID & 0x00007fffULL); // If upper 33 bits are zero, use lower 31 bits as regular matrix code.
            else marker[i].patt_id = 0;
            ARLOGd("Marker %3d is matrix code %llu.\n", i + 1, marker[i].globalID);
            marker[i].patt_type = AR_MULTI_PATTERN_TYPE_MATRIX;
            patt_type |= 0x02;
        }

        get_buff(buf, 256, fp);
        if( sscanf(buf,
#ifdef ARDOUBLE_IS_FLOAT
                   "%f",
#else
                   "%lf",
#endif
                   &marker[i].width) != 1 ) {
            ARLOGe("Error processing multimarker config file '%s', marker definition %3d: First line must be pattern width.\n", filename, i + 1);
            goto bail;
        }
        
        j = 0;
        get_buff(buf, 256, fp);
        if( sscanf(buf,
#ifdef ARDOUBLE_IS_FLOAT
                   "%f %f %f %f",
#else
                   "%lf %lf %lf %lf",
#endif
                   &marker[i].trans[j][0],
                   &marker[i].trans[j][1],
                   &marker[i].trans[j][2],
                   &marker[i].trans[j][3]) != 4 ) {
            // Perhaps this is an old ARToolKit v2.x multimarker file?
            // If so, then the next line is two values (center) and should be skipped.
            float t1, t2;
            if( sscanf(buf,
                       "%f %f",
                       &t1, &t2) != 2 ) {
                ARLOGe("Error processing multimarker config file '%s', marker definition %3d: Lines 2 - 4 must be marker transform.\n", filename, i + 1);
                goto bail;
            }
        } else j++;
        do {
            get_buff(buf, 256, fp);
            if( sscanf(buf, 
#ifdef ARDOUBLE_IS_FLOAT
                       "%f %f %f %f",
#else
                       "%lf %lf %lf %lf",
#endif
                       &marker[i].trans[j][0],
                       &marker[i].trans[j][1],
                       &marker[i].trans[j][2],
                       &marker[i].trans[j][3]) != 4 ) {
                ARLOGe("Error processing multimarker config file '%s', marker definition %3d: Lines 2 - 4 must be marker transform.\n", filename, i + 1);
                goto bail;
            }
            j++;
        } while (j < 3);
        arUtilMatInv( (const ARdouble (*)[4])marker[i].trans, marker[i].itrans );

        wpos3d[0][0] =  -marker[i].width/2.0;
        wpos3d[0][1] =   marker[i].width/2.0;
        wpos3d[1][0] =   marker[i].width/2.0;
        wpos3d[1][1] =   marker[i].width/2.0;
        wpos3d[2][0] =   marker[i].width/2.0;
        wpos3d[2][1] =  -marker[i].width/2.0;
        wpos3d[3][0] =  -marker[i].width/2.0;
        wpos3d[3][1] =  -marker[i].width/2.0;
        for( j = 0; j < 4; j++ ) {
            marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0]
                                  + marker[i].trans[0][1] * wpos3d[j][1]
                                  + marker[i].trans[0][3];
            marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0]
                                  + marker[i].trans[1][1] * wpos3d[j][1]
                                  + marker[i].trans[1][3];
            marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0]
                                  + marker[i].trans[2][1] * wpos3d[j][1]
                                  + marker[i].trans[2][3];
        }
    }

    fclose(fp);

    arMalloc(marker_info, ARMultiMarkerInfoT, 1);
    marker_info->marker     = marker;
    marker_info->marker_num = num;
    marker_info->min_submarker = 0;
    marker_info->prevF      = 0;
    if( (patt_type & 0x03) == 0x03 ) marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE_AND_MATRIX;
    else if( patt_type & 0x01 )    marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE;
    else                           marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_MATRIX;
    marker_info->cfPattCutoff = AR_MULTI_CONFIDENCE_PATTERN_CUTOFF_DEFAULT;
    marker_info->cfMatrixCutoff = AR_MULTI_CONFIDENCE_MATRIX_CUTOFF_DEFAULT;

    return marker_info;
    
bail:
    fclose(fp);
    free(marker);
    return NULL;
}

static char *get_buff(char *buf, int n, FILE *fp)
{
    char *ret;
    size_t l;
    
    do {
        ret = fgets(buf, n, fp);
        if (ret == NULL) return (NULL); // EOF or error.
        
        // Remove NLs and CRs from end of string.
        l = strlen(buf);
        while (l > 0) {
            if (buf[l - 1] != '\n' && buf[l - 1] != '\r') break;
            l--;
            buf[l] = '\0';
        }
    } while (buf[0] == '#' || buf[0] == '\0'); // Reject comments and blank lines.
    
    return (ret);
}
