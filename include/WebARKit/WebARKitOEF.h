#ifndef WEBARKITOEF_H
#define WEBARKITOEF_H
/*
 *  WebARKitOEF.h
 *  WebARKit
 *  This file is part of WebARKit.
 *
 *  ARToolKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKit.  If not, see <http://www.gnu.org/licenses/>.
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
 *  Copyright 2022 WebARKit.org All rights reserved.
 *  Author(s): Walter Perdan (@kalwalt)
 *
 */

/*!
	@header WebARKitOEF
	@abstract OneEuroFilter to filtering signals.
	@discussion
        This header declares types and API for OEF filtering,
        this is a only header library.
  @copyright 2022 WebARKit.org
 */

 /*original code based on http://www.lifl.fr/~casiez/1euro/OneEuroFilter.cc
 *
 * OneEuroFilter.cc -
 *
 * Author: Nicolas Roussel (nicolas.roussel@inria.fr)
 *
 * Copyright 2019 Inria
 * 
 * BSD License https://opensource.org/licenses/BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <array>
#include <iostream>
#include <cmath>
#include <AR/ar.h>

namespace OEF
{
  typedef double TimeStamp ; // in seconds

  static const TimeStamp UndefinedTime = -1.0 ;

  class LowPassFilter {
    
    double y, a, s ;
    bool initialized ;

    void setAlpha(double alpha) {
      if (alpha<=0.0 || alpha>1.0) 
        webarkitLOGe("alpha should be in [0.0, 1.0]: %f", alpha);
      a = alpha ;
    }

  public:

    LowPassFilter(double alpha, double initval=0.0) {
      y = s = initval ;
      setAlpha(alpha) ;
      initialized = false ;
    }

    double filter(double value) {
      double result ;
      if (initialized)
        result = a*value + (1.0-a)*s ;
      else {
        result = value ;
        initialized = true ;
      }
      y = value ;
      s = result ;
      return result ;
    }

    double filterWithAlpha(double value, double alpha) {
      setAlpha(alpha) ;
      return filter(value) ;
    }

    bool hasLastRawValue(void) {
      return initialized ;
    }

    double lastRawValue(void) {
      return y ;
    }

  };

// -----------------------------------------------------------------

  class OneEuroFilter {

    double freq ;
    double mincutoff ;
    double beta_ ;
    double dcutoff ;
    LowPassFilter *x ;
    LowPassFilter *dx ;
    TimeStamp lasttime ;

    double alpha(double cutoff) {
      double te = 1.0 / freq ;
      double tau = 1.0 / (2*M_PI*cutoff) ;
      return 1.0 / (1.0 + tau/te) ;
    }

    void setFrequency(double f) {
      if (f<=0) webarkitLOGe("freq should be >0");
      freq = f ;
    }

    void setMinCutoff(double mc) {
      if (mc<=0) webarkitLOGe("mincutoff should be >0");
      mincutoff = mc ;
    }

    void setBeta(double b) {
      beta_ = b ;
    }

    void setDerivateCutoff(double dc) {
      if (dc<=0) webarkitLOGe("dcutoff should be >0");
      dcutoff = dc ;
    }

  public:

    OneEuroFilter(double freq, 
		  double mincutoff=1.0, double beta_=0.0, double dcutoff=1.0) {
      webarkitLOGi("mincutoff is: %f", mincutoff);
      setFrequency(freq) ;
      setMinCutoff(mincutoff) ;
      setBeta(beta_) ;
      setDerivateCutoff(dcutoff) ;
      x = new LowPassFilter(alpha(mincutoff)) ;
      dx = new LowPassFilter(alpha(dcutoff)) ;
      lasttime = UndefinedTime ;
    }

    double filter(double value, TimeStamp timestamp=UndefinedTime) {
      // update the sampling frequency based on timestamps
      webarkitLOGi("timestamp is: %d", timestamp);
      if (lasttime!=UndefinedTime && timestamp!=UndefinedTime)
        freq = 1.0 / (timestamp-lasttime) ;
      lasttime = timestamp ;
      // estimate the current variation per second 
      double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
      double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff)) ;
      // use it to update the cutoff frequency
      double cutoff = mincutoff + beta_*fabs(edvalue) ;
      // filter the given value
      return x->filterWithAlpha(value, alpha(cutoff)) ;
    }

    std::array<std::array<int, 3>, 4>  filterMat(ARdouble m[3][4], TimeStamp timestamp=UndefinedTime) {
      std::array<std::array<int, 3>, 4> out_mat;
      ARdouble q[4], p[3];
      if (arUtilMat2QuatPos((const ARdouble (*)[4])m, q, p) < 0) return {};
      arUtilQuatNorm(q);
      q[0] = filter(q[0], timestamp);
      q[1] = filter(q[1], timestamp);
      q[2] = filter(q[2], timestamp);
      q[3] = filter(q[3], timestamp);
      p[0] = filter(p[0], timestamp);
      p[1] = filter(p[1], timestamp);
      p[2] = filter(p[2], timestamp);
      if (arUtilQuatPos2Mat(q, p, m) < 0) return {};
      for(int i; i<4; i++){
        for(int j; j<3; j++){
          out_mat[i][j] =m[i][j];
        }
      }
      return out_mat;
    }

    int filterMat2(ARdouble m[3][4], ARdouble out[3][4], TimeStamp timestamp=UndefinedTime) {
      ARdouble q[4], p[3];
      if (arUtilMat2QuatPos((const ARdouble (*)[4])m, q, p) < 0) return -1;
      arUtilQuatNorm(q);
      q[0] = filter(q[0], timestamp);
      q[1] = filter(q[1], timestamp);
      q[2] = filter(q[2], timestamp);
      q[3] = filter(q[3], timestamp);
      p[0] = filter(p[0], timestamp);
      p[1] = filter(p[1], timestamp);
      p[2] = filter(p[2], timestamp);
      if (arUtilQuatPos2Mat(q, p, out) < 0) return -1;
      return 0;
    }

    ~OneEuroFilter(void) {
      delete x ;
      delete dx ;
    }

  };
};
#endif //ifndef WEBARKITOEF_H