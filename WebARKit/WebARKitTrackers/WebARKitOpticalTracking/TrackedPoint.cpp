/*
 *  TrackedPoint.cpp
 *  WebARKit
 *
 *  This file is part of WebARKit.
 *
 *  WebARKit is free software: you can redistribute it and/or modify
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
 *  Copyright 2018 Realmax, Inc.
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2010-2015 ARToolworks, Inc.
 *  Copyright 2023 WebARKit
 *
 *  Author(s): Philip Lamb, Daniel Bell, Walter Perdan
 *
 *  This code was taken from artoolkitX https://github.com/artoolkitx/artoolkitx
 *  with small modifications to adapt to existing WebARKIt code.
 *
 */

#include <WebARKitTrackers/WebARKitOpticalTracking/TrackedPoint.h>

bool TrackedPoint::IsTracking()
{
    return tracking;
}

void TrackedPoint::SetTracking(bool newTracking)
{
    tracking = newTracking;
}

bool TrackedPoint::IsSelected()
{
    return selected;
}

void TrackedPoint::SetSelected(bool newSelected)
{
    selected = newSelected;
}