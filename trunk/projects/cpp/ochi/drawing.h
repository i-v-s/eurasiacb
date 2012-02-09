/*
 Drawing functions
 Copyright 2010. All rights reserved.
 Bob Mottram <fuzzgun@gmail.com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __DRAWING_H__
#define __DRAWING_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

class Drawing {
public:
    static void line(
        int image_width,
        int image_height,
        unsigned char * image_data,
        int tx,
        int ty,
        int bx,
        int by,
        int r,
        int g,
        int b,
        int line_width);
    static void cross(
        int image_width,
        int image_height,
        unsigned char * image_data,
        int x,
        int y,
        int r,
        int g,
        int b,
        int radius,
        int line_width);
    static void rectangle(
        int image_width,
        int image_height,
        unsigned char * image_data,
        int x,
        int y,
        int r,
        int g,
        int b,
        int line_width,
        int width,
        int length,
        double rotation);
};

#endif

