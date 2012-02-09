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

#include "drawing.h"

void Drawing::line(
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
    int line_width)
{
    int length,i,x,y,n,offset;
    int dx = bx-tx;
    int dy = by-ty;
    if (abs(dx)>abs(dy)) {
        length = abs(dx);
        for (offset = -line_width;offset<=line_width;offset++) {
            for (i=0;i<length;i++) {
                x = tx + i*dx/length;
                y = ty + offset + i*dy/length;
                if ((x>0) && (x < image_width) &&
                    (y>0) && (y < image_height)) {
                    n = (y*image_width+x)*3;
                    image_data[n] = r;
                    image_data[n+1] = g;
                    image_data[n+2] = b;
                }
            }
        }
    }
    else {
        length = abs(dy);
        for (offset = -line_width;offset<=line_width;offset++) {
            for (i=0;i<length;i++) {
                x = tx + offset + i*dx/length;
                y = ty + i*dy/length;
                if ((x>0) && (x < image_width) &&
                    (y>0) && (y < image_height)) {
                    n = (y*image_width+x)*3;
                    image_data[n] = r;
                    image_data[n+1] = g;
                    image_data[n+2] = b;
                }
            }
        }
    }
}

void Drawing::cross(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int x,
    int y,
    int r,
    int g,
    int b,
    int radius,
    int line_width)
{
    Drawing::line(
        image_width, image_height, image_data,
        x-radius, y, x+radius, y,
        r, g, b, line_width);
    Drawing::line(
        image_width, image_height, image_data,
        x, y-radius, x, y+radius,
        r, g, b, line_width);
}

void Drawing::rectangle(
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
    double rotation)
{
    int vertex, prev_vertex;
    double xx[4],yy[4];
    xx[0] = x - (width / 2);
    yy[0] = y - (length / 2);

    double dx = x - xx[0];
    double dy = y - yy[0];
    double dist = sqrt(dx*dx + dy*dy);
    double angle = atan2(dx,dy)+rotation;
    xx[0] = x + dist*sin(angle);
    yy[0] = y + dist*cos(angle);

    angle = atan2(-dx,dy)+rotation;
    xx[1] = x + dist*sin(angle);
    yy[1] = y + dist*cos(angle);

    angle = atan2(-dx,-dy)+rotation;
    xx[2] = x + dist*sin(angle);
    yy[2] = y + dist*cos(angle);

    angle = atan2(dx,-dy)+rotation;
    xx[3] = x + dist*sin(angle);
    yy[3] = y + dist*cos(angle);

    for (vertex=0;vertex<4;vertex++) {
        prev_vertex = vertex-1;
        if (prev_vertex<0) prev_vertex=3;
        Drawing::line(
            image_width, image_height, image_data,
            xx[vertex], yy[vertex],
            xx[prev_vertex], yy[prev_vertex],
            r, g, b, line_width);
    }
}
