/*
 libpng utilities
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

#ifndef __PNGUTILS_H__
#define __PNGUTILS_H__

#define PNG_DEBUG 3
#include <png.h>
#include <cstdio>
#include <stdlib.h>

class PngUtils {
private:
    png_byte color_type;
    png_byte bit_depth;
    png_structp png_ptr;
    png_infop info_ptr;
    int number_of_passes;
public:
    int width, height;
    png_bytep * row_pointers;
    unsigned char png_loaded;

    void Clear();
    void ReadImage(int16_t* I);
    //void Read(char* file_name);
    /*
    int Write(
        char* filename,
        int width, int height,
        unsigned char *buffer, char* title);
    */
    PngUtils();
    ~PngUtils();
};

#endif

