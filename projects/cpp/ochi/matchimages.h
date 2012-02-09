/*
 Match a pair of images
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

#ifndef __MATCHIMAGES_H__
#define __MATCHIMAGES_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <vector>
#include "matcher.h"
#include "pngUtils.h"
#include "drawing.h"
#include "rectify.h"

using namespace std;

class MatchImages {
private:
    Matcher *M;
    unsigned char * left_image_matches;
    unsigned char * right_image_matches;
    int image_width, image_height;

    void png_free();
    void read_image(
        int16_t* I);
public:
    int16_t * I1, * I2;
    vector<Matcher::p_match>* getMatches();
    static void pad_zeros(
        int n,
        int zeros,
        char * str);
    void draw_matches(
        unsigned char * image_data,
        int16_t * I,
        int right_image);
    void draw_matches(
        unsigned char * image_data,
        std::vector<int> inliers);
    void match_images(
        int image_width,
        int image_height,
        unsigned char * left_image,
        unsigned char * right_image,
        int32_t max_features,
        double  bucket_width,
        double  bucket_height);
    int match_images(
        char * left_image_filename,
        char * right_image_filename,
        int v_shift,
        Rectify * rectification,        
        int32_t max_features,
        double  bucket_width,
        double  bucket_height,
        int frame_index);
    void match_features(
        int32_t max_features,
        double  bucket_width,
        double  bucket_height);

    MatchImages();
    ~MatchImages();
};

#endif

