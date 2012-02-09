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

#include "matchimages.h"

MatchImages::MatchImages()
{
    M=NULL;
    I1=I2=NULL;
    left_image_matches = right_image_matches = NULL;
    image_width = image_height = 0;
}

MatchImages::~MatchImages()
{
    if (M!=NULL) {
        delete M;
    }
    if (I1!=NULL) {
        free(I1);
        free(I2);
        free(left_image_matches);
        free(right_image_matches);
    }
}

void MatchImages::match_features (
    int32_t max_features,
    double  bucket_width,
    double  bucket_height)
{
    const int32_t dims[] = { image_width, image_height };

    // Create matcher
    if (M == NULL) {
        M = new Matcher();
    }

    // Locate features
    M->computeFeaturesFromImagePair(I1, I2, dims);

    // Match
    M->matchFeatures();
    
    // remove closeby features via bucketing
    M->bucketFeatures(max_features,bucket_width,bucket_height);    
}

vector<Matcher::p_match>* MatchImages::getMatches()
{
    return M->getMatches();
}

void MatchImages::draw_matches(
    unsigned char * image_data,
    std::vector<int> inliers)
{
    int i=0,n=0, r=0,g=255,b=0,x0,y0,x1,y1;
    int r2=0,g2=0,b2=0;
    for (y0=0;y0<image_height;y0++) {
        for (x0=0;x0<image_width;x0++,n++) {
            image_data[n*3] = I1[n]/3;
            image_data[n*3+1] = I1[n]/3;
            image_data[n*3+2] = I1[n]/3;
        }
    }

    n=0;
    for (vector<Matcher::p_match>::iterator it=M->getMatches()->begin();
        it!=M->getMatches()->end(); it++, n++) {
        if (n == inliers[i]) {
            i++;
            x0 = it->u1c;
            y0 = it->v1c;
            x1 = it->u1p;
            y1 = it->v1p;
            Drawing::line(
                image_width, image_height, image_data,
                x0, y0, x1, y1, r, g, b,1);            
            switch(n%7) {
                case 0: { r2=255; g2=0; b2=0; break; }
                case 1: { r2=0; g2=255; b2=0; break; }
                case 2: { r2=0; g2=0; b2=255; break; }
                case 3: { r2=255; g2=0; b2=255; break; }
                case 4: { r2=0; g2=255; b2=255; break; }
                case 5: { r2=255; g2=255; b2=0; break; }
                case 6: { r2=255; g2=255; b2=255; break; }
            }
            Drawing::cross(
                image_width, image_height, image_data,
                x0, y0, r2,g2,b2,4,0);
        }
    }
}

void MatchImages::draw_matches(
    unsigned char * image_data,
    int16_t * I,
    int right_image)
{
    int n=0,r=0,g=255,b=0,x0,y0,x1,y1;
    int r2=0,g2=0,b2=0;
    for (y0=0;y0<image_height;y0++) {
        for (x0=0;x0<image_width;x0++,n++) {
            image_data[n*3] = I[n]/3;
            image_data[n*3+1] = I[n]/3;
            image_data[n*3+2] = I[n]/3;
        }
    }
    n=0;
    for (vector<Matcher::p_match>::iterator it=M->getMatches()->begin();
        it!=M->getMatches()->end(); it++, n++) {
        if (right_image==0) {
            x0 = it->u1c;
            y0 = it->v1c;
            x1 = it->u1p;
            y1 = it->v1p;
        }
        else {
            x0 = it->u2c;
            y0 = it->v2c;
            x1 = it->u2p;
            y1 = it->v2p;
        }

        Drawing::line(
            image_width, image_height, image_data,
            x0, y0, x1, y1, r, g, b,1);

        switch(n%7) {
            case 0: { r2=255; g2=0; b2=0; break; }
            case 1: { r2=0; g2=255; b2=0; break; }
            case 2: { r2=0; g2=0; b2=255; break; }
            case 3: { r2=255; g2=0; b2=255; break; }
            case 4: { r2=0; g2=255; b2=255; break; }
            case 5: { r2=255; g2=255; b2=0; break; }
            case 6: { r2=255; g2=255; b2=255; break; }
        }
        Drawing::cross(
            image_width, image_height, image_data,
            x0, y0, r2,g2,b2,4,0);
    }
}

void MatchImages::pad_zeros(
    int n,
    int zeros,
    char * str)
{
    int i=1,j,thresh=10,ctr=0;
    char n_str[10];

    sprintf((char*)n_str,"%d",n);

    while (i<zeros) {
        if (n<thresh) {
            ctr++;
        }
        thresh*=10;
        i++;
    }
    for (i=0;i<ctr;i++) {
        str[i]='0';
    }
    j=0;
    while (n_str[j]!=0) {
        str[i++]=n_str[j++];
    }
    str[i]=0;
}

int MatchImages::match_images(
    char * left_image_filename,
    char * right_image_filename,
    int v_shift,
    Rectify * rectification,
    int32_t max_features,
    double  bucket_width,
    double  bucket_height,
    int frame_index)
{
    FILE * file;
    int success=0;
    char str[256];
    char number_str[7];
    PngUtils png;

    if ((file = fopen(left_image_filename,"r"))) {
        fclose(file);
        if ((file = fopen(right_image_filename,"r"))) {
            fclose(file);
            png.Read(left_image_filename);
            image_width = png.width;
            image_height = png.height;
            if (I1 == NULL) {
                I1 = (int16_t*)malloc(image_width*image_height*sizeof(int16_t));
                I2 = (int16_t*)malloc(image_width*image_height*sizeof(int16_t));
                left_image_matches = (unsigned char*)malloc(image_width*image_height*3);
                right_image_matches = (unsigned char*)malloc(image_width*image_height*3);
            }
            png.ReadImage(I1);
            png.Clear();

            png.Read(right_image_filename);
            if ((png.width==image_width) && (png.height==image_height)) {
                png.ReadImage(I2);
                png.Clear();

                rectification[0].update(image_width, image_height,I1,-v_shift);
                rectification[1].update(image_width, image_height,I2,+v_shift);

                match_features(max_features, bucket_width, bucket_height);

                if (frame_index>-1) {
                    draw_matches(left_image_matches, I1, 0);
                    draw_matches(right_image_matches, I2, 1);

                    MatchImages::pad_zeros(frame_index,6,number_str);
                    sprintf((char*)str,"matches_left_%s.png", number_str);
                    png.Write(
                        (char*)str, image_width, image_height,
                        left_image_matches, NULL);
                    sprintf((char*)str,"matches_right_%s.png", number_str);
                    png.Write(
                        (char*)str, image_width, image_height,
                        right_image_matches, NULL);
                }
                success=1;
            }
            else {
                png.Clear();
                printf("Images are not the same size\n");
            }
        }
        else {
            printf("Image not found\n%s\n",right_image_filename);
        }
    }
    else {
        printf("Image not found\n%s\n",left_image_filename);
    }
    return success;
}


void MatchImages::match_images(
    int image_width,
    int image_height,
    unsigned char * left_image,
    unsigned char * right_image,
    int32_t max_features,
    double  bucket_width,
    double  bucket_height)
{
    int i,n=0;
    this->image_width = image_width;
    this->image_height = image_height;
    if (I1 == NULL) {
        I1 = (int16_t*)malloc(image_width*image_height*sizeof(int16_t));
        I2 = (int16_t*)malloc(image_width*image_height*sizeof(int16_t));
    }

    for (i = 0; i < image_width*image_height*3; i += 3, n++) {
        I1[n] = left_image[i] + left_image[i+1] + left_image[i+2];
        I2[n] = right_image[i] + right_image[i+1] + right_image[i+2];
    }

    match_features(max_features, bucket_width, bucket_height);
}

