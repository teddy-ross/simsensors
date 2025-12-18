/* 
   Rangefinder simulation visualizer using OpenCV

   Copyright (C) 2025 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <sensors/rangefinder.hpp>

class RangefinderVisualizer {

    public:

        RangefinderVisualizer(SimRangefinder * rangefinder)
        {
            this->rangefinder = rangefinder;
        }

        void show(const int16_t * distance_mm, const uint16_t scaleup) 
        {
            const uint16_t new_width = this->rangefinder->width * scaleup;
            const uint16_t new_height = this->rangefinder->height * scaleup;

            cv::Mat img = cv::Mat::zeros(new_height, new_width, CV_8UC1);

            const double min_distance_mm = this->rangefinder->min_distance_m * 1000;
            const double max_distance_mm = this->rangefinder->max_distance_m * 1000;

            for (uint8_t x=0; x<this->rangefinder->width; ++x) {

                for (uint8_t y=0; y<this->rangefinder->height; ++y) {

                    const double d = distance_mm[y * this->rangefinder->width + x];

                    cv::rectangle(img,
                            cv::Point(x*scaleup, y*scaleup),
                            cv::Point((x+1)*scaleup, (y+1)*scaleup),
                            d == -1 ? 255 : (uint8_t)((d-min_distance_mm) /
                                (double)(max_distance_mm - min_distance_mm) * 255), 
                            -1);
                }
            }

            cv::imshow("lidar", img);

            cv::waitKey(1);
        }

    private:

        SimRangefinder * rangefinder;
};
