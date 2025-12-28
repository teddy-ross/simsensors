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

#include <simsensors/src/sensors/rangefinder.hpp>

namespace simsens {

    class RangefinderVisualizer {

        public:

            RangefinderVisualizer(SimRangefinder * rangefinder)
            {
                this->rangefinder = rangefinder;
            }

            void show(const int * distances_mm, const int scaleup) 
            {
                const int new_width = this->rangefinder->width * scaleup;
                const int new_height = this->rangefinder->height * scaleup;

                cv::Mat img = cv::Mat::zeros(new_height, new_width, CV_8UC1);

                for (uint8_t x=0; x<this->rangefinder->width; ++x) {

                    for (uint8_t y=0; y<this->rangefinder->height; ++y) {

                        const int d_mm = distances_mm[y * this->rangefinder->width + x];

                        cv::rectangle(img,
                                cv::Point(x*scaleup, y*scaleup),
                                cv::Point((x+1)*scaleup, (y+1)*scaleup),
                                distance_to_grayscale(d_mm), 
                                -1);
                    }
                }

                cv::imshow("lidar", img);

                cv::waitKey(1);
            }

        private:

            SimRangefinder * rangefinder;

            uint8_t distance_to_grayscale(const int d_mm)
            {
                const double dmin_m = this->rangefinder->min_distance_m;
                const double dmax_m = this->rangefinder->max_distance_m;

                return d_mm == -1 ? 255 : 
                    (uint8_t)((d_mm/1000. - dmin_m) / (double)(dmax_m - dmin_m) * 255);
            }
    };

}
