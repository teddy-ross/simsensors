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

            static void show(
                    const int * distances_mm,
                    const double min_distance_m,
                    const double max_distance_m,
                    const int width,
                    const int height,
                    const int scaleup)
            {
                const int new_width = width * scaleup;
                const int new_height = height * scaleup;

                cv::Mat img = cv::Mat::zeros(new_height, new_width, CV_8UC1);

                for (uint8_t x=0; x<width; ++x) {

                    for (uint8_t y=0; y<height; ++y) {

                        const int d_mm = distances_mm[y * width + x];

                        const uint8_t gray = d_mm == -1 ? 255 : 
                            (uint8_t)((d_mm/1000. - min_distance_m) /
                                    (max_distance_m - min_distance_m) * 255);

                        cv::rectangle(img,
                                cv::Point(x*scaleup, y*scaleup),
                                cv::Point((x+1)*scaleup, (y+1)*scaleup),
                                gray, -1);
                    }
                }

                cv::imshow("lidar", img);

                cv::waitKey(1);
            }
    };

}
