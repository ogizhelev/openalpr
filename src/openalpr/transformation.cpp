/*
 * Copyright (c) 2015 OpenALPR Technology, Inc.
 * Open source Automated License Plate Recognition [http://www.openalpr.com]
 *
 * This file is part of OpenALPR.
 *
 * OpenALPR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License
 * version 3 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "transformation.h"

using namespace std;
using namespace cv;

namespace alpr
{

  Transformation::Transformation(Mat bigImage, Mat smallImage, Rect regionInBigImage) {
    this->bigImage = bigImage;
    this->smallImage = smallImage;
    this->regionInBigImage = regionInBigImage;
  }


  Transformation::~Transformation() {
  }

  // Re-maps the coordinates from the smallImage to the coordinate space of the bigImage.
  vector<Point2f> Transformation::transformSmallPointsToBigImage(vector<Point> points)
  {
    vector<Point2f> floatPoints;
    for (unsigned int i = 0; i < points.size(); i++)
      floatPoints.push_back(points[i]);

    return transformSmallPointsToBigImage(floatPoints);

  }

  // Re-maps the coordinates from the smallImage to the coordinate space of the bigImage.
  vector<Point2f> Transformation::transformSmallPointsToBigImage(vector<Point2f> points)
  {
    vector<Point2f> bigPoints;
    for (unsigned int i = 0; i < points.size(); i++)
    {
      float bigX = (points[i].x * ((float) regionInBigImage.width / smallImage.cols));
      float bigY = (points[i].y * ((float) regionInBigImage.height / smallImage.rows));

      bigX = bigX + regionInBigImage.x;
      bigY = bigY + regionInBigImage.y;

      bigPoints.push_back(Point2f(bigX, bigY));
    }

    return bigPoints;
  }


  Mat Transformation::getTransformationMatrix(vector<Point2f> corners, Size outputImageSize)
  {
    // Corners of the destination image
    vector<Point2f> quad_pts;
    quad_pts.push_back(Point2f(0, 0));
    quad_pts.push_back(Point2f(outputImageSize.width, 0));
    quad_pts.push_back(Point2f(outputImageSize.width, outputImageSize.height));
    quad_pts.push_back(Point2f(0, outputImageSize.height));

    return getTransformationMatrix(corners, quad_pts);
  }

  Mat Transformation::getTransformationMatrix(vector<Point2f> corners, vector<Point2f> outputCorners)
  {

    // Get transformation matrix
    Mat transmtx = getPerspectiveTransform(corners, outputCorners);

    return transmtx;
  }


  Mat Transformation::crop(Size outputImageSize, Mat transformationMatrix)
  {


    Mat deskewed(outputImageSize, this->bigImage.type());

    // Apply perspective transformation to the image
    warpPerspective(this->bigImage, deskewed, transformationMatrix, deskewed.size(), INTER_CUBIC);

    return deskewed;
  }

  std::vector<cv::Point2f> Transformation::findPlateCorners(cv::Mat gray)
  {
    std::vector<cv::Point2f> corners;

    // Threshold the image
    cv::Mat thresholded;
    cv::threshold(gray, thresholded, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresholded, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour
    double max_area = 0;
    std::vector<cv::Point> best_contour;
    for (size_t i = 0; i < contours.size(); i++)
    {
      double area = cv::contourArea(contours[i]);
      if (area > max_area)
      {
        max_area = area;
        best_contour = contours[i];
      }
    }

    if (best_contour.empty())
      return corners;

    // Approximate the contour to a polygon
    std::vector<cv::Point> approx_poly;
    cv::approxPolyDP(best_contour, approx_poly, cv::arcLength(best_contour, true) * 0.02, true);

    if (approx_poly.size() == 4)
    {
      for(int i = 0; i < 4; i++)
        corners.push_back(approx_poly[i]);
    }

    return corners;
  }
}