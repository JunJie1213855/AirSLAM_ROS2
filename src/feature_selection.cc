#include "feature_selection.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

std::vector<HeatmapKeypoint> SelectHeatmapPoints(
    const float* scores, int height, int width, float threshold, int border,
    int max_points, bool use_mask_uniformity, int min_distance,
    double scale_x, double scale_y) {
  if (!scores || height <= 0 || width <= 0 || max_points <= 0 ||
      !std::isfinite(scale_x) || !std::isfinite(scale_y) || scale_x <= 0 || scale_y <= 0 ||
      (use_mask_uniformity && min_distance <= 0)) {
    throw std::invalid_argument("Invalid feature selection dimensions or mask distance");
  }
  std::vector<HeatmapKeypoint> candidates;
  border = std::max(border, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const float score = scores[y * width + x];
      if (!std::isfinite(score) || score < threshold ||
          x < border || x > width - border || y < border || y > height - border) continue;
      candidates.push_back({score, x, y});
    }
  }
  const auto by_score = [](const HeatmapKeypoint& a, const HeatmapKeypoint& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  };
  if (!use_mask_uniformity) {
    // Match the original global-Top-K path, including scan order below the cap.
    if (candidates.size() > static_cast<size_t>(max_points)) {
      std::sort(candidates.begin(), candidates.end(), by_score);
      candidates.resize(max_points);
    }
    return candidates;
  }
  std::sort(candidates.begin(), candidates.end(), by_score);
  cv::Mat mask(static_cast<int>(std::ceil(height * scale_y)),
               static_cast<int>(std::ceil(width * scale_x)), CV_8UC1, cv::Scalar(255));
  std::vector<HeatmapKeypoint> selected;
  selected.reserve(std::min(candidates.size(), static_cast<size_t>(max_points)));
  for (const auto& point : candidates) {
    const cv::Point pixel(cvRound(point.x * scale_x), cvRound(point.y * scale_y));
    if (pixel.x < 0 || pixel.y < 0 || pixel.x >= mask.cols || pixel.y >= mask.rows ||
        mask.at<unsigned char>(pixel) == 0) continue;
    selected.push_back(point);
    // VINS-style circular exclusion; deep detections are ranked by confidence.
    cv::circle(mask, pixel, min_distance, cv::Scalar(0), cv::FILLED);
    if (selected.size() == static_cast<size_t>(max_points)) break;
  }
  return selected;
}
