#ifndef AIRSLAM_FEATURE_SELECTION_H
#define AIRSLAM_FEATURE_SELECTION_H
#include <vector>

struct HeatmapKeypoint {
  float score;
  int x, y;
};

// Returned coordinates stay in heatmap pixels; masking uses original-image pixels.
std::vector<HeatmapKeypoint> SelectHeatmapPoints(
    const float* scores, int height, int width, float threshold, int border,
    int max_points, bool use_mask_uniformity, int min_distance,
    double scale_x = 1.0, double scale_y = 1.0);
#endif
