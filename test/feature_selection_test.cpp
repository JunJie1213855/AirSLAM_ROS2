#include <gtest/gtest.h>
#include "feature_selection.h"

TEST(FeatureSelection, MaskKeepsStrongestLocalPointAndRecoversDistantWeakerPoints) {
  std::vector<float> scores(20*20, 0);
  scores[5*20+5]=1; scores[5*20+6]=.9; scores[6*20+5]=.8; scores[5*20+12]=.5;
  const auto selected=SelectHeatmapPoints(scores.data(),20,20,.1,0,2,true,3);
  ASSERT_EQ(2u,selected.size());
  EXPECT_EQ(5,selected[0].x); EXPECT_EQ(5,selected[0].y);
  EXPECT_EQ(12,selected[1].x); EXPECT_EQ(5,selected[1].y);
  const auto original=SelectHeatmapPoints(scores.data(),20,20,.1,0,2,false,3);
  ASSERT_EQ(2u,original.size()); EXPECT_EQ(6,original[1].x);
}

TEST(FeatureSelection, SpacingUsesOriginalPixelsWithAnisotropicResize) {
  std::vector<float> scores(20*20,0);
  scores[2*20+6]=1; scores[2*20+10]=.9; scores[6*20+6]=.8;
  const auto selected=SelectHeatmapPoints(scores.data(),20,20,.1,0,10,true,7,2.,.5);
  ASSERT_EQ(2u,selected.size());
  EXPECT_EQ(6,selected[0].x); EXPECT_EQ(10,selected[1].x);
}

TEST(FeatureSelection, FiltersBorderAndKeepsTheConfiguredPointBudget) {
  std::vector<float> scores(30*30,0);
  scores[0]=1;
  scores[5*30+5]=.9; scores[5*30+15]=.8; scores[15*30+5]=.7;
  const auto selected=SelectHeatmapPoints(scores.data(),30,30,.1,3,2,true,3);
  ASSERT_EQ(2u,selected.size());
  EXPECT_EQ(5,selected[0].x); EXPECT_EQ(15,selected[1].x);
}

TEST(FeatureSelection, DisabledMaskPreservesScanOrderBelowBudget) {
  std::vector<float> scores(20*20,0);
  scores[5*20+5]=.2; scores[5*20+6]=.9;
  const auto selected=SelectHeatmapPoints(scores.data(),20,20,.1,0,10,false,15);
  ASSERT_EQ(2u,selected.size()); EXPECT_EQ(5,selected[0].x); EXPECT_EQ(6,selected[1].x);
}
