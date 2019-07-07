/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : unscented_kalman_filter.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "state_estimate_filter.h"
#include "normal_distribution.h"

namespace state_estimate_filter
{
  /**
   * @brief アンセンテッドカルマンフィルタクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class UnscentedKalmanFilter : AbstractStateEstimateFilter
  {
  public:	// constructors & destoructors
    UnscentedKalmanFilter();

    UnscentedKalmanFilter(
      StateSpaceModel^ i_stateSpaceModel);

    UnscentedKalmanFilter(
      ProcessModelEquation^ i_processModelEquation,
      List<ObservationModelEquation^>^ i_observationModelEquation,
      const cv::Mat i_vecStateTrueInit);

    UnscentedKalmanFilter(
      StateSpaceModel ^i_stateSpaceModel,
      NormalDistribution^ i_posterioriEstimationInit);

  public: // methods
    void Predict(
      const cv::Mat i_vecInputCur) override;

    void Filter() override;

    void Loop(
      const int i_count,
      EstimationResult ^o_estimationResult) override;

  private: // methods

    void UnscentedTransform(
      const cv::Mat i_xMean,
      const cv::Mat i_Pxx,
      ModelFunc^ i_function,
      cv::Mat* o_yMean,
      cv::Mat* o_Pyy,
      cv::Mat* o_Pxy);

    void Cholesky(
      const cv::Mat src,
      cv::Mat* dst);

  public: // properties

  private:  // fields
    /** xhat-_k 現時刻-事前出力推定値分布. */
    ProbabilityDistribution^ priorEstimationOutputCur_ = nullptr;

    /** xhat-_k 現時刻-事前状態・出力推定値分布. */
    ProbabilityDistribution^ priorEstimationStateAndOutputCur_ = nullptr;
  };

}
