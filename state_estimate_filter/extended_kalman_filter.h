/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : extended_kalman_filter.h
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
   * @brief 拡張カルマンフィルタクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ExtendedKalmanFilter : AbstractStateEstimateFilter
  {
  public:	// constructors & destoructors
    ExtendedKalmanFilter();

    ExtendedKalmanFilter(
      StateSpaceModel^ i_stateSpaceModel);

    ExtendedKalmanFilter(
      ProcessModelEquation^ i_processModelEquation,
      List<ObservationModelEquation^>^ i_observationModelEquation,
      const cv::Mat i_vecStateTrueInit);

    ExtendedKalmanFilter(
      StateSpaceModel ^i_stateSpaceModel,
      NormalDistribution^ i_posterioriEstimationInit);

  public: // methods
    void Predict(
      const cv::Mat i_vecInputCur) override;

    void Filter() override;

    void Loop(
      const int i_count,
      EstimationResult ^i_estimationResult) override;

  private: // methods

  public: // properties

  private:  // fields

  };
}