/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : linear_kalman_filter.h
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
   * @brief 線形カルマンフィルタクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class LinearKalmanFilter : AbstractStateEstimateFilter
  {
  public:	// constructors & destoructors
    LinearKalmanFilter();

    LinearKalmanFilter(
      StateSpaceModel^ i_stateSpaceModel);

    LinearKalmanFilter(
      ProcessModelEquation^ i_processModelEquation,
      List<ObservationModelEquation^>^ i_observationModelEquation,
      const cv::Mat i_vecStateTrueInit);

    LinearKalmanFilter(
      StateSpaceModel ^i_stateSpaceModel,
      NormalDistribution^ i_posterioriEstimationInit);

  public: // methods
    void Predict(
      const cv::Mat i_vecInputCur) override;  // 予測ステップ(事前推定)

    void Filter() override; // フィルタリングステップ(事後推定)

  private: // methods

  public: // properties
    /**
     * 事後状態推定値から予測した観測値と実観測値の誤差.
     *
     * @return  The error between posteriori estimation and observation.
    **/
    property double ErrorBtwnPosterioriEstimationAndObservation
    {
      double get() override;  // オーバーライド。基底クラスでは非線形関数で定義。
    };

  private:  // fields

  };
}
