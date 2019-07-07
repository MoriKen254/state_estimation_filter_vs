/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : point_estimation.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include <opencv2/opencv_lib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//
#include "state_estimate_filter.h"
#include "particle.h"
//
using namespace System;
using namespace System::Collections::Generic;

namespace state_estimate_filter
{
  /**
   * @brief 点推定アルゴリズム基底クラス. 抽象クラスなのでアルゴリズムの実装は無し.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class AbstractPointEstimation abstract
  {
  public: // constructor & destoructor
    AbstractPointEstimation();

  public: // methods

    /**
     * パーティクル分布から点推定を行う. 純粋仮想関数.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_stateEstimateFilter 状態推定フィルタオブジェクト.
     *
     * @return  点推定値.
    **/
    virtual cv::Mat EstimateFromParticles(
      AbstractStateEstimateFilter^ i_stateEstimateFilter) = NULL;
  };

  /**
   * @brief Minimum Mean Squared Error 点推定アルゴリズムクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class MMSEPointEstimation : AbstractPointEstimation
  {
  public: // constructor & destoructor
    MMSEPointEstimation();

  public: // methods
    cv::Mat EstimateFromParticles(
      AbstractStateEstimateFilter^ i_stateEstimateFilter) override;
  };

  /**
   * @brief MaxWeight 点推定アルゴリズムクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class MaxWeightPointEstimation : AbstractPointEstimation
  {
  public: // constructor & destoructor
    MaxWeightPointEstimation();

  public: // methods
    cv::Mat EstimateFromParticles(
      AbstractStateEstimateFilter^ i_stateEstimateFilter) override;
  };

  /**
   * @brief pf-Maximum A Posteriori 点推定アルゴリズムクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class PfMapPointEstimation : AbstractPointEstimation
  {
  public: // constructor & destoructor
    PfMapPointEstimation();

  public: // methods
    cv::Mat EstimateFromParticles(
      AbstractStateEstimateFilter^ i_stateEstimateFilter) override;

  private:  // methods
    List<Particle^>^ PredictNextStateWithoutNoise(
      AbstractStateEstimateFilter^ i_stateEstimateFilter);

    void CalcLikelihoodsOfXhatFromYOldSum(
      AbstractStateEstimateFilter^ i_stateEstimateFilter
    , List<Particle^>^ i_priorCurParticleListWithoutNoise);
    
    vector<cv::Mat> CalcPosterioriProbability(
      AbstractStateEstimateFilter^ i_stateEstimateFilter);

    int AcquireMapMaxIndex(
      vector<cv::Mat> posterioriOfXhatFromYCur);

  public:
    vector<double> *posterioriOfXhatFromYCur_;

  private:  // fields
    /** 予測尤度: p(xhat(i)[k] | yhat[1:k-1]): メソッド間IF用に持っているだけ。. */
    vector<cv::Mat> *likelihoodsOfXhatFromYOldSum_ = new vector<cv::Mat>;
  };

  /**
   * @brief Kernel Max A Posteriori 点推定アルゴリズムクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class KernelMapPointEstimation : AbstractPointEstimation
  {
  public: // constructor & destoructor
    KernelMapPointEstimation();

  public: // methods
    cv::Mat EstimateFromParticles(
      AbstractStateEstimateFilter^ i_stateEstimateFilter) override;
  };
}
