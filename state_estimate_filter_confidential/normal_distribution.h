/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : normal_distribution.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "probability_distribution.h"

namespace state_estimate_filter
{
  /**
   * @brief 正規分布クラス.
   *
   * @author  Morita
   * @date  2015/06/17
   */
  public ref class NormalDistribution : ProbabilityDistribution
  {
  public: // constructors & destoructors
    NormalDistribution(){};

    NormalDistribution(
      NormalDistribution ^obj);

    NormalDistribution(
      NormalDistribution% obj);

    NormalDistribution(
      const cv::Mat i_vecMean,
      const cv::Mat i_matVariance);

    NormalDistribution(
      const double i_mean,
      const double i_variance);

  public: // operators
    NormalDistribution operator =(NormalDistribution% rhs) { return NormalDistribution(rhs); }

  public: // methods
    void CalcDencityFunction(
      const cv::Mat i_matValue,
      cv::Mat* o_vecResult) override;

    void CalcLogDencityFunction(
      const cv::Mat i_matValue,
      cv::Mat* o_vecResult) override;

    double CalcDencityFunction(
      const double i_value) override;

    cv::Mat GenRandomValue() override;
  };
}