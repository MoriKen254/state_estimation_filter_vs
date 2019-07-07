/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : low_variance_sampling.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "resampling.h"

namespace state_estimate_filter
{
  /**
   * @brief Low Variance Sampling アルゴリズム リサンプリングクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class LowVarianceSampling : AbstractResampling
  {
  public:
    static const double kThreshEssDefault = 0.33;
  public:
    LowVarianceSampling()
    {
    }

    LowVarianceSampling(const double i_threshEss)
    {
      threshEss_ = i_threshEss;
    }

  public: // methods
    ParticleDistribution^ Resample(
      ParticleDistribution^ i_priorEstimationCur) override;

    double CalcEffectiveSampleSize(
      ParticleDistribution^ i_priorEstimationCur);

    vector<double> CalcCumulutiveSumsWeight(
      List<Particle^>^ i_priorParticleListCur);

    vector<double> CalcCumulutiveSumsBase(
      List<Particle^>^ i_priorParticleListCur);

    vector<double> CalcResampleID(
      List<Particle^>^ i_priorParticleListCur);

  private:
    double threshEss_ = kThreshEssDefault;
  };
}
