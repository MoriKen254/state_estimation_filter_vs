/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : particle_distribution.h
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
#include "particle.h"

namespace state_estimate_filter
{
  /**
   * @brief パーティクル分布クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ParticleDistribution : ProbabilityDistribution
  {
  public: // constructors & destoructors

    ParticleDistribution(){};

    ParticleDistribution(
      ParticleDistribution ^obj);

    ParticleDistribution(
      ParticleDistribution% obj);

    ParticleDistribution(
      const cv::Mat i_vecMean,
      const cv::Mat i_matVariance,
      List<Particle^>^ i_particles);

    /**
     * 代入演算子.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param [in,out]  rhs 代入元オブジェクト.
     *
     * @return  自オブジェクト.
    **/
    ParticleDistribution operator =(ParticleDistribution% rhs) { return ParticleDistribution(rhs); }

  public: // properties
    /**
     * パーティクル群のプロパティ.
     *
     * @return  The particles.
    **/
    property List<Particle^>^ Particles_
    {
      List<Particle^>^ get();
      void set(List<Particle^>^ value);
    }

  public: // fiedls
    /** 現時刻リサンプリングフラグ。trueなら実行、falseなら不実行. */
    bool resampledCurr_ = false;

  private: // fields
    /** パーティクル群. */
    List<Particle^>^ particles_ = nullptr;
  };
}
