/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : resampling.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "particle_distribution.h"
//
using namespace System;
using std::vector;

namespace state_estimate_filter
{
  /**
   * 抽象リサンプリングクラス. 実装は無し.
   *
   * @author  Morita
   * @date  2015/06/27
  **/
  public ref class AbstractResampling abstract
  {
  public: // methods

    /**
     * Default constructor.
     *
     * @author  Morita
     * @date  2015/06/23
    **/
    AbstractResampling(){};

    /**
     * リサンプリングする.
     *
     * @author  Morita
     * @date  2015/06/23
     *
     * @param i_priorEstimationCur  現時刻事後推定値.
     *
     * @return  リサンプル後の粒子分布.
    **/
    virtual ParticleDistribution^ Resample(
      ParticleDistribution^ i_priorEstimationCur) = 0;
  };
}
