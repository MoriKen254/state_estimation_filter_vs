// particle.h
#pragma once

#include "normal_distribution.h"

namespace state_estimate_filter
{  
  /**
   * @brief パーティクル1つのパラメータを格納したクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class Particle
  {
  public: // constructors
    Particle();

    Particle(
      Particle% obj);

    Particle(
      Particle^ obj);

    Particle(
        NormalDistribution^ i_stateDistribution
      , const double i_weight);

  public: // operators
    Particle operator =(Particle% rhs);

  public: // properties
    /**
     * 確率分布のプロパティ.
     *
     * @return  The stateDistribution_.
    **/
    property NormalDistribution^ StateDistribution_
    { 
      NormalDistribution^ get(); 
      void set(NormalDistribution^ value); 
    }

    /**
     * 重みプロパティ.
     *
     * @return  The weight.
    **/
    property double Weight_
    { 
      double get(); 
      void set(const double value);
    }

  private:  // fields
    /** 確率分布(正規分布) */
    NormalDistribution^ stateDistribution_ = nullptr;
    /** 重み. */
    double weight_ = 0;
  };
}
