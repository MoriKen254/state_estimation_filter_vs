/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : probability_distribution.h
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
using namespace System;
using namespace System::Collections::Generic;

namespace state_estimate_filter
{
  /**
   * @brief 確率分布基底クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ProbabilityDistribution
  {
  public: // constructors & destoructors
    ProbabilityDistribution();

    ProbabilityDistribution(
      ProbabilityDistribution ^obj);

    ProbabilityDistribution(
      const cv::Mat i_vecMean,
      const cv::Mat i_matVariance);

    ProbabilityDistribution(
      const double i_mean,
      const double i_variance);

  public: // methods
    virtual void CalcDencityFunction(
      const cv::Mat i_matValue,
      cv::Mat* o_vecResult);

    virtual void CalcLogDencityFunction(
      const cv::Mat i_matValue,
      cv::Mat* o_vecResult);

    virtual double CalcDencityFunction(
      const double i_value);

    virtual cv::Mat GenRandomValue();

  public: // properties

    /**
     * 平均値ベクトルのプロパティ.
     *
     * @return  The vector mean.
    **/
    property cv::Mat VecMean_
    {
      cv::Mat get();
      void set(const cv::Mat value);
    }

    /**
     * 平均値ベクトルリストのプロパティ.
     *
     * @return  A List of vector means.
    **/
    property List<double>^ VecMeanList_
    { 
      List<double>^ get(); 
    }

    /**
     * 共分散行列プロパティ.
     *
     * @return  The matrix variance.
    **/
    property cv::Mat MatVariance_
    { 
      cv::Mat get(); 
      void set(const cv::Mat value);
    }

  private:  // fields

    /**
     * 平均値.
     *
     * ### remarks  cv::Matクラス程度なら本当はポインタではないクラスをメンバにしたいんだけど、 
     *              マネージドクラスで非マネージドなクラスをメンバに指定できないので、ポインタに。
     *              ポリモフィジックではないcv::Matについては、わざわざメモリ管理したくないので、 
     *              なるべくフィールド以外でポインタを使わないようにする。
    **/
    cv::Mat* vecMean_ = NULL;

    /**
     * 共分散行列。1次元ならスカラ.
     *
     * ### remarks  cv::Matクラス程度なら本当はポインタではないクラスをメンバにしたいんだけど、
     *              マネージドクラスで非マネージドなクラスをメンバに指定できないので、ポインタに。
     *              ポリモフィジックではないcv::Matについては、わざわざメモリ管理したくないので、
     *              なるべくフィールド以外でポインタを使わないようにする。
    **/
    cv::Mat* matVariance_ = NULL;
  };
}
