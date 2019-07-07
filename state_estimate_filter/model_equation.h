/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : model_equation.h
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
#include "probability_distribution.h"
//
using namespace System;

namespace state_estimate_filter
{
  /**
   * @brief モデル方程式基底クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ModelEquation
  {
  public:
    ModelEquation();

    ModelEquation(
      ModelEquation% obj);

    ModelEquation(
      ProbabilityDistribution^ i_noize_);

  public: // methods
    virtual cv::Mat CalcFunctionWithNoise(
      const cv::Mat i_vecState,
      const int i_time);

    virtual cv::Mat CalcFunctionWithoutNoise(
      const cv::Mat i_vecState,
      const int i_time);

    virtual void UpdateModel(
      const cv::Mat i_inputCur,
      const cv::Mat i_stateEstimatedCur,
      const int i_time);

  public: // operators
    ModelEquation operator =(ModelEquation% rhs);

  public: // properties

    /**
     * ノイズのプロパティ. noize_のget/setを提供.
     *
     * @return  The noize_.
    **/
    property ProbabilityDistribution^ Noise_
    {
      ProbabilityDistribution^ get();
      void set(ProbabilityDistribution^ value);
    }

  private:  // fields
    /** ノイズ. */
    ProbabilityDistribution^ noize_ = nullptr;
  };

  /**
   * @brief プロセスモデル方程式基底クラス.
   *
   * @author  Morita
   * @date  2015/06/17
   *
   * ### remarks  Morita, 2015/06/17.
  **/
  public ref class ProcessModelEquation : ModelEquation
  {
  public: // constructors & destoructors
    ProcessModelEquation();

    ProcessModelEquation(
      ProcessModelEquation% obj);

    ProcessModelEquation(ProbabilityDistribution^ i_noize_);

  public: // methods

  public: // properties

    /**
     * A行列のプロパティ. *matA_のget/setを提供.
     *
     * @return  The *matA_.
    **/
    property cv::Mat MatA_
    {
      cv::Mat get();
      void set(const cv::Mat value);
    }

    /**
     * bベクトルのプロパティ.
     *
     * @return  The *vecb_.
    **/
    property cv::Mat Vecb_
    {
      cv::Mat get();
      void set(const cv::Mat value);
    }

  private:  // fields
    // モデリングパラメータ
    /** A行列. */
    cv::Mat *matA_ = NULL;
    /** bベクトル. */
    cv::Mat *vecb_ = NULL;
  };

  /**
   * @brief 線形プロセスモデル方程式クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class LinearProcessModelEquation : ProcessModelEquation
  {
  public: // constructors & destoructors
    LinearProcessModelEquation();

    LinearProcessModelEquation(
      const cv::Mat i_matA,
      const cv::Mat i_vecb,
      ProbabilityDistribution^ i_noize);

  public: // properties

  private:  // fields

  };

  /**
   * @brief モデル関数デリゲード.
   *
   * @author  Morita
   * @date  2015/06/17
   *
   * @param i_x     状態ベクトル.
   * @param i_time  現在時刻.
   *
   * @return  関数計算結果.
  **/
  public delegate cv::Mat ModelFunc(const cv::Mat i_x, const int k);

  /**
   * @brief 非線形プロセスモデル方程式クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class NonLinearProcessModelEquation : ProcessModelEquation
  {
  public: // constructors & destoructors
    NonLinearProcessModelEquation();

    NonLinearProcessModelEquation(
      ModelFunc^ i_function,
      List<ModelFunc^>^ i_fJacobians,
      const cv::Mat i_vecb,
      ProbabilityDistribution^ i_noize);

  public: // methods
    cv::Mat CalcFunctionWithoutNoise(
      const cv::Mat i_vecState,
      const int i_time) override;

    cv::Mat CalcFunctionWithNoise(
      const cv::Mat i_vecState,
      const int i_time) override;

    void UpdateModel(
      const cv::Mat i_inputCur,
      const cv::Mat i_stateEstimatedCur,
      const int i_time) override;

  public:  // fields
    /// <summary>モデル関数</summary>
    ModelFunc^ function_ = nullptr;
    /// <summary>ヤコビアン</summary>
    List<ModelFunc^>^ fJacobians_ = nullptr;
  };

  /**
   * @brief 観測モデル方程式クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ObservationModelEquation: ModelEquation
  {
  public: // constructors & destoructors
    ObservationModelEquation();

    ObservationModelEquation(
      ObservationModelEquation% obj);

    ObservationModelEquation(
      ProbabilityDistribution^ i_noize_);

  public: // methods
    void ObserveWithNoize(
      const cv::Mat i_vecStateTrue,
      const int i_time);

  public: // properties

    /**
     * C行列のプロパティ. *matC_のget/setを提供.
     *
     * @return  *matC_. NULLポインタ時は-1を格納した1次元cv::Mat.
    **/
    property cv::Mat MatC_
    {
      cv::Mat get();
      void set(const cv::Mat value);
    }

  public:
    /** 現時刻観測値確率分布. */
    ProbabilityDistribution^ observationCur_ = gcnew ProbabilityDistribution();

  private:  // fields
    /** C行列. */
    cv::Mat *matC_ = NULL;
  };

  /**
   * @brief 線形観測モデル方程式クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class LinearObservationModelEquation : ObservationModelEquation
  {
  public: // constructors & destoructors
    LinearObservationModelEquation();
    
    LinearObservationModelEquation(
      const cv::Mat i_matC
    , ProbabilityDistribution ^i_noize);

  public: // methods
    cv::Mat CalcFunctionWithNoise(
      const cv::Mat i_vecStateTrue,
      const int i_time) override;
  };

  /**
   * @brief 非線形観測モデル方程式クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class NonLinearObservationModelEquation : ObservationModelEquation
  {
  public: // constructors & destoructors
    NonLinearObservationModelEquation();

    NonLinearObservationModelEquation(
      ModelFunc^ i_function,
      List<ModelFunc^>^ i_fJacobians,
      ProbabilityDistribution^ i_noize);

  public: // methods
    cv::Mat CalcFunctionWithoutNoise(
      const cv::Mat i_vecState,
      const int i_time) override;

    cv::Mat CalcFunctionWithNoise(
      const cv::Mat i_vecState,
      const int i_time) override;

    void UpdateModel(
      const cv::Mat i_inputCur,
      const cv::Mat i_stateEstimatedCur,
      const int i_time) override;

  public:  // fields
    /** モデル関数. */
    ModelFunc^ function_ = nullptr;
    /** ヤコビアン. */
    List<ModelFunc^>^ fJacobians_ = nullptr;
  };
}
