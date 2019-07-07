/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : state_space_model.h
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
#include "model_equation.h"
#include "probability_distribution.h"
//
using namespace System;

namespace state_estimate_filter
{
  /**
   * @brief 状態空間モデル基底クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class StateSpaceModel
  {
  public: // constructors & destoructors
    StateSpaceModel();

    StateSpaceModel(
      StateSpaceModel% obj);

    StateSpaceModel(
      StateSpaceModel^ obj);

    StateSpaceModel(
      ProcessModelEquation^ i_processModelEquation,
      List<ObservationModelEquation^>^ i_observationModelEquation,
      const cv::Mat i_vecStateTrueInit);

  public:
    virtual void UpdateInnerTrueValueWithNoise(
      const cv::Mat i_inputCur,
      const int i_time);

    virtual cv::Mat UpdateOuterTrueValueWithNoise(
      const cv::Mat i_inputCur,
      const int i_time);

    virtual void ObserveWithNoise(
      const cv::Mat i_trueCur,
      const int i_time);

    virtual cv::Mat PredictPriorStateCurWithoutNoise(
      const cv::Mat i_stateEstimatedOld,
      const cv::Mat i_inputCur, const int i_time);

    virtual void PredictPriorStateCurWithNoise(
      const cv::Mat i_stateEstimatedOld,
      const cv::Mat i_inputCur,
      const int i_time,
      cv::Mat* o_statePredictedCur);

  public: // properties

    /**
     * プロセスモデル式のプロパティ. *processModelEquation_のget/setを提供.
     *
     * @return  The *processModelEquation_.
    **/
    property ProcessModelEquation^ ProcessModelEquation_
    {
      ProcessModelEquation^ get();
      void set(ProcessModelEquation^ value);
    }

    /**
     * 観測モデル式のプロパティ. *processModelEquation_のget/setを提供.
     *
     * @return  The *processModelEquation_.
    **/
    property List<ObservationModelEquation^>^ ObservationModelEquations_
    {
      List<ObservationModelEquation^>^ get();
      void set(List<ObservationModelEquation^>^ value);
    }

    /**
     * 真値のプロパティ. *vecStateTrueCur_のget/setを提供.
     *
     * @return  The *vecStateTrueCur_.
    **/
    property cv::Mat VecStateTrueCur_
    {
      cv::Mat get();
      void set(cv::Mat value);
    }

  public: // fields

  private:  // fields
    /** プロセスモデル式. */
    ProcessModelEquation^ processModelEquation_;
    /** 観測モデル式. */
    List<ObservationModelEquation^>^ observationModelEquations_;
    /** 真値ベクトル. 真値は状態空間モデルの中だけで持っている.    
     *  現実世界において, 状態推定クラスでは知りようがない. */
    cv::Mat* vecStateTrueCur_ = NULL;
  };

  /**
   * @brief 線形状態空間モデルクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class LinearStateSpaceModel : StateSpaceModel
  {
  public:	// constructors & destoructors
    LinearStateSpaceModel();

    LinearStateSpaceModel(
      LinearStateSpaceModel% obj);

    LinearStateSpaceModel(
      LinearStateSpaceModel^ obj);

    LinearStateSpaceModel(
      ProcessModelEquation^ i_processModelEquation
    , List<ObservationModelEquation^>^ i_observationModelEquation
    , const cv::Mat i_trueValue);

  public: // methods
    void UpdateInnerTrueValueWithNoise(
      const cv::Mat i_inputCur
    , const int i_time) override;

    cv::Mat UpdateOuterTrueValueWithNoise(
      const cv::Mat i_inputCur
    , const int i_time) override;

    cv::Mat PredictPriorStateCurWithoutNoise(
      const cv::Mat i_stateEstimatedOld,
      const cv::Mat i_inputCur,
      const int i_time) override;
  };

  /**
   * @brief 非線形状態空間モデルクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class NonLinearStateSpaceModel : StateSpaceModel
  {
  public:	// constructors & destoructors
    NonLinearStateSpaceModel();

    NonLinearStateSpaceModel(
      NonLinearStateSpaceModel% obj);

    NonLinearStateSpaceModel(
      NonLinearStateSpaceModel^ obj);

    NonLinearStateSpaceModel(
      ProcessModelEquation^ i_processModelEquation
    , List<ObservationModelEquation^>^ i_observationModelEquation
    , const cv::Mat i_trueValue);

  public: // methods
    void UpdateInnerTrueValueWithNoise(
      const cv::Mat i_inputCur
    , const int i_time) override;

    cv::Mat UpdateOuterTrueValueWithNoise(
      const cv::Mat i_inputCur
    , const int i_time) override;

    void UpdateModel(
      const cv::Mat i_inputCur
    , const cv::Mat i_stateEstimatedCur
    , const int i_time);

    cv::Mat PredictPriorStateCurWithoutNoise(
      const cv::Mat i_stateEstimatedOld
    , const cv::Mat i_inputCur
    , const int i_time) override;
   
    void PredictPriorStateCurWithNoise(
      const cv::Mat i_stateEstimatedOld
    , const cv::Mat i_inputCur
    , const int i_time
    , cv::Mat* o_statePredictedCur) override;
  };
}
