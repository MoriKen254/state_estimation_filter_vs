/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : linear_kalman_filter.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "linear_kalman_filter.h"
#include "state_estimater_factory.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
LinearKalmanFilter::LinearKalmanFilter() :
AbstractStateEstimateFilter()
{
  ConfigureDefaultParameters();
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel 状態空間モデル.
**/
LinearKalmanFilter::LinearKalmanFilter(
  StateSpaceModel^ i_stateSpaceModel)
  : AbstractStateEstimateFilter(
    i_stateSpaceModel)
{
  ConfigureDefaultParameters();
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_processModelEquation      プロセスモデル方程式.
 * @param i_observationModelEquation  観測モデル方程式.
 * @param i_vecStateTrueInit          真の状態初期値.
**/
LinearKalmanFilter::LinearKalmanFilter(
  ProcessModelEquation^ i_processModelEquation,
  List<ObservationModelEquation^>^ i_observationModelEquation,
  const cv::Mat i_vecStateTrueInit)
  : AbstractStateEstimateFilter(
    i_processModelEquation,
    i_observationModelEquation,
    i_vecStateTrueInit)
{
  ConfigureDefaultParameters();
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel           状態空間モデル.
 * @param i_posterioriEstimationInit  事後状態推定初期値.
**/
LinearKalmanFilter::LinearKalmanFilter(
  StateSpaceModel ^i_stateSpaceModel,
  NormalDistribution^ i_posterioriEstimationInit)
  : AbstractStateEstimateFilter(
    i_stateSpaceModel)
{
  PosterioriEstimationOld_ = gcnew NormalDistribution(i_posterioriEstimationInit);
  ConfigureDefaultParameters();

#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationOld_->VecMean_.at<double>(0, 0);
#endif
}

/**
 * 一段先予測. 運動モデルに従って予測する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecInputCur 制御入力ベクトル.
**/
void LinearKalmanFilter::Predict(
  const cv::Mat i_vecInputCur)
{
  // arrange
  //VecInputCur_.at<double>(0, 0) = 0.; // 入力を常に0とする。
  cv::Mat matA = StateSpaceModel_->ProcessModelEquation_->MatA_;
  cv::Mat vecb = StateSpaceModel_->ProcessModelEquation_->Vecb_;
  cv::Mat vecStateOld = PosterioriEstimationOld_->VecMean_;
  cv::Mat matVarianceStateOld = PosterioriEstimationOld_->MatVariance_;
  cv::Mat matVarianceNoiseProcess = StateSpaceModel_->ProcessModelEquation_->Noise_->MatVariance_;

  // act
  //--状態: xhat_k = A*(x_k - 1) + b*(u_k)
  PriorEstimationCur_->VecMean_ =
    StateSpaceModel_->PredictPriorStateCurWithoutNoise(vecStateOld, i_vecInputCur, TimeCur_);

  //-- 誤差共分散: Pm = A*P*A' + B*Q*B'
  PriorEstimationCur_->MatVariance_ = matA*matVarianceStateOld*matA.t() + vecb*matVarianceNoiseProcess*vecb.t();

#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = vecStateOld.at<double>(0, 0);
  result[debugIndex++] = vecStatePredictedCurTemp.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matVarianceStateOld.at<double>(0, 0);
  result[debugIndex++] = matVarianceNoiseProcess.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->MatVariance_.at<double>(0, 0);
#endif
}

/**
 * フィルタリング. 観測の結果を考慮する.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void LinearKalmanFilter::Filter()
{
  // arrange
  List<ObservationModelEquation^>^ linearObservationModelEquations = StateSpaceModel_->ObservationModelEquations_;
  cv::Mat matPm = PriorEstimationCur_->MatVariance_;
  cv::Mat matC = linearObservationModelEquations[0]->MatC_;
  cv::Mat matVarianceObserve = StateSpaceModel_->ObservationModelEquations_[0]->Noise_->MatVariance_;
  cv::Mat vecObservationCur = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_;

  //-- カルマンゲイン: G = P-*C/(C'*P-*C + R)
  cv::Mat matG = matPm*matC / (matC.t()*matPm*matC + matVarianceObserve);

  // act
  //-- 事後状態推定値: xhat_k = xhat-_k + G*(y-C'xhat-_k)
  //---- イノベーション過程 = y-C'xhat-_k
  cv::Mat vecInnovation = vecObservationCur - matC.t() * PriorEstimationCur_->VecMean_;
  //---- xhat_k = xhat-_k + G*innovaion
  PosterioriEstimationCur_->VecMean_ = PriorEstimationCur_->VecMean_ + matG * vecInnovation;

  //-- 事後誤差共分散: P_k = (I - G*C')*Pm
  PosterioriEstimationCur_->MatVariance_ = (cv::Mat::eye(1, 1, CV_64F) - matG*matC.t())*matPm;

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = matG.at<double>(0, 0);
  result[debugIndex++] = VecObservationCur_.at<double>(0, 0);
  result[debugIndex++] = vecInnovation.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matC.at<double>(0, 0);
  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matPm.at<double>(0, 0);
  result[debugIndex++] = PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
#endif
}

/**
 * Gets the ErrorBtwnPosterioriEstimationAndObservation.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A ErrorBtwnPosterioriEstimationAndObservation.
**/
double LinearKalmanFilter::ErrorBtwnPosterioriEstimationAndObservation::get()
{
  // arrange
  //-- 事後状態推定値から予測した観測値
  cv::Mat vecPosterioriEstimationAndObservation = StateSpaceModel_->ObservationModelEquations_[0]->MatC_ * PosterioriEstimationCur_->VecMean_;
  //-- 事後状態推定値から予測した観測値(doulbe)
  double posterioriEstimationAndObservation = vecPosterioriEstimationAndObservation.at<double>(0, 0);
  //-- 実観測値
  double observation = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);

  // act
  return (observation - posterioriEstimationAndObservation);
}
