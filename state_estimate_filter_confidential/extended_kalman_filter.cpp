/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : extended_kalman_filter.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "extended_kalman_filter.h"
#include "state_estimater_factory.h"
#include "truth_observation_generator.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
ExtendedKalmanFilter::ExtendedKalmanFilter() :
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
ExtendedKalmanFilter::ExtendedKalmanFilter(
  StateSpaceModel^ i_stateSpaceModel) :
  AbstractStateEstimateFilter(i_stateSpaceModel)
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
 * @param i_vecStateTrueInit          初期状態真値.
**/
ExtendedKalmanFilter::ExtendedKalmanFilter(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_vecStateTrueInit)
  : AbstractStateEstimateFilter(
    i_processModelEquation
  , i_observationModelEquation
  , i_vecStateTrueInit)
{
  ConfigureDefaultParameters();
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel           状態推定用モデル方程式一式.
 * @param i_posterioriEstimationInit  事後推定値初期値.
**/
ExtendedKalmanFilter::ExtendedKalmanFilter(
  StateSpaceModel ^i_stateSpaceModel
, NormalDistribution^ i_posterioriEstimationInit)
  : AbstractStateEstimateFilter(
    i_stateSpaceModel)
{
  PosterioriEstimationOld_ = gcnew NormalDistribution(i_posterioriEstimationInit);
  ConfigureDefaultParameters();
  // debug
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
void ExtendedKalmanFilter::Predict(
  const cv::Mat i_vecInputCur)
{
  // arrange
  //VecInputCur_.at<double>(0, 0) = 0.; // 入力を常に0とする。
  cv::Mat matVarianceStateOld = PosterioriEstimationOld_->MatVariance_;
  cv::Mat matVarianceNoiseProcess = this->StateSpaceModel_->ProcessModelEquation_->Noise_->MatVariance_;

  // act
  //--状態: xhat[k] = f(x[k-1]) + b*(u[k])
  PriorEstimationCur_->VecMean_ =
    StateSpaceModel_->PredictPriorStateCurWithoutNoise(PosterioriEstimationOld_->VecMean_, i_vecInputCur, TimeCur_);

  //--モデル更新(事前推定後すぐに行う)
  cv::Mat vecInput = cv::Mat::zeros(1, 1, CV_64F);  // 入力はゼロとする
  (static_cast<NonLinearStateSpaceModel^>(StateSpaceModel_))->UpdateModel(vecInput, PriorEstimationCur_->VecMean_, TimeCur_);

  //-- 誤差共分散: Pm = A*P*A' + B*Q*B'
  cv::Mat matA = StateSpaceModel_->ProcessModelEquation_->MatA_;
  cv::Mat vecb = StateSpaceModel_->ProcessModelEquation_->Vecb_;
  PriorEstimationCur_->MatVariance_ = matA*matVarianceStateOld*matA.t() + vecb*matVarianceNoiseProcess*vecb.t();

#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = vecStateOld.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matVarianceStateOld.at<double>(0, 0);
  result[debugIndex++] = matVarianceNoiseProcess.at<double>(0, 0);
  result[debugIndex++] = matA.at<double>(0, 0);
  result[debugIndex++] = vecb.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->MatVariance_.at<double>(0, 0);
#endif
}

/**
 * フィルタリング. 観測の結果を考慮する.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void ExtendedKalmanFilter::Filter()
{
  // arrange
  ObservationModelEquation^ nonlinearObservationModelEquation
    = StateSpaceModel_->ObservationModelEquations_[0];
  cv::Mat matPm = PriorEstimationCur_->MatVariance_;
  cv::Mat matC = nonlinearObservationModelEquation->MatC_;
  cv::Mat matVarianceObserve = StateSpaceModel_->ObservationModelEquations_[0]->Noise_->MatVariance_;
  //-- カルマンゲイン: G = P-*C/(C'*P-*C + R)
  cv::Mat matG = matPm*matC / (matC.t()*matPm*matC + matVarianceObserve);

  // act
  //-- 事後状態推定値: xhat_k = xhat-_k + G*(y-C'xhat-_k)
  //---- イノベーション過程 = y - h(xhat-_k)
  cv::Mat hx =
    nonlinearObservationModelEquation->CalcFunctionWithoutNoise(PriorEstimationCur_->VecMean_, TimeCur_);
  cv::Mat vecInnovation = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_ - hx;
  //---- xhat_k = xhat-_k + G*innovaion
  PosterioriEstimationCur_->VecMean_ = PriorEstimationCur_->VecMean_ + matG * vecInnovation;

  //-- 事後誤差共分散: P_k = (I - G*C')*Pm
  PosterioriEstimationCur_->MatVariance_ = (cv::Mat::eye(1, 1, CV_64F) - matG*matC.t())*matPm;

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = matG.at<double>(0, 0);
  result[debugIndex++] = hx.at<double>(0, 0);
  result[debugIndex++] = VecObservationCur_.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matC.at<double>(0, 0);
  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matPm.at<double>(0, 0);
  result[debugIndex++] = PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
#endif
}

/**
 * シミュレーションを行う.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_count             ループ回数.
 * @param i_estimationResult  状態推定結果.
**/
void ExtendedKalmanFilter::Loop(
  const int i_count,
  EstimationResult^ i_estimationResult)
{
  // 入力は常に0
  cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

  TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
  List<TruthAndObservation^>^ truthAndObservations =
    truthAndObservationGenerator->Generate(this->StateSpaceModel_, i_count + 1);
#if MY_DEBUG
  vector<double> truths(truthAndObservations->Count);
  vector<double> observations(truthAndObservations->Count);
  for (int i = 0; i < truthAndObservations->Count; i++)
  {
    truths[i] = truthAndObservations[i]->truth_[0];
    observations[i] = truthAndObservations[i]->observation_[0];
  }
#endif
  // カルマンフィルタ開始。時刻k=1からk=4まで実施   
  for (int k = 0; k < i_count; k++)
  {
    // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
    // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
    //this->UpdateToNextTime(vecInputCur);
    this->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_);
    this->Estimate(vecInputCur); // 入力値を元に推定
    this->AcquireEstimationResult(i_estimationResult);  // 結果を引数オブジェクトに格納
  }
}
