/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : unscented_kalman_filter.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "unscented_kalman_filter.h"
#include "state_estimater_factory.h"
#include "truth_observation_generator.h"
//
using namespace state_estimate_filter;

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
UnscentedKalmanFilter::UnscentedKalmanFilter() :
AbstractStateEstimateFilter()
{
  ConfigureDefaultParameters();
  priorEstimationOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
  priorEstimationStateAndOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel 状態空間モデル.
**/
UnscentedKalmanFilter::UnscentedKalmanFilter(
  StateSpaceModel^ i_stateSpaceModel)
  : AbstractStateEstimateFilter(
      i_stateSpaceModel)
{
  ConfigureDefaultParameters();
  priorEstimationOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
  priorEstimationStateAndOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_processModelEquation      プロセスモデル方程式.
 * @param i_observationModelEquation  観測モデル方程式.
 * @param i_vecStateTrueInit          真の状態初期値.
**/
UnscentedKalmanFilter::UnscentedKalmanFilter(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_vecStateTrueInit)
  : AbstractStateEstimateFilter(
      i_processModelEquation,
      i_observationModelEquation,
      i_vecStateTrueInit)
{
  ConfigureDefaultParameters();
  priorEstimationOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
  priorEstimationStateAndOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateSpaceModel           状態空間モデル.
 * @param i_posterioriEstimationInit  事後状態推定初期値.
**/
UnscentedKalmanFilter::UnscentedKalmanFilter(
  StateSpaceModel ^i_stateSpaceModel,
  NormalDistribution^ i_posterioriEstimationInit)
  : AbstractStateEstimateFilter(
    i_stateSpaceModel)
{
  PosterioriEstimationOld_ = gcnew NormalDistribution(i_posterioriEstimationInit);
  ConfigureDefaultParameters();
  // UKF 専用の変数
  priorEstimationOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
  priorEstimationStateAndOutputCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationOld_->VecMean_.at<double>(0, 0);
#endif
}

/**
 * 予測(事前推定)する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecInputCur 現時刻制御入力ベクトル.
**/
void UnscentedKalmanFilter::Predict(
  const cv::Mat i_vecInputCur)
{
  // arrange
  //-- 変数名短縮用バッファ
  cv::Mat vecb = StateSpaceModel_->ProcessModelEquation_->Vecb_;
  cv::Mat vecStateOld = PosterioriEstimationOld_->VecMean_;
  cv::Mat matVarianceStateOld = PosterioriEstimationOld_->MatVariance_;
  cv::Mat matVarianceNoiseProcess = this->StateSpaceModel_->ProcessModelEquation_->Noise_->MatVariance_;
  //-- 出力結果格納用バッファ
  cv::Mat vecUTransedMeanyy = PosterioriEstimationOld_->VecMean_.clone();
  cv::Mat matUTrancedVarianceyy = PosterioriEstimationOld_->MatVariance_.clone();
  cv::Mat matUTrancedVariancexy = PosterioriEstimationOld_->MatVariance_.clone();

  // act
  //-- U変換1回目(xhat[k-1] -> xhatm[k])
  UnscentedTransform(PosterioriEstimationOld_->VecMean_, PosterioriEstimationOld_->MatVariance_,
    (static_cast<NonLinearProcessModelEquation^>(StateSpaceModel_->ProcessModelEquation_))->function_,
    &vecUTransedMeanyy, &matUTrancedVarianceyy, NULL);
  //---- 事前状態推定: xhatm[k]
  PriorEstimationCur_->VecMean_ = vecUTransedMeanyy.clone();
  //---- 事前誤差共分散: Pm = Σ{wi(χi[k]-xhatm[k])(χi[k]-xhatm[k])'} + B*Q*B'
  PriorEstimationCur_->MatVariance_ = matUTrancedVarianceyy.clone() + vecb*matVarianceNoiseProcess*vecb.t();

  //-- U変換2回目(xhatm[k] -> yhatm[k])
  UnscentedTransform(PriorEstimationCur_->VecMean_, PriorEstimationCur_->MatVariance_,
    (static_cast<NonLinearObservationModelEquation^>(StateSpaceModel_->ObservationModelEquations_[0]))->function_,
    &vecUTransedMeanyy, &matUTrancedVarianceyy, &matUTrancedVariancexy);
  //---- 事前出力推定値: yhatm[k]
  priorEstimationOutputCur_->VecMean_ = vecUTransedMeanyy.clone();
  //---- 事前出力誤差共分散行列: Pyym[k]
  priorEstimationOutputCur_->MatVariance_ = matUTrancedVarianceyy.clone();
  //---- 事前状態・出力誤差共分散行列: Pxym[k]
  priorEstimationStateAndOutputCur_->MatVariance_ = matUTrancedVariancexy.clone();
}

/**
 * フィルタリング(事後推定)する.
 *
 * @author  Morita
 * @date  2015/06/17
**/
void UnscentedKalmanFilter::Filter()
{
  // arrange
  List<ObservationModelEquation^>^ nonlinearObservationModelEquations
    = StateSpaceModel_->ObservationModelEquations_;
  cv::Mat matPm = PriorEstimationCur_->MatVariance_;
  cv::Mat matC = nonlinearObservationModelEquations[0]->MatC_;
  cv::Mat matVarianceObserve = StateSpaceModel_->ObservationModelEquations_[0]->Noise_->MatVariance_;
  cv::Mat matPriorVarianceStateAndOutput = priorEstimationStateAndOutputCur_->MatVariance_;
  cv::Mat matPriorVarianceOutput = priorEstimationOutputCur_->MatVariance_;
  //-- カルマンゲイン: G = P-*C/(C'*P-*C + R)
  cv::Mat matG = matPriorVarianceStateAndOutput / (matPriorVarianceOutput + matVarianceObserve);

  // act
  //-- 事後状態推定値: xhat_k = xhat-_k + G*(y-C'xhat-_k)
  //---- イノベーション過程 = y - h(xhat-_k)
  cv::Mat vecObservationCur = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_;
  cv::Mat vecInnovation = vecObservationCur - priorEstimationOutputCur_->VecMean_;

  //---- xhat_k = xhat-_k + G*innovaion
  PosterioriEstimationCur_->VecMean_ = PriorEstimationCur_->VecMean_ + matG * vecInnovation;

  //-- 事後誤差共分散: P_k = (I - G*C')*Pm
  PosterioriEstimationCur_->MatVariance_
    = PriorEstimationCur_->MatVariance_ - matG*priorEstimationStateAndOutputCur_->MatVariance_.t();

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = matG.at<double>(0, 0);
  result[debugIndex++] = VecObservationCur_.at<double>(0, 0);
  result[debugIndex++] = PriorEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matC.at<double>(0, 0);
  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
  result[debugIndex++] = matPm.at<double>(0, 0);
  result[debugIndex++] = PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
#endif
}

/**
 * アンセンテッド変換.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_xMean           入力分布平均.
 * @param i_Pxx             入力分布誤差分散.
 * @param i_function        変換関数．y=f(x).
 * @param [in,out]  o_yMean 出力分布平均.
 * @param [in,out]  o_Pyy   出力分布誤差分散.
 * @param [in,out]  o_Pxy   入出力分布共分散.
**/
void UnscentedKalmanFilter::UnscentedTransform(
  const cv::Mat i_xMean,
  const cv::Mat i_Pxx,
  ModelFunc^ i_function,
  cv::Mat* o_yMean,
  cv::Mat* o_Pyy,
  cv::Mat* o_Pxy)
{
  // arrange
  const double xdof = 1.; // 状態ベクトル次元数
  const double kappa = 3. - xdof; // スケーリングパラメータ
  std::vector<double> w(1 + 2 * (int)xdof); // 重み
  w[0] = kappa / (xdof + kappa);
  w[1] = 1 / (2 * (xdof + kappa));
  w[2] = w[1];

  // act
  //-- シグマポイントの生成
  cv::Mat SQRTMat = cv::Mat_<double>(1, 1) << sqrt(i_Pxx.at<double>(0, 0)); // 平方根行列

  std::vector<cv::Mat> sigmaPointsOld(1 + 2 * (int)xdof);
  sigmaPointsOld[0] = i_xMean;
  sigmaPointsOld[1] = i_xMean + sqrt(xdof + kappa)*SQRTMat;
  sigmaPointsOld[2] = i_xMean - sqrt(xdof + kappa)*SQRTMat;

  //-- シグマポイントの更新: y = f(x)
  std::vector<cv::Mat> sigmaPointsNew(1 + 2 * (int)xdof);
  cv::Mat i_vecInputCur = cv::Mat_<double>(1, 1) << 0.0;
  for (int i = 0; i < 3; i++)
    sigmaPointsNew[i] = i_function(sigmaPointsOld[i], TimeCur_);

  //-- 重み付平均: y_mean[k]
  cv::Mat yTemp = cv::Mat::zeros(1, 1, CV_64F);
  for (int i = 0; i < 3; i++)
    yTemp += w[i] * sigmaPointsNew[i];

  if (o_yMean == NULL)  // NULLポインタなら確保して返してあげる
    o_yMean = new cv::Mat(yTemp.clone());
  else  // 確保されていれば普通にコピー
    *o_yMean = yTemp.clone();

  //-- 出力誤差共分散行列: Pyy
  cv::Mat PyyTemp = cv::Mat::zeros(i_Pxx.rows, i_Pxx.cols, CV_64F);
  for (int i = 0; i < 3; i++)
  {
    cv::Mat erryy = sigmaPointsNew[i] - yTemp;
    PyyTemp += w[i] * erryy * erryy.t();
  }

  if (o_Pyy == NULL)  // NULLポインタなら確保して返してあげる
    o_Pyy = new cv::Mat(PyyTemp.clone());
  else  // 確保されていれば普通にコピー
    *o_Pyy = PyyTemp.clone();

  ////
  // 以下、o_Pxy がNULLポインタなら計算もしない。
  if (o_Pxy == NULL)  // o_Pxy は省略可能。
    return;

  //-- 入出力出力誤差共分散行列: Pxy[k]
  *o_Pxy = cv::Mat::zeros(1, 1, CV_64F);
  for (int i = 0; i < 3; i++)
  {
    cv::Mat errxx = sigmaPointsOld[i] - i_xMean;
    cv::Mat erryy = sigmaPointsNew[i] - *o_yMean;
    *o_Pxy += w[i] * errxx * erryy.t();
  }

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  for (int i = 0; i < 3; i++)
    result[debugIndex++] = sigmaPointsOld[i].at<double>(0, 0);

  for (int i = 0; i < 3; i++)
    result[debugIndex++] = sigmaPointsNew[i].at<double>(0, 0);

  result[debugIndex++] = yTemp.at<double>(0, 0);

  result[debugIndex++] = PyyTemp.at<double>(0, 0);

  result[debugIndex++] = o_Pxy->at<double>(0, 0);
#endif
}

/**
 * Cholesky 変換.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param src           変換元行列.
 * @param [in,out]  dst 変換元行列.
**/
void UnscentedKalmanFilter::Cholesky(
  const cv::Mat src,
  cv::Mat* dst)
{
  double dstTemp[3 * 3] = { 0 };

  dstTemp[0 * 3 + 0] = sqrt(src.at<double>(0, 0));
  dstTemp[1 * 3 + 0] = src.at<double>(1, 0) / dstTemp[0 * 3 + 0];
  dstTemp[2 * 3 + 0] = src.at<double>(2, 0) / dstTemp[0 * 3 + 0];

  dstTemp[1 * 3 + 1] = sqrt(src.at<double>(1, 1) - dstTemp[1 * 3 + 0] * dstTemp[1 * 3 + 0]);
  dstTemp[2 * 3 + 1] = sqrt(src.at<double>(2, 1) - dstTemp[2 * 3 + 0] * dstTemp[1 * 3 + 0]) / dstTemp[1 * 3 + 1];
  dstTemp[2 * 3 + 2] = sqrt(src.at<double>(2, 2) - dstTemp[2 * 3 + 0] * dstTemp[2 * 3 + 0]);

  *dst = cv::Mat(3, 3, CV_32F, dstTemp).clone();

  // debugs
#if MY_DEBUG
  double result[3][3] = { 0 };

  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < 3; i++)
    {
      result[j][i] = dst->at<double>(j, i)
    }
  }
#endif
}

/**
 * シミュレーションを行い, 結果を返す.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_count             全シミュレーション時間.
 * @param o_estimationResult  状態推定結果.
**/
void UnscentedKalmanFilter::Loop(
  const int i_count,
  EstimationResult ^o_estimationResult)
{
  // 入力を常にゼロにするシミュレーション
  cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

  TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
  List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(this->StateSpaceModel_, i_count + 1);
#if MY_DEBUG
  TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
  List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(this, i_count + 1);

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
    this->TimeCur_ = k + 1;
    // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
    // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
    //this->UpdateToNextTime(vecInputCur);
    this->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_);
    this->Estimate(vecInputCur); // 入力値を元に推定
    this->AcquireEstimationResult(o_estimationResult);  // 結果を引数オブジェクトに格納
  }
}
