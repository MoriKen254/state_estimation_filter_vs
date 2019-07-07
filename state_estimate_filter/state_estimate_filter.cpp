/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : state_estimate_filter.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "state_estimate_filter.h"
#include "truth_observation_generator.h"
//
using namespace state_estimate_filter;


AbstractStateEstimateFilter::AbstractStateEstimateFilter()
 : stateSpaceModel_(
    gcnew StateSpaceModel())
{
  ConfigureDefaultParameters();
}

AbstractStateEstimateFilter::AbstractStateEstimateFilter(
  StateSpaceModel^ i_stateSpaceModel)
  : stateSpaceModel_(
      i_stateSpaceModel)
{
  ConfigureDefaultParameters();
}

AbstractStateEstimateFilter::AbstractStateEstimateFilter(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_vecStateTrueInit)
  : stateSpaceModel_(
      gcnew StateSpaceModel(
        i_processModelEquation
      , i_observationModelEquation, i_vecStateTrueInit))
{
  ConfigureDefaultParameters();
}

void AbstractStateEstimateFilter::Estimate(
  const cv::Mat i_vecInputCur)
{
  this->Predict(i_vecInputCur);  // 一段先予測
  this->Filter(); // フィルタリング
}

void AbstractStateEstimateFilter::UpdateToNextTime(
  const cv::Mat i_vecInput)
{
  // 現時刻事後推定値　→　一時刻前事後推定値にシフト
  PosterioriEstimationOld_ = gcnew ProbabilityDistribution(PosterioriEstimationCur_);

  // 真値更新(状態空間モデル内の真値を更新する)
  StateSpaceModel_->UpdateInnerTrueValueWithNoise(i_vecInput, TimeCur_);

  // 観測値更新(状態推定フィルタクラスの観測値を更新する)
  StateSpaceModel_->ObserveWithNoise(StateSpaceModel_->VecStateTrueCur_, TimeCur_);  // 観測

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0); // 一時刻シフト
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);  // 真値
  result[debugIndex++] = stateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);  // 観測
#endif
}

void AbstractStateEstimateFilter::UpdateToNextTimeFromOuterValue(
  double i_truth,
  List<double>^ i_observations)
{
  // 現時刻事後推定値　→　一時刻前事後推定値にシフト
  PosterioriEstimationOld_ = gcnew ProbabilityDistribution(PosterioriEstimationCur_);

  // 真値更新(状態空間モデル内の真値を更新する)
  StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0) = i_truth;

  // 観測値更新(状態推定フィルタクラスの観測値を更新する)
  for (int i = 0; i < StateSpaceModel_->ObservationModelEquations_->Count; i++)
    StateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0) = i_observations[i];

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = PosterioriEstimationCur_->VecMean_.at<double>(0, 0); // 一時刻シフト
  result[debugIndex++] = StateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);  // 真値
  result[debugIndex++] = StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);  // 観測
#endif
}

void AbstractStateEstimateFilter::ConfigureDefaultParameters()
{
  // arrange
  //-- プロセス分布初期値
  cv::Mat vecMeanProcessInit = cv::Mat::zeros(1, 1, CV_64F);  // 平均0
  cv::Mat MatVarianceProcessInit = cv::Mat::zeros(1, 1, CV_64F); // 分散0
  //-- 観測分布初期値
  //cv::Mat vecObservationModelEquations_[0]->observationCur_ = cv::Mat::zeros(1, 1, CV_64F);

  // act
  //-- xhat_k-1 1時刻前-事後状態推定値分布
  if (PosterioriEstimationOld_ == nullptr) // NULLポインタの時だけ上書きする
    PosterioriEstimationOld_ = gcnew ProbabilityDistribution(vecMeanProcessInit, MatVarianceProcessInit);

  //-- xhat-_k 現時刻-事前状態推定値分布 
  // PosterioriEstimationOld_ で初期化する
  PriorEstimationCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);

  //-- xhat_k 現時刻-事後状態推定値分布
  // PosterioriEstimationOld_ で初期化する
  PosterioriEstimationCur_ = gcnew ProbabilityDistribution(PosterioriEstimationOld_);
}

void AbstractStateEstimateFilter::Loop(
  const int i_count
, EstimationResult^ i_estimationResult)
{
  // 入力は常に0
  cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

  TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
  List<TruthAndObservation^>^ truthAndObservations =
    truthAndObservationGenerator->Generate(this->stateSpaceModel_, i_count + 1);
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

EstimationResult^ AbstractStateEstimateFilter::Loop(
  List<TruthAndObservation^>^ i_truthAndObservations)
{
  const int countTime = i_truthAndObservations->Count;
  // 入力を常にゼロにするシミュレーション
  cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

  EstimationResult^ estimationResult = gcnew EstimationResult;

  // パーティクルフィルタ開始。時刻k=1からk=4まで実施   
  for (int k = 0; k < countTime; k++)
  {
    this->TimeCur_ = k + 1;
    // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
    // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
    //this->UpdateToNextTime(vecInputCur);
    this->UpdateToNextTimeFromOuterValue(i_truthAndObservations[k]->truth_, i_truthAndObservations[k]->observations_);
    this->Estimate(vecInputCur); // 入力値を元に推定
    this->AcquireEstimationResult(estimationResult);  // 結果を引数オブジェクトに格納
  }

  return estimationResult;
}

void AbstractStateEstimateFilter::AcquireEstimationResult(
  EstimationResult^ i_estimationResult)
{
  // arrange
  double posterioriEstimation = posterioriEstimationCur_->VecMeanList_[0];// VecMean_.at<double>(0, 0);
  double posterioriErrorCovariance = posterioriEstimationCur_->MatVariance_.at<double>(0, 0);
  double trueValue = stateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);
  //-- 観測値. 複数センサ対応. 
  List<double>^ obsevationsMultiSensor = gcnew List<double>;
  for (int i = 0; i < stateSpaceModel_->ObservationModelEquations_->Count; i++)
    obsevationsMultiSensor->Add(stateSpaceModel_->ObservationModelEquations_[i]->observationCur_->VecMean_.at<double>(0, 0));

  // act
  //-- 事後推定値
  i_estimationResult->posterioriEstimations->Add(posterioriEstimation);
  i_estimationResult->posterioriErrorDeviations->Add(sqrt(posterioriErrorCovariance));
  //-- 真値
  i_estimationResult->trueValues->Add(trueValue);
  //-- 観測値. 複数センサ対応. 
  i_estimationResult->observationsMultiSensor->Add(obsevationsMultiSensor);
  //-- 事後状態推定値と真値の誤差
  i_estimationResult->errsBtwnPosterioriEstimationAndTruth->Add(ErrorBtwnPosterioriEstimationAndTruth);
  //-- 事後状態推定値から予測した観測値と実観測値の誤差
  i_estimationResult->errsBtwnPosterioriEstimationAndObservation->Add(ErrorBtwnPosterioriEstimationAndObservation);
}

/**
 * Gets the stateSpaceModel_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  processModel_.
**/
StateSpaceModel^ AbstractStateEstimateFilter::StateSpaceModel_::get()
{ 
  return stateSpaceModel_; 
}

/**
 * Sets the given value to stateSpaceModel_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void AbstractStateEstimateFilter::StateSpaceModel_::set(
  StateSpaceModel^ value)
{ 
  stateSpaceModel_ = value; 
}

/**
 * Gets the posterioriEstimationOld_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  posterioriEstimationOld_.
**/
ProbabilityDistribution^ AbstractStateEstimateFilter::PosterioriEstimationOld_::get()
{
  // NULLポインタ時は-1を返す
  if (posterioriEstimationOld_ == nullptr)
    return gcnew ProbabilityDistribution(cv::Mat::ones(1, 1, CV_64F)*(-1), cv::Mat::ones(1, 1, CV_64F)*(-1));

  return posterioriEstimationOld_;
}

/**
 * Sets the given value to posterioriEstimationOld_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void AbstractStateEstimateFilter::PosterioriEstimationOld_::set(
  ProbabilityDistribution^ value)
{
  if (value == nullptr)
    return;

  posterioriEstimationOld_ = value;
}

/**
 * Gets the priorEstimationCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  priorEstimationCur_.
**/
ProbabilityDistribution^ AbstractStateEstimateFilter::PriorEstimationCur_::get()
{
  // NULLポインタ時は-1を返す
  if (priorEstimationCur_ == nullptr)
    return gcnew ProbabilityDistribution(cv::Mat::ones(1, 1, CV_64F)*(-1), cv::Mat::ones(1, 1, CV_64F)*(-1));

  return priorEstimationCur_;
}

/**
 * Sets the given value to priorEstimationCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void AbstractStateEstimateFilter::PriorEstimationCur_::set(
  ProbabilityDistribution^ value)
{
  if (value == nullptr)
    return;

  priorEstimationCur_ = value;
}

/**
 * Gets the posterioriEstimationCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  posterioriEstimationCur_.
**/
ProbabilityDistribution^ AbstractStateEstimateFilter::PosterioriEstimationCur_::get()
{
  // NULLポインタ時は-1を返す
  if (posterioriEstimationCur_ == nullptr)
    return gcnew ProbabilityDistribution(cv::Mat::ones(1, 1, CV_64F)*(-1), cv::Mat::ones(1, 1, CV_64F)*(-1));

  return posterioriEstimationCur_;
}

/**
 * Sets the given value to posterioriEstimationCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void AbstractStateEstimateFilter::PosterioriEstimationCur_::set(
  ProbabilityDistribution^ value)
{
  if (value == nullptr)
    return;

  posterioriEstimationCur_ = value;
}

/**
 * Gets the timeCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  An int.
**/
int AbstractStateEstimateFilter::TimeCur_::get()
{ 
  return timeCur_; 
}

/**
 * Sets the given value to value.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void AbstractStateEstimateFilter::TimeCur_::set(
  int value)
{ 
  timeCur_ = value; 
}

/**
 * Gets the error between posteriori estimation and truth.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  An error between posteriori estimation and truth..
**/
double AbstractStateEstimateFilter::ErrorBtwnPosterioriEstimationAndTruth::get()
{
  // arrange
  double posterioriEstimation = posterioriEstimationCur_->VecMean_.at<double>(0, 0);
  double truth = stateSpaceModel_->VecStateTrueCur_.at<double>(0, 0);

  // act
  return (truth - posterioriEstimation);
}

/**
 * Gets the error between posteriori estimation and observation.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A error between posteriori estimation and observation.
**/
double AbstractStateEstimateFilter::ErrorBtwnPosterioriEstimationAndObservation::get()
{
  // arrange
  //-- 事後状態推定値から予測した観測値
  cv::Mat vecPosterioriEstimationAndObservation =
    stateSpaceModel_->ObservationModelEquations_[0]->CalcFunctionWithoutNoise(posterioriEstimationCur_->VecMean_/*stateSpaceModel_->VecStateTrueCur_*/, timeCur_);
  //-- 事後状態推定値から予測した観測値(doulbe)
  double posterioriEstimationAndObservation = vecPosterioriEstimationAndObservation.at<double>(0, 0);
  //-- 実観測値
  double observation = stateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);

  // act
  return (observation - posterioriEstimationAndObservation);
}
