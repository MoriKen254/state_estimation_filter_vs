/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : state_space_model.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#include "stdafx.h"
//
#include "state_space_model.h"
//
using namespace state_estimate_filter;

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
StateSpaceModel::StateSpaceModel()
{
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj コピー元オブジェクト.
**/
StateSpaceModel::StateSpaceModel(
  StateSpaceModel% obj)
{
  delete(processModelEquation_);
  observationModelEquations_->Clear();
  delete(vecStateTrueCur_);
  //delete(observationCur_);
  //
  processModelEquation_ = gcnew ProcessModelEquation(*obj.ProcessModelEquation_);  // プロセスモデル式
  observationModelEquations_->Add(gcnew ObservationModelEquation(*obj.ObservationModelEquations_[0]));  // 観測モデル式
  vecStateTrueCur_ = new cv::Mat(obj.VecStateTrueCur_); // 真値

  cv::Mat vecObservationMeanInit = cv::Mat::zeros(1, 1, CV_64F);
  cv::Mat matObservationVarianceInit = obj.ObservationModelEquations_[0]->Noise_->MatVariance_.clone();
  observationModelEquations_[0]->observationCur_ = gcnew ProbabilityDistribution(vecObservationMeanInit, matObservationVarianceInit);
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj コピー元オブジェクト.
**/
StateSpaceModel::StateSpaceModel(
  StateSpaceModel^ obj)
{
  delete(processModelEquation_);
  observationModelEquations_->Clear();
  delete(vecStateTrueCur_);
//  delete(observationCur_);
  //
  processModelEquation_ = gcnew ProcessModelEquation(*obj->ProcessModelEquation_);  // プロセスモデル式
  observationModelEquations_->Add(gcnew ObservationModelEquation(*obj->ObservationModelEquations_[0]));  // 観測モデル式
  vecStateTrueCur_ = new cv::Mat(obj->VecStateTrueCur_); // 真値

  cv::Mat vecObservationMeanInit = cv::Mat::zeros(1, 1, CV_64F);
  cv::Mat matObservationVarianceInit = obj->ObservationModelEquations_[0]->Noise_->MatVariance_.clone();
  observationModelEquations_[0]->observationCur_ = gcnew ProbabilityDistribution(vecObservationMeanInit, matObservationVarianceInit);
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
StateSpaceModel::StateSpaceModel(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_vecStateTrueInit)
  : processModelEquation_(i_processModelEquation)
  , observationModelEquations_(i_observationModelEquation)
{
  vecStateTrueCur_ = new cv::Mat(i_vecStateTrueInit);

  cv::Mat vecObservationMeanInit = cv::Mat::zeros(1, 1, CV_64F);
  cv::Mat matObservationVarianceInit = i_processModelEquation->Noise_->MatVariance_.clone();
  ObservationModelEquations_[0]->observationCur_ = gcnew ProbabilityDistribution(vecObservationMeanInit, matObservationVarianceInit);
}

/**
* 内部状態の真値を更新する(ノイズ含). 内部の真値メンバ変数を更新する.
*
* @author  Morita
* @date  2015/06/17
*
* @param i_inputCur  制御入力ベクトル.
* @param i_time      現在時刻.
**/
void StateSpaceModel::UpdateInnerTrueValueWithNoise(
  const cv::Mat i_inputCur
, const int i_time)
{
}

/**
 * 真値を更新して戻り値として与える(ノイズ含).
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur  制御入力ベクトル.
 * @param i_time      現在時刻.
 *
 * @return  更新後の真値(内部状態は更新しない).
**/
cv::Mat StateSpaceModel::UpdateOuterTrueValueWithNoise(
  const cv::Mat i_inputCur
, const int i_time)
{
  return cv::Mat(1, 1, CV_64F, -99);
}

/**
 * Observe with noise.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_trueCur 現時刻真値.
 * @param i_time    現時刻.
**/
void StateSpaceModel::ObserveWithNoise(
  const cv::Mat i_trueCur,
  const int i_time)
{
  // 実装は観測モデル式クラスに任せる(Bridgeパターン). 観測値はObservationModelEquation毎のメンバに格納される.
  for (int indexSensor = 0; indexSensor < ObservationModelEquations_->Count; indexSensor++)
    ObservationModelEquations_[indexSensor]->ObserveWithNoize(i_trueCur, i_time);
}

/**
 * ノイズ無しの一段先予測を出力する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimatedOld 一時刻前状態推定値.
 * @param i_inputCur          制御入力ベクトル.
 * @param i_time              現在時刻.
 *
 * @return  ノイズ無しの一段先予測計算結果.
**/
cv::Mat StateSpaceModel::PredictPriorStateCurWithoutNoise(
  const cv::Mat i_stateEstimatedOld
, const cv::Mat i_inputCur, const int i_time)
{
  return cv::Mat(1, 1, CV_64F, cv::Scalar(-99));
}

/**
 * ノイズ付の一段先予測を出力する. パーティクルフィルタで用いる.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimatedOld           一時刻前状態推定値.
 * @param i_inputCur                    現時刻入力.
 * @param i_time                        現時刻.
 * @param [in,out]  o_statePredictedCur 一段先予測出力.
**/
void StateSpaceModel::PredictPriorStateCurWithNoise(
  const cv::Mat i_stateEstimatedOld
, const cv::Mat i_inputCur
, const int i_time
, cv::Mat* o_statePredictedCur)
{
}

/**
 * Gets the processModelEquation_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  processModelEquation_.
**/
ProcessModelEquation^ StateSpaceModel::ProcessModelEquation_::get()
{
  return processModelEquation_;
}

/**
 * Sets the given value to processModelEquation_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void StateSpaceModel::ProcessModelEquation_::set(
  ProcessModelEquation^ value)
{
  processModelEquation_ = value;
}

/**
 * Gets the observationModelEquation_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  observationModelEquation_.
**/
List<ObservationModelEquation^>^ StateSpaceModel::ObservationModelEquations_::get()
{
  return observationModelEquations_;
}

/**
 * Sets the given value to observationModelEquation_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void StateSpaceModel::ObservationModelEquations_::set(
  List<ObservationModelEquation^>^ value)
{
  observationModelEquations_ = value;
}

/**
 * Gets the *vecStateTrueCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A *vecStateTrueCur_.
**/
cv::Mat StateSpaceModel::VecStateTrueCur_::get()
{
  return *vecStateTrueCur_;
}

/**
 * Sets the given value to *vecStateTrueCur_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void StateSpaceModel::VecStateTrueCur_::set(
  cv::Mat value)
{
  *vecStateTrueCur_ = value;
}

/**
* デフォルトコンストラクタ.
*
* @author  Morita
* @date  2015/06/17
**/
LinearStateSpaceModel::LinearStateSpaceModel()
{
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj コピー元オブジェクト.
**/
LinearStateSpaceModel::LinearStateSpaceModel(
  LinearStateSpaceModel% obj)
  : StateSpaceModel(obj)
{
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj コピー元オブジェクト.
**/
LinearStateSpaceModel::LinearStateSpaceModel(
  LinearStateSpaceModel^ obj)
  : StateSpaceModel(obj)
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_processModelEquation      プロセスモデル方程式.
 * @param i_observationModelEquation  観測モデル方程式.
 * @param i_trueValue                 初期真値.
**/
LinearStateSpaceModel::LinearStateSpaceModel(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_trueValue)
  : StateSpaceModel(
    i_processModelEquation
  , i_observationModelEquation
  , i_trueValue)
{
}

/**
 * 内部状態を更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur  制御入力ベクトル.
 * @param i_time      現在時刻.
**/
void LinearStateSpaceModel::UpdateInnerTrueValueWithNoise(
  const cv::Mat i_inputCur
, const int i_time)
{
  cv::Mat matA = ProcessModelEquation_->MatA_;
  cv::Mat vecb = ProcessModelEquation_->Vecb_;
  cv::Mat vecStateTrueOld = VecStateTrueCur_;

  // x_k = A*(x_k-1) + b*(u_k) + (vk)
  VecStateTrueCur_ = matA*vecStateTrueOld + vecb*i_inputCur + ProcessModelEquation_->Noise_->GenRandomValue();

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = vecStateTrueOld.at<double>(0, 0); // 一時刻前真値
  result[debugIndex++] = VecStateTrueCur_.at<double>(0, 0);  // 現時刻真値
#endif
}

/**
 * 真値を更新して戻り値として与える(ノイズ含).
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur  制御入力ベクトル.
 * @param i_time      現在時刻.
 *
 * @return  更新後の真値(内部状態は更新しない.)
**/
cv::Mat LinearStateSpaceModel::UpdateOuterTrueValueWithNoise(
  const cv::Mat i_inputCur
, const int i_time)
{
  cv::Mat matA = ProcessModelEquation_->MatA_;
  cv::Mat vecb = ProcessModelEquation_->Vecb_;
  cv::Mat vecStateTrueOld = VecStateTrueCur_;

  // x_k = A*(x_k-1) + b*(u_k) + (vk)
  return matA*vecStateTrueOld + vecb*i_inputCur + ProcessModelEquation_->Noise_->GenRandomValue();

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = vecStateTrueOld.at<double>(0, 0); // 一時刻前真値
  result[debugIndex++] = VecStateTrueCur_.at<double>(0, 0);  // 現時刻真値
#endif
}

/**
 * ノイズ無しの一段先予測を出力する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimatedOld 一時刻前状態推定値.
 * @param i_inputCur          現時刻入力.
 * @param i_time              現時刻.
 *
 * @return  関数計算結果.
**/
cv::Mat LinearStateSpaceModel::PredictPriorStateCurWithoutNoise(
  const cv::Mat i_stateEstimatedOld
, const cv::Mat i_inputCur
, const int i_time)
{
  cv::Mat matA = ProcessModelEquation_->MatA_;
  cv::Mat vecb = ProcessModelEquation_->Vecb_;

  // x_k = A*(x_k-1) + b*(u_k) + (vk)
  return matA*i_stateEstimatedOld + vecb*i_inputCur;
}

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
NonLinearStateSpaceModel::NonLinearStateSpaceModel()
{
}

/**
 * コピーコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj コピー元オブジェクト.
**/
NonLinearStateSpaceModel::NonLinearStateSpaceModel(
  NonLinearStateSpaceModel% obj)
  : StateSpaceModel(obj)
{
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj コピー元オブジェクト.
**/
NonLinearStateSpaceModel::NonLinearStateSpaceModel(
  NonLinearStateSpaceModel^ obj)
  : StateSpaceModel(obj)
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_processModelEquation      プロセスモデル方程式.
 * @param i_observationModelEquation  観測モデル方程式.
 * @param i_trueValue                 初期真値.
**/
NonLinearStateSpaceModel::NonLinearStateSpaceModel(
  ProcessModelEquation^ i_processModelEquation
, List<ObservationModelEquation^>^ i_observationModelEquation
, const cv::Mat i_trueValue)
  : StateSpaceModel(
    i_processModelEquation
  , i_observationModelEquation
  , i_trueValue)
{
}

/**
 * 真値を更新する(ノイズ含). 内部の真値メンバ変数を更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur  制御入力ベクトル.
 * @param i_time      現在時刻.
**/
void NonLinearStateSpaceModel::UpdateInnerTrueValueWithNoise(
  const cv::Mat i_inputCur
, const int i_time)
{
  // x_k = f(x_k-1) + (vk): 真値にはプロセスノイズが入る
  VecStateTrueCur_ = ProcessModelEquation_->CalcFunctionWithNoise(VecStateTrueCur_, i_time);

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = VecStateTrueCur_.at<double>(0, 0); // 真値
#endif
}

/**
 * 真値を更新して戻り値として与える(ノイズ含).
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur  制御入力ベクトル.
 * @param i_time      現在時刻.
 *
 * @return  更新後の真値(内部状態は更新しない.)
**/
cv::Mat NonLinearStateSpaceModel::UpdateOuterTrueValueWithNoise(
  const cv::Mat i_inputCur
, const int i_time)
{
  // x_k = f(x_k-1) + (vk): 真値にはプロセスノイズが入る
  return ProcessModelEquation_->CalcFunctionWithNoise(VecStateTrueCur_, i_time);

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = VecStateTrueCur_.at<double>(0, 0); // 真値
#endif
}

/**
 * モデルを更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur          制御入力ベクトル.
 * @param i_stateEstimatedCur 現時刻状態推定値.
 * @param i_time              現時刻.
**/
void NonLinearStateSpaceModel::UpdateModel(
  const cv::Mat i_inputCur
, const cv::Mat i_stateEstimatedCur
, const int i_time)
{
  // 1次近似プロセスモデルの更新
  ProcessModelEquation_->UpdateModel(i_inputCur, i_stateEstimatedCur, i_time);
  // 1次近似観測モデルの更新(制御入力は観測モデルに寄与しない)
  ObservationModelEquations_[0]->UpdateModel(cv::Mat::zeros(0, 0, CV_64F), i_stateEstimatedCur, i_time);
}

/**
 * ノイズ無しの一段先予測を出力する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimatedOld 一時刻前状態推定値.
 * @param i_inputCur          現時刻入力.
 * @param i_time              現時刻.
 *
 * @return  ノイズ無しの一段先予測を出力結果.
**/
cv::Mat NonLinearStateSpaceModel::PredictPriorStateCurWithoutNoise(
  const cv::Mat i_stateEstimatedOld
, const cv::Mat i_inputCur
, const int i_time)
{
  return ProcessModelEquation_->CalcFunctionWithoutNoise(i_stateEstimatedOld, i_time);
}

/**
 * ノイズ付の一段先予測を出力する。パーティクルフィルタで用いる.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateEstimatedOld           一時刻前状態推定値.
 * @param i_inputCur                    現時刻入力.
 * @param i_time                        現時刻.
 * @param [in,out]  o_statePredictedCur 状態推定結果.
**/
void NonLinearStateSpaceModel::PredictPriorStateCurWithNoise(
  const cv::Mat i_stateEstimatedOld
, const cv::Mat i_inputCur
, const int i_time
, cv::Mat* o_statePredictedCur)
{
  *o_statePredictedCur = ProcessModelEquation_->CalcFunctionWithNoise(i_stateEstimatedOld, i_time);
}
