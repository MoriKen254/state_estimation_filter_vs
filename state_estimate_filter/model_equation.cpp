/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : model_equation.cpp
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
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
ModelEquation::ModelEquation()
{
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj The object.
**/
ModelEquation::ModelEquation(
  ModelEquation% obj)
{
  // ポリモフィジックな動作をさせるためポインタをコピーすること
  noize_ = obj.Noise_;
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_noize_  プロセスノイズ分布.
**/
ModelEquation::ModelEquation(
  ProbabilityDistribution ^i_noize_)
{
  // ポリモフィジックな動作をさせるためポインタをコピーすること
  noize_ = i_noize_;
}

/**
 * ノイズ付で関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecState  現時刻状態.
 * @param i_time      現時刻.
 *
 * @return  関数出力値.
**/
cv::Mat ModelEquation::CalcFunctionWithNoise(
  const cv::Mat i_vecState
, const int i_time)
{
  return cv::Mat::zeros(0, 0, CV_64F);
}

/**
 * ノイズ無で関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecState  現時刻状態.
 * @param i_time      現時刻.
 *
 * @return  関数出力値.
**/
cv::Mat ModelEquation::CalcFunctionWithoutNoise(
  const cv::Mat i_vecState,
  const int i_time)
{
  return cv::Mat::zeros(0, 0, CV_64F);
}

/**
 * 次時刻に更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur          現時刻入力.
 * @param i_stateEstimatedCur 現時刻状態推定値.
 * @param i_time              現時刻.
**/
void ModelEquation::UpdateModel(
  const cv::Mat i_inputCur,
  const cv::Mat i_stateEstimatedCur,
  const int i_time)
{
}

/**
 * 代入演算子.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  rhs 代入元オブジェクト.
 *
 * @return  代入元オブジェクトのコピー.
**/
ModelEquation ModelEquation::operator = (ModelEquation% rhs)
{ 
  return ModelEquation(rhs); 
}

/**
 * Gets the noize_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  noize_.
**/
ProbabilityDistribution^ ModelEquation::Noise_::get()
{
  return noize_;
};

/**
 * Sets the given value to noize_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ModelEquation::Noise_::set(
  ProbabilityDistribution^ value)
{
  noize_ = value;
};

/**
 * Default constructor.
 *
 * @author  Morita
 * @date  2015/06/17
**/
ProcessModelEquation::ProcessModelEquation()
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
ProcessModelEquation::ProcessModelEquation(
  ProcessModelEquation% obj)
  : ModelEquation(obj)
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_noize_  プロセスノイズ分布.
**/
ProcessModelEquation::ProcessModelEquation(
  ProbabilityDistribution ^i_noize_) :
  ModelEquation(i_noize_)
{
}

/**
 * Gets the *matA_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A *matA_.
**/
cv::Mat ProcessModelEquation::MatA_::get()
{
  // NULLポインタの時は-1を返す
  if (matA_ == NULL)
    return cv::Mat::ones(1, 1, CV_64F) * -1;

  return *matA_;
}

/**
 * Sets the given value to *matA_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ProcessModelEquation::MatA_::set(
  const cv::Mat value)
{
  // NULL ポインタの時は生成
  if (matA_ == NULL)
  {
    matA_ = new cv::Mat(value);
    return;
  }
  // 確保済みの時はコピー
  *matA_ = value;
}

/**
 * Gets the *vecb_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A cv::Mat.
**/
cv::Mat ProcessModelEquation::Vecb_::get()
{
  // NULLポインタの時は-1を返す
  if (vecb_ == NULL)
    return cv::Mat::ones(1, 1, CV_64F) * -1;

  return *vecb_;
}

/**
 * Sets the given value to *vecb_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ProcessModelEquation::Vecb_::set(
  const cv::Mat value)
{
  // NULL ポインタの時は生成
  if (vecb_ == NULL)
  {
    vecb_ = new cv::Mat(value);
    return;
  }
  // 確保済みの時はコピー
  *vecb_ = value;
}

/**************************************************************************************************/

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
LinearProcessModelEquation::LinearProcessModelEquation()
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_matA  A行列.
 * @param i_vecb  Bベクトル.
 * @param i_noize プロセスノイズ分布.
**/
LinearProcessModelEquation::LinearProcessModelEquation(
  const cv::Mat i_matA
, const cv::Mat i_vecb
, ProbabilityDistribution ^i_noize)
  : ProcessModelEquation(
      i_noize)
{
  MatA_ = i_matA;
  Vecb_ = i_vecb;
}

/**************************************************************************************************/

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
NonLinearProcessModelEquation::NonLinearProcessModelEquation()
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_function    プロセスモデル関数.
 * @param i_fJacobians  プロセスモデルヤコビアン関数.
 * @param i_vecb        入力bベクトル.
 * @param i_noize       ノイズ分布.
**/
NonLinearProcessModelEquation::NonLinearProcessModelEquation(
  ModelFunc^ i_function
, List<ModelFunc^>^ i_fJacobians
, const cv::Mat i_vecb
, ProbabilityDistribution^ i_noize)
  : function_(i_function)
  , ProcessModelEquation(i_noize)
{
  fJacobians_ = i_fJacobians;//gcnew List<ModelFunc^>(i_fJacobians);
  MatA_ = cv::Mat::zeros(i_vecb.rows, i_vecb.rows, CV_64F);
  Vecb_ = i_vecb;
}

/**
 * ノイズ付で関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateTrueCur  現時刻状態.
 * @param i_time          現時刻.
 *
 * @return  関数出力値.
**/
cv::Mat NonLinearProcessModelEquation::CalcFunctionWithNoise(
  const cv::Mat i_stateTrueCur,
  const int i_time)
{
  // x_k+1 = h(x_k) + w_k
#if MY_DEBUG
  double input = i_stateTrueCur.at<double>(0, 0);
  double output = (function_(i_stateTrueCur, i_time)).at<double>(0, 0);
  cv::Mat noize = Noise_->GenRandomValue();
  double noized = noize.at<double>(0, 0);
#endif
  return function_(i_stateTrueCur, i_time) + Noise_->GenRandomValue();
}

/**
 * ノイズ無で関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_stateOld  一時刻前状態.
 * @param i_time      現時刻.
 *
 * @return  関数出力値.
**/
cv::Mat NonLinearProcessModelEquation::CalcFunctionWithoutNoise(
  const cv::Mat i_stateOld,
  const int i_time)
{
  if (function_ == nullptr)
    return cv::Mat::ones(1, 1, CV_64F)*(-1);

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = i_stateOld.at<double>(0, 0);
  result[debugIndex++] = (function_(i_stateOld, i_time)).at<double>(0, 0);
#endif

  return function_(i_stateOld, i_time);
}

/**
 * 次時刻に更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur          現時刻入力.
 * @param i_stateEstimatedCur 現時刻状態推定値.
 * @param i_time              現時刻.
**/
void NonLinearProcessModelEquation::UpdateModel(
  const cv::Mat i_inputCur,
  const cv::Mat i_stateEstimatedCur,
  const int i_time)
{
  if (fJacobians_ == nullptr)
    return;

  for (int i = 0; i < MatA_.rows; i++)
  {
    for (int j = 0; j < MatA_.cols; j++)
    {
      MatA_ = fJacobians_[j + i * 3](i_stateEstimatedCur, i_time);

      // debug
#if MY_DEBUG
      double result[20] = { 0 };
      int debugIndex = 0;

      result[debugIndex++] = i_stateEstimatedCur.at<double>(0, 0);
      result[debugIndex++] = MatA_.at<double>(0, 0);
#endif
    }
  }
}

/**************************************************************************************************/

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
ObservationModelEquation::ObservationModelEquation()
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
ObservationModelEquation::ObservationModelEquation(
  ObservationModelEquation% obj)
: ModelEquation(obj)
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_noize_  観測ノイズ分布.
**/
ObservationModelEquation::ObservationModelEquation(
  ProbabilityDistribution ^i_noize_)
: ModelEquation(i_noize_)
{
}

/**
 * ノイズ無しで観測を行い, 観測値メンバ変数を更新する. 
 *
 * @author  Morita
 * @date  2015/06/20
 *
 * @param i_vecStateTrue  状態推定真値.
 * @param i_time          現在時刻.
**/
void ObservationModelEquation::ObserveWithNoize(
  const cv::Mat i_vecStateTrue,
  const int i_time)
{
  observationCur_->VecMean_ = CalcFunctionWithNoise(i_vecStateTrue, i_time);
};

/**
 * Gets the *matC_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  A *matC_.
**/
cv::Mat ObservationModelEquation::MatC_::get()
{
  // NULLポインタの時は-1を返す
  if (matC_ == NULL)
    return cv::Mat::ones(1, 1, CV_64F) * -1;

  return *matC_;
}

/**
 * Sets the given value to *matC_.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param value The value to set.
**/
void ObservationModelEquation::MatC_::set(
  const cv::Mat value)
{
  // NULL ポインタの時は生成
  if (matC_ == NULL)
  {
    matC_ = new cv::Mat(value);
    return;
  }
  // 確保済みの時はコピー
  *matC_ = value;
}

/**************************************************************************************************/

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
LinearObservationModelEquation::LinearObservationModelEquation()
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_matC  C行列.
 * @param i_noize 観測ノイズ分布.
**/
LinearObservationModelEquation::LinearObservationModelEquation(
  const cv::Mat i_matC
, ProbabilityDistribution ^i_noize)
  : ObservationModelEquation(i_noize)
{
  MatC_ = i_matC;
}

/**
 * 現時刻真値と観測ノイズを元に, 観測値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecStateTrue  状態推定真値.
 * @param i_time          現在時刻.
 *
 * @return  関数計算結果.
**/
cv::Mat LinearObservationModelEquation::CalcFunctionWithNoise(
  const cv::Mat i_vecStateTrue
, const int i_time)
{
  // y_k = c*x_k + w_k
  return MatC_*i_vecStateTrue + this->Noise_->GenRandomValue();
}

/**************************************************************************************************/

/**
 * デフォルトコンストラクタ.
 *
 * @author  Morita
 * @date  2015/06/17
**/
NonLinearObservationModelEquation::NonLinearObservationModelEquation()
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_function    観測モデル関数.
 * @param i_fJacobians  観測モデルヤコビアン関数リスト(各関数を各変数で偏微分した分用意)
 * @param i_noize       観測ノイズ分布.
**/
NonLinearObservationModelEquation::NonLinearObservationModelEquation(
  ModelFunc^ i_function
, List<ModelFunc^>^ i_fJacobians
, ProbabilityDistribution^ i_noize)
  : function_(i_function)
  , ObservationModelEquation(i_noize)
{
  const int dof = 1;
  fJacobians_ = i_fJacobians;// gcnew List<ModelFunc^>(i_fJacobians);
  MatC_ = cv::Mat::zeros(dof, dof, CV_64F);
}


/**
 * ノイズ無で関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecState  現時刻状態.
 * @param i_time      現時刻.
 *
 * @return  関数出力値.
**/
cv::Mat NonLinearObservationModelEquation::CalcFunctionWithoutNoise(
  const cv::Mat i_vecState,
  const int i_time)
{
  // y_k = h(x_k)
#if MY_DEBUG
  double input = i_vecState.at<double>(0, 0);
  double output = (function_(i_vecState, i_time)).at<double>(0, 0);
#endif
  return function_(i_vecState, i_time);
}

/**
 * ノイズ付で関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecStateTrue  現時刻状態.
 * @param i_time          現時刻.
 *
 * @return  関数出力値.
**/
cv::Mat NonLinearObservationModelEquation::CalcFunctionWithNoise(
  const cv::Mat i_vecStateTrue,
  const int i_time)
{
  // y_k = h(x_k) + w_k
#if MY_DEBUG
  double input = i_vecStateTrue.at<double>(0, 0);
  double output = (function_(i_vecStateTrue, i_time)).at<double>(0, 0);
  cv::Mat noize = Noise_->GenRandomValue();
  double noized = noize.at<double>(0, 0);
#endif
  return function_(i_vecStateTrue, i_time) + Noise_->GenRandomValue();
}

/**
 * 次時刻に更新する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_inputCur          現時刻入力.
 * @param i_stateEstimatedCur 現時刻状態推定値.
 * @param i_time              現時刻.
**/
void NonLinearObservationModelEquation::UpdateModel(
  const cv::Mat i_inputCur
, const cv::Mat i_stateEstimatedCur
, const int i_time)
{
  // C行列の更新(ヤコビアン)
  if (fJacobians_ == nullptr)
    MatC_ = cv::Mat::ones(1, 1, CV_64F);

  for (int i = 0; i < MatC_.rows; i++)
  {
    for (int j = 0; j < MatC_.cols; j++)
    {
      MatC_ = fJacobians_[j + i * 3](i_stateEstimatedCur, i_time);

      // debug
#if MY_DEBUG
      double result[20] = { 0 };
      int debugIndex = 0;

      result[debugIndex++] = i_stateEstimatedCur.at<double>(0, 0);
      result[debugIndex++] = MatC_.at<double>(0, 0);
#endif      
    }
  }
}
