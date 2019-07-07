/*
======================================================================
Project Name    : state_estimate_filter
File Name       : normal_distribution.cpp
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

This source code or any portion thereof must not be
reproduced or used in any manner whatsoever.
======================================================================
*/
#include "stdafx.h"
//
#include "normal_distribution.h"
//
using namespace state_estimate_filter;

/**
 * Copy constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param obj コピー元オブジェクト.
**/
NormalDistribution::NormalDistribution(
  NormalDistribution ^obj)
{
  do
  {
    if (obj == nullptr)
      break;
    VecMean_ = obj->VecMean_;
    MatVariance_ = obj->MatVariance_;
  } while (false);
}

/**
 * Constructor.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param [in,out]  obj コピー元オブジェクト.
**/
NormalDistribution::NormalDistribution(
  NormalDistribution% obj)
{
  this->VecMean_ = obj.VecMean_;
  this->MatVariance_ = obj.MatVariance_;
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecMean     期待値ベクトル.
 * @param i_matVariance 共分散行列.
**/
NormalDistribution::NormalDistribution(
  const cv::Mat i_vecMean,
  const cv::Mat i_matVariance) :
  ProbabilityDistribution(
  i_vecMean,
  i_matVariance)
{
}

/**
 * メンバイニシャライザ.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_mean      期待値ベクトル.
 * @param i_variance  共分散行列.
**/
NormalDistribution::NormalDistribution(
  const double i_mean,
  const double i_variance)
  : ProbabilityDistribution(
    i_mean
  , i_variance)
{
}

/**
 * 確率密度関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecValue            関数入力値.
 * @param [in,out]  o_vecResult 関数出力値.
**/
void NormalDistribution::CalcDencityFunction(
  const cv::Mat i_vecValue,
  cv::Mat* o_vecResult)
{
  // result = 1/(sqrt(2π*σ^2)) * exp{-(x-μ)^2/(2σ^2) }
  // result = fact1 * fact3
  // fact1 = 1/(sqrt(2π*σ^2))
  // *fact2 = -(x-μ)^2/(2σ^2)
  // *fact3 = exp ( fact2 )　とおく
  cv::Mat matSqrtedVariance = cv::Mat(MatVariance_.clone());  // clone にしないとdeep copyになる。
  cv::sqrt(MatVariance_, matSqrtedVariance);
  cv::Mat fact1 = 1 / (sqrt(2 * CV_PI) * matSqrtedVariance);

  // queredError = e^2
  cv::Mat squeredError = cv::Mat::zeros(1, 1, CV_64F);
  cv::pow(i_vecValue - VecMean_, 2, squeredError);

  // fact2 = -(x-μ)^2/(2σ^2)
  cv::Mat fact2 = -squeredError / (2 * MatVariance_);

  // fact3 = exp ( fact2 )
  cv::Mat fact3 = cv::Mat::zeros(1, 1, CV_64F);
  cv::exp(fact2, fact3);

  // result = fact1 * fact3
  *o_vecResult = fact1 * fact3;

  // debug
#if 1
  double result[20] = { 0 };
  int debugIndex = 0;
  result[debugIndex++] = i_vecValue.at<double>(0, 0);
  result[debugIndex++] = VecMean_.at<double>(0, 0);
  result[debugIndex++] = fact1.at<double>(0, 0);
  result[debugIndex++] = fact2.at<double>(0, 0);
  result[debugIndex++] = fact3.at<double>(0, 0);
  result[debugIndex++] = o_vecResult->at<double>(0, 0);
#endif
}

/**
 * 対数確率密度関数値を計算する.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @param i_vecValue            関数入力値.
 * @param [in,out]  o_vecResult 関数出力値.
**/
void NormalDistribution::CalcLogDencityFunction(
  const cv::Mat i_vecValue,
  cv::Mat* o_vecResult)
{
  // TODO: 尤度関数をこれに切り替える
  // result = -log(sqrt(2π*σ^2)) - (x-μ)^2/(2σ^2)
  // result = fact1 + fact2
  // fact1 = -log(sqrt(2π)*σ)
  // fact2 = -(x-μ)^2/(2σ^2)　とおく
  cv::Mat matSqrtedVariance = MatVariance_.clone();  // clone にしないとdeep copyになる。
  cv::Mat matLogSqrtedVariance = MatVariance_.clone();  // clone にしないとdeep copyになる。
  cv::sqrt(MatVariance_, matSqrtedVariance);
  cv::log(sqrt(2 * CV_PI) * matSqrtedVariance, matLogSqrtedVariance);
  cv::Mat fact1 = -matLogSqrtedVariance;

  // queredError = e^2
  cv::Mat squeredError = cv::Mat::zeros(1, 1, CV_64F);
  cv::pow(i_vecValue - VecMean_, 2, squeredError);

  // fact2 = -(x-μ)^2/(2σ^2)
  cv::Mat fact2 = -squeredError / (2 * MatVariance_);

  // result = fact1 + fact2
  *o_vecResult = fact1 + fact2;
  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;
  result[debugIndex++] = i_vecValue.at<double>(0, 0);
  result[debugIndex++] = VecMean_.at<double>(0, 0);
  result[debugIndex++] = fact1.at<double>(0, 0);
  result[debugIndex++] = fact2.at<double>(0, 0);
  result[debugIndex++] = o_vecResult->at<double>(0, 0);
#endif
}

double NormalDistribution::CalcDencityFunction(
  const double i_value)
{
  //return exp(result.at<double>(0, 0));
  // result = 1/(sqrt(2π*σ^2)) * exp{-(x-μ)^2/(2σ^2) }
  // result = fact1 * fact3
  // fact1 = 1/(sqrt(2π*σ^2))
  // *fact2 = -(x-μ)^2/(2σ^2)
  // *fact3 = exp ( fact2 )　とおく
  cv::Mat matSqrtedVariance = cv::Mat(MatVariance_.clone());  // clone にしないとdeep copyになる。
  cv::sqrt(MatVariance_, matSqrtedVariance);
  cv::Mat fact1 = 1 / (sqrt(2 * CV_PI) * matSqrtedVariance);

  // queredError = e^2
  cv::Mat vecValue = cv::Mat_<double>(1, 1) << i_value;
  cv::Mat squeredError = cv::Mat::zeros(1, 1, CV_64F);
  cv::pow(vecValue - VecMean_, 2, squeredError);

  // fact2 = -(x-μ)^2/(2σ^2)
  cv::Mat fact2 = -squeredError / (2 * MatVariance_);

  // fact3 = exp ( fact2 )
  cv::Mat fact3 = cv::Mat::zeros(1, 1, CV_64F);
  cv::exp(fact2, fact3);

  // result = fact1 * fact3
  cv::Mat dencity = fact1 * fact3;

  // debug
#if 1
  double result[20] = { 0 };
  int debugIndex = 0;
  result[debugIndex++] = vecValue.at<double>(0, 0);
  result[debugIndex++] = VecMean_.at<double>(0, 0);
  result[debugIndex++] = fact1.at<double>(0, 0);
  result[debugIndex++] = fact2.at<double>(0, 0);
  result[debugIndex++] = dencity.at<double>(0, 0);
#endif

  return dencity.at<double>(0, 0);
}

/**
 * 定義された正規分布に従って乱数を生成する。初期シードは固定.
 *
 * @author  Morita
 * @date  2015/06/17
 *
 * @return  乱数値.
**/
cv::Mat NormalDistribution::GenRandomValue()
{
  cv::Mat vecRandom = cv::Mat::zeros(VecMean_.rows, VecMean_.cols, CV_64F);
  cv::randn(vecRandom, VecMean_, MatVariance_);

  // debug
#if MY_DEBUG
  double result[20] = { 0 };
  int debugIndex = 0;

  result[debugIndex++] = MatVariance_.at<double>(0, 0);
  result[debugIndex++] = vecRandom.at<double>(0, 0);
#endif
  return vecRandom;
}
