/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : state_estimater_factory.h
Encoding        : UTF-8 (CR LF)
Creation Date   : 2015/06/14

Copyright (c) <2015> Masaru Morita. All rights reserved.

Released under the MIT license
http://opensource.org/licenses/mit-license.php
====================================================================================================
*/
#pragma once
//
#include "state_estimate_filters.h"

namespace state_estimate_filter
{
  public ref class ModelFunctionSet abstract
  {
  public: // methods
    /**
    * プロセスモデル関数.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    virtual cv::Mat Function(
      const cv::Mat i_x
      , const int i_time) = 0;

  public: // fields
    /** プロセスモデル導関数. */
    List<ModelFunc^>^ derivedFunctions_ = gcnew List<ModelFunc^>();
  };

  /**
  * @brief プロセスモデル関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetProcess abstract : ModelFunctionSet
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetProcess()
    {
      matA = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      vecb = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
    };

  public: // methods

  public: // fields
    /** A行列. */
    cv::Mat* matA;
    /** bベクトル. */
    cv::Mat* vecb;
  };

  /**
  * @brief プロセスモデル関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetObserve abstract : ModelFunctionSet
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetObserve()
    {
      matC = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
    };

  public: // methods

  public: // fields
    /** C行列. */
    cv::Mat* matC;
  };

  /**
  * @brief ランダムウォークプロセス関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetProcessRandomWalk : ModelFunctionSetProcess
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetProcessRandomWalk()
      : ModelFunctionSetProcess()
    {
      matA = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      vecb = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      //
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetProcessRandomWalk::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * ランダムウォークプロセスモデル関数: f(x[k]) = x[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      return i_x;
    };

    /**
    * ランダムウォークプロセスモデルヤコビアン: f'(x[k]) = 1.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      return cv::Mat::ones(1, 1, CV_64F);
    };
  };

  /**
  * @brief ランダムウォーク観測モデル関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetObserveRandomWalk : ModelFunctionSetObserve
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetObserveRandomWalk()
      : ModelFunctionSetObserve()
    {
      matC = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      //
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetObserveRandomWalk::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * ランダムウォーク観測モデル関数: h(x[k]) = x[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      return i_x;
    };

    /**
    * ランダムウォーク観測モデルヤコビアン: h'(x[k]) = x[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      return cv::Mat::ones(1, 1, CV_64F);
    };
  };

  /**
  * @brief 等速直線運動関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetProcessLinearUniformMotion : ModelFunctionSetProcess
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetProcessLinearUniformMotion()
      : ModelFunctionSetProcess()
    {
      matA = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      vecb = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      //
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetProcessLinearUniformMotion::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * 等速直線運動プロセス関数: f(x[k]) = x[k] + 1.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      return i_x + 1;
    };

    /**
    * 等速直線運動プロセスモデルヤコビアン: f'(x[k]) = 1.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      return cv::Mat::ones(1, 1, CV_64F);
    };
  };

  /**
  * @brief 等速直線運動観測モデル関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetObserveLinearUniformMotion : ModelFunctionSetObserve
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetObserveLinearUniformMotion()
      : ModelFunctionSetObserve()
    {
      matC = new cv::Mat((cv::Mat_<double>(1, 1) << 1.0));
      //
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetObserveLinearUniformMotion::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * 等速直線運動観測モデル関数: h(x[k]) = x[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      return i_x;
    };

    /**
    * 等速直線運動観測モデルヤコビアン: h'(x[k]) = x[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      return cv::Mat::ones(1, 1, CV_64F);
    };
  };

  /**
  * @brief ベンチマーク用非線形プロセス関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetProcessBenchMark : ModelFunctionSetProcess
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetProcessBenchMark()
      : ModelFunctionSetProcess()
    {
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetProcessBenchMark::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * ベンチマーク用非線形プロセスモデル関数: f(x[k]) = 0.2*x[k] + 25*x[k]/(1+x[k]*x[k]) + 8*cos(1.2*k) + v[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      cv::Mat vecCos = cv::Mat::zeros(1, 1, CV_64F);
      vecCos.at<double>(0, 0) = cv::cos(1.2*i_time);
      cv::Mat den = cv::Mat::ones(1, 1, CV_64F) + i_x*i_x;

      return (0.2*i_x + 25 * i_x / den + 8 * vecCos);
    };

    /**
    * ベンチマーク用非線形プロセスモデル導関数: f'(x[k]) = 0.2 + 25*(1-x[k]*x[k])/{(1+x[k]*x[k])*(1+x[k]*x[k])} + v[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      cv::Mat vecCos = cv::Mat::zeros(1, 1, CV_64F);
      cv::Mat den = (cv::Mat::ones(1, 1, CV_64F) + i_x*i_x) * (cv::Mat::ones(1, 1, CV_64F) + i_x*i_x);

      return (0.2 + 25 * (cv::Mat::ones(1, 1, CV_64F) - i_x*i_x) / den);
    };
  };

  /**
  * @brief ベンチマーク用非線形プロセス関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetObserveBenchMark : ModelFunctionSetObserve
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetObserveBenchMark()
      : ModelFunctionSetObserve()
    {
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetObserveBenchMark::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * / <summary>
    *  ベンチマーク用非線形観測モデル関数: h(x[k]) = 0.05*(x[k])^2 + w[k]
    * / </summary>
    * / <param name="i_x">状態ベクトル</param>
    * / <param name="i_time">現在時刻</param>
    * / <returns>関数計算結果</returns>
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     Zero-based index of the x coordinate.
    * @param i_time  Zero-based index of the time.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      cv::Mat vecPow = cv::Mat::zeros(1, 1, CV_64F);
      vecPow.at<double>(0, 0) = cv::pow(i_x.at<double>(0, 0), 2);
      return 0.05*vecPow;
    };

    /**
    * ベンチマーク用非線形観測モデル導関数: h'(x[k]) = 0.1*(x[k]) + w[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      return 0.1*i_x;
    };
  };

  /**
  * @brief EKF演習用非線形プロセス関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetProcessEKFSample : ModelFunctionSetProcess
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetProcessEKFSample()
      : ModelFunctionSetProcess()
    {
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetProcessEKFSample::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * EKF演習用非線形プロセスモデル関数: f(x[k]) = x[k] + 3cos{(x[k])/10} + v[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      cv::Mat vecCos = cv::Mat::zeros(1, 1, CV_64F);
      vecCos.at<double>(0, 0) = cv::cos(i_x.at<double>(0, 0) / 10.);
      return i_x + 3 * vecCos;
    };

    /**
    * EKF演習用非線形プロセスモデルヤコビアン: f'(x[k]) = 1 - (3/10)sin{(x[k])/10}
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      cv::Mat vecSin = cv::Mat::zeros(1, 1, CV_64F);
      vecSin.at<double>(0, 0) = cv::sin(i_x.at<double>(0, 0) / 10.);
      return cv::Mat::ones(1, 1, CV_64F) - vecSin*(3. / 10.);
    };
  };

  /**
  * @brief EKF演習用非線形プロセス関数.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelFunctionSetObserveEKFSample : ModelFunctionSetObserve
  {
  public: // constructors
    /**
    * デフォルトコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    ModelFunctionSetObserveEKFSample()
      : ModelFunctionSetObserve()
    {
      ModelFunc^ derivedFunction = gcnew ModelFunc(this, &ModelFunctionSetObserveEKFSample::DerivedFunction);
      derivedFunctions_->Add(derivedFunction);
    };

  public: // methods
    /**
    * EKF演習用非線形観測モデル関数: h(x[k]) = (x[k])^3 + w[k].
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat Function(
      const cv::Mat i_x,
      const int i_time) override
    {
      cv::Mat vecPow = cv::Mat::zeros(1, 1, CV_64F);
      vecPow.at<double>(0, 0) = cv::pow(i_x.at<double>(0, 0), 3);
      return vecPow;
    };

    /**
    * EKF演習用非線形観測モデルヤコビアン: h'(x[k]) = 3(x[k])^2.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_x     状態ベクトル.
    * @param i_time  現在時刻.
    *
    * @return  関数計算結果.
    **/
    cv::Mat DerivedFunction(
      const cv::Mat i_x,
      const int i_time)
    {
      cv::Mat vecPow = cv::Mat::zeros(1, 1, CV_64F);
      vecPow.at<double>(0, 0) = cv::pow(i_x.at<double>(0, 0), 2);
      return 3 * vecPow;
    };
  };

  /**
   * @brief 確率分布の基本パラメータ.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class DistributionParam
  {
  public: // constructors

    /**
     * Default constructor.
     *
     * @author  Morita
     * @date  2015/06/23
    **/
    DistributionParam(){};

    /**
     * メンバイニシャライザ.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_mean      期待値.
     * @param i_variance  分散.
    **/
    DistributionParam(
      const double i_mean,
      const double i_variance)
    {
      mean_ = i_mean;
      variance_ = i_variance;
    };

    /**
     * コピーコンストラクタ.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param obj コピー元オブジェクト.
    **/
    DistributionParam(
      DistributionParam ^obj)
    {
      mean_ = obj->mean_;
      variance_ = obj->variance_;
    }

  public: // fields
    /** 期待値. */
    double mean_;
    /** 分散. */
    double variance_;
  };

  /**
  * @brief モデルの基本パラメータ.
  *
  * @author  Morita
  * @date  2015/06/17
  **/
  public ref class ModelParam
  {
  public: // constructors

    /**
     * Default constructor.
     *
     * @author  Morita
     * @date  2015/06/23
    **/
    ModelParam()
    {
    };

    /**
     * Constructor.
     *
     * @author  Morita
     * @date  2015/06/20
     *
     * @param i_modelFunctionSet  モデル関数及びヤコビアン一式.
     * @param i_mean      期待値.
     * @param i_variance  分散.
    **/
    ModelParam(
      ModelFunctionSet^ i_modelFunctionSet,
      const double i_noizeMean,
      const double i_noizeVariance)
    {
      modelFunctionSet_ = i_modelFunctionSet;
      // モデル関数
      function_ = gcnew ModelFunc(i_modelFunctionSet, &ModelFunctionSet::Function);
      // モデル導関数
      derivations_ = i_modelFunctionSet->derivedFunctions_;
      // 期待値と分散
      noizeMean_ = i_noizeMean;
      noizeVariance_ = i_noizeVariance;
    };

    /**
    * コピーコンストラクタ.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param obj コピー元オブジェクト.
    **/
    ModelParam(
      DistributionParam ^obj)
    {
      noizeMean_ = obj->mean_;
      noizeVariance_ = obj->variance_;
    }

  public: // fields
    /** モデル関数. */
    ModelFunc^ function_;
    /** モデル導関数. */
    List<ModelFunc^>^ derivations_;
    ModelFunctionSet^ modelFunctionSet_;
    /** ノイズ期待値. */
    double noizeMean_;
    /** ノイズ分散. */
    double noizeVariance_;
  };

  /**
   * @brief 状態推定フィルタを使用するために必要なパラメータ一式格納クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class StateEstimaterParam
  {
  public: // constructors

    /**
     * デフォルトコンストラクタ.
     *
     * @author  Morita
     * @date  2015/06/17
    **/
    StateEstimaterParam(){};

    /**
     * メンバイニシャライザ.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_posteriorEstimationStateInit  事後推定初期値.
     * @param i_stateTrueInit                 真値初期値.
     * @param i_modelParamProcess             プロセスモデルパラメータ.
     * @param i_modelParamObservation         観測モデルパラメータ. 複数センサ
    **/
    StateEstimaterParam(
      DistributionParam ^i_posteriorEstimationStateInit,
      const double i_stateTrueInit,
      ModelParam^ i_modelParamProcess,
      List<ModelParam^>^ i_modelParamsObservation)
        : posteriorEstimationStateInit_(i_posteriorEstimationStateInit)
        , stateTrueInit_(i_stateTrueInit)
        , modelParamProcess_(i_modelParamProcess)
        , modelParamsObservation_(i_modelParamsObservation)
    {
    };

    /**
     * メンバイニシャライザ.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_posteriorEstimationStateInit  事後推定初期値.
     * @param i_stateTrueInit                 真値初期値.
     * @param i_modelParamProcess             プロセスモデルパラメータ.
     * @param i_modelParamsObservation        観測モデルパラメータ. 複数センサ.
     * @param i_countParticles                パーティクル数.
     * @param i_pointEstimation               点推定アルゴリズムオブジェクト.
     * @param i_randomDistributionForPredict  パーティクル事前推定時の乱数分散.
     * @param i_resampling                    リサンプリングアルゴリズムオブジェクト.
    **/
    StateEstimaterParam(
      DistributionParam ^i_posteriorEstimationStateInit,
      const double i_stateTrueInit,
      ModelParam^ i_modelParamProcess,
      List<ModelParam^>^ i_modelParamsObservation,
      const int i_countParticles,
      AbstractPointEstimation^ i_pointEstimation,
      DistributionParam^ i_randomDistributionForPredict,
      AbstractResampling^ i_resampling)
        : posteriorEstimationStateInit_(i_posteriorEstimationStateInit),
          stateTrueInit_(i_stateTrueInit),
          modelParamProcess_(i_modelParamProcess),
          modelParamsObservation_(i_modelParamsObservation),
          countParticles_(i_countParticles),
          pointEstimation_(i_pointEstimation),
          randomDistributionForPredict_(i_randomDistributionForPredict),
          resampling_(i_resampling)
    {
    };

  public: // method

    /**
     * 参照を変えずにフィールドを更新する.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_posteriorEstimationStateInit  事後推定初期値.
     * @param i_stateTrueInit                 真値初期値.
     * @param i_modelParamProcess             プロセスモデル関数パラメータ.
     * @param i_modelParamsObservation         観測モデル関数パラメータ. 複数センサ
     * @param i_countParticles                パーティクル数.
     * @param i_pointEstimation               点推定アルゴリズムオブジェクト.
     * @param i_resampling                    リサンプリングアルゴリズムオブジェクト.
    **/
    void SetParametersNoReferenceChange(
      DistributionParam ^i_posteriorEstimationStateInit,
      const double i_stateTrueInit,
      ModelParam^ i_modelParamProcess,
      List<ModelParam^>^ i_modelParamsObservation,
      const int i_countParticles,
      AbstractPointEstimation^ i_pointEstimation,
      DistributionParam^ i_randomDistributionForPredict,
      AbstractResampling^ i_resampling)
    {
      posteriorEstimationStateInit_ = i_posteriorEstimationStateInit;
      stateTrueInit_ = i_stateTrueInit;
      modelParamProcess_ = i_modelParamProcess;
      modelParamsObservation_ = i_modelParamsObservation;
      countParticles_ = i_countParticles;
      pointEstimation_ = i_pointEstimation;
      randomDistributionForPredict_ = i_randomDistributionForPredict;
      resampling_ = i_resampling;
    }

  public: // fields
    /** 事後推定初期. */
    DistributionParam ^posteriorEstimationStateInit_;
    /** 真値初期値. */
    double stateTrueInit_;
    /** プロセスモデル. */
    ModelParam^ modelParamProcess_;
    /** 観測モデル. 複数センサ. */
    List<ModelParam^>^ modelParamsObservation_;
    /** パーティクル数. */
    int countParticles_;
    /** 点推定アルゴリズム. */
    AbstractPointEstimation^ pointEstimation_;
    /** パーティクル予測時乱数分布(プロセルモデル誤差とは別に用意) */
    DistributionParam^ randomDistributionForPredict_;
    /** リサンプリングアルゴリズムオブジェクト. */
    AbstractResampling^ resampling_;
  };

  /**
   * @brief 状態推定フィルタ　ファクトリ基底クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class AbstractStateEstimateFilterFactory abstract
  {
  public: // methods

    /**
     * 状態推定フィルタオブジェクトを生成する.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_stateEstimateParam  状態推定フィルタパラメータ一式.
     *
     * @return  状態推定フィルタオブジェクト.
    **/
    virtual AbstractStateEstimateFilter^ Create(
      StateEstimaterParam^ i_stateEstimateParam) = 0;

    virtual void PrepareParamsForStateSpaceModel(
      StateEstimaterParam^ i_stateEstimateParam);

    virtual ProcessModelEquation^ CreateProcessModelEquation(
      StateEstimaterParam^ i_stateEstimateParam);

    virtual ObservationModelEquation^ CreateObservationModelEquation(
      ModelParam^ i_modelParamObservation);

    virtual ProbabilityDistribution^ CreatePosterioriEstimationInit(
      DistributionParam^ i_distParamPosteriorEstimationInit);

  protected:
    /** プロセスモデル式. */
    ProcessModelEquation^ processModelEquation_ = nullptr;
    /** 観測モデル式. */
    List<ObservationModelEquation^>^ observationModelEquation_ = gcnew List<ObservationModelEquation^>();
    /** 真値初期値. */
    cv::Mat* vecStateTrueInit_ = NULL;
  };

  /**
   * @brief 線形カルマンフィルタ　ファクトリクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class LinearKalmanFilterFactory : AbstractStateEstimateFilterFactory
  {
  public: // methods
    AbstractStateEstimateFilter^ Create(
      StateEstimaterParam^ i_stateEstimateParam) override;
    
    ProcessModelEquation^ CreateProcessModelEquation(
      StateEstimaterParam^ i_stateEstimateParam) override;

    ObservationModelEquation^ CreateObservationModelEquation(
      ModelParam^ i_modelParamObservation) override;
  };

  /**
   * @brief 拡張カルマンフィルタ　ファクトリクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ExtendedKalmanFilterFactory : AbstractStateEstimateFilterFactory
  {
  public: // methods
    AbstractStateEstimateFilter^ Create(
      StateEstimaterParam^ i_stateEstimateParam) override;
  };

  /**
   * @brief アンセンテッドフィルタ　ファクトリクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class UnscentedKalmanFilterFactory : AbstractStateEstimateFilterFactory
  {
  public: // methods
    AbstractStateEstimateFilter^ Create(
      StateEstimaterParam^ i_stateEstimateParam) override;
  };

  /**
   * @brief パーティクルフィルタ　ファクトリクラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class ParticleFilterFactory : AbstractStateEstimateFilterFactory
  {
  public: // methods
    AbstractStateEstimateFilter^ Create(
      StateEstimaterParam^ i_stateEstimateParam) override;

    ProbabilityDistribution^ CreatePosterioriEstimationInit(
      DistributionParam^ i_distParamPosteriorEstimationInit) override;

    ProbabilityDistribution^ CreateRandomDistributionForPredict(
      DistributionParam^ i_distParamRandomPredict);

  private:  // fields
    /** パーティクル数. */
    int countParticles_;
  };
}
