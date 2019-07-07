/*
====================================================================================================
Project Name    : state_estimate_filter
File Name       : state_estimate_filter.h
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
#include "state_space_model.h"
//
using namespace System;
using namespace System::Collections::Generic;
using namespace System::Xml::Serialization;
using namespace System::IO;
using namespace System::Text;
using std::vector;

namespace state_estimate_filter
{
  /**
   * @brief 真値及び観測値格納クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class TruthAndObservation
  {
  public:

    /**
     * メンバイニシャライザ.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_truth       真値.
     * @param i_observation 観測値. 複数センサを配列で表現.
    **/
    TruthAndObservation(
      const double i_truth,
      List<double>^ i_observations)
    {
      truth_ = i_truth;
      // センサ数分コピー
      for (int i = 0; i < i_observations->Count; i++)
        observations_->Add(i_observations[i]);
    };

  public:
    /** 真値.*/
    double truth_;
    /** 観測値. センサ数を配列で表現. */
    List<double>^ observations_ = gcnew List<double>;
  };

  /**
   * @brief 状態推定結果一式を格納するクラス. 外部に渡したい値は全てここに格納する.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class EstimationResult
  {
  public:
    /** The save directory. */
    literal String^ kSaveDir = "result";
    /** The default file name. */
    literal String^ kFileNameDefault = "estimation_result.xml";

  public:
    EstimationResult(){};
    /**
     * リファレンスを変えずにフィールドを更新する.
     * (機能としては問題ないが, かっこ悪い感じがするので良い手段があれば置き換えたい.)
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_estimationResult  コピー元オブジェクト.
    **/
    void SetAllFeildsNoRefChange(
      EstimationResult^ i_estimationResult)
    {
      posterioriEstimations = i_estimationResult->posterioriEstimations;
      priorEstimations = i_estimationResult->priorEstimations;
      posterioriErrorDeviations = i_estimationResult->posterioriErrorDeviations;
      trueValues = i_estimationResult->trueValues;
      observationsMultiSensor = i_estimationResult->observationsMultiSensor;
      varObservationsMultiSensor = i_estimationResult->varObservationsMultiSensor;
      errsBtwnPosterioriEstimationAndTruth = i_estimationResult->errsBtwnPosterioriEstimationAndTruth;
      errsBtwnPosterioriEstimationAndObservation = i_estimationResult->errsBtwnPosterioriEstimationAndObservation;
      posterioriParticlesStates = i_estimationResult->posterioriParticlesStates;
      posterioriParticlesWeights = i_estimationResult->posterioriParticlesWeights;
      priorParticlesStates = i_estimationResult->priorParticlesStates;
      priorParticlesWeights = i_estimationResult->priorParticlesWeights;
      resampleBit = i_estimationResult->resampleBit;
      statesObserve = i_estimationResult->statesObserve;
      posterioriPfMap = i_estimationResult->posterioriPfMap;
      deviance_ = i_estimationResult->deviance_;
      likelihoodsH0_ = i_estimationResult->likelihoodsH0_;
      likelihoodsH1_ = i_estimationResult->likelihoodsH1_;
    }

    /**
     * 推定結果をファイルで保存する. モジュールパス\result\指定ファイル名.xml.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_fileName  保存ファイル名.
    **/
    void Save(
      String^ i_fileName)
    {
      //保存先のファイル名
      if (i_fileName == nullptr)
        i_fileName = gcnew String(kFileNameDefault);

      // result フォルダの作成
      Directory::CreateDirectory(kSaveDir);

      //＜XMLファイルに書き込む＞
      //XmlSerializerオブジェクトを作成
      //書き込むオブジェクトの型を指定する
      XmlSerializer^ serializer = gcnew XmlSerializer(EstimationResult::typeid);
      //ファイルを開く（UTF-8 BOM無し）
      String^ filePath = kSaveDir + "\\" + i_fileName;
      StreamWriter^ sw = gcnew StreamWriter(filePath, false, gcnew UTF8Encoding(false));
      //シリアル化し、XMLファイルに保存する
      serializer->Serialize(sw, this);
      //閉じる
      sw->Close();
    }

    /**
     * 推定結果をファイルで読込む. モジュールパス\result\指定ファイル名.xml.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_fileName  読込ファイル名.
    **/
    void Load(
      String^ i_fileName)
    {
      //保存元のファイル名
      if (i_fileName == nullptr)
        return;

      // result フォルダの作成
      Directory::CreateDirectory(kSaveDir);

      //XmlSerializerオブジェクトを作成
      XmlSerializer^ serializer = gcnew XmlSerializer(EstimationResult::typeid);
      //読み込むファイルを開く
      String^ filePath = kSaveDir + "\\" + i_fileName;
      StreamReader^ sr = gcnew StreamReader(filePath, gcnew UTF8Encoding(false));
      //XMLファイルから読み込み、逆シリアル化する
      EstimationResult^ obj = static_cast<EstimationResult^>(serializer->Deserialize(sr));
      // メンバに格納
      Copy(obj);
      //ファイルを閉じる
      sr->Close();
    }

  private:
    /**
     * オブジェクトのフィールドをコピーする.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_obj コピー元オブジェクト.
    **/
    void Copy(
      EstimationResult^ i_obj)
    {
      posterioriEstimations = gcnew List<double>(i_obj->posterioriEstimations);
      posterioriErrorDeviations = gcnew List<double>(posterioriErrorDeviations);
      trueValues = gcnew List<double>(i_obj->trueValues);
      observationsMultiSensor = gcnew List<List<double>^>(i_obj->observationsMultiSensor);
      errsBtwnPosterioriEstimationAndTruth = gcnew List<double>(i_obj->errsBtwnPosterioriEstimationAndTruth);
      errsBtwnPosterioriEstimationAndObservation = gcnew List<double>(i_obj->errsBtwnPosterioriEstimationAndObservation);
    };

  public:
    /** 全時刻-事後状態推定確率分布_推定値. */
    List<double>^ posterioriEstimations = gcnew List<double>;

    /** 全時刻-事後状態推定確率分布_推定値. */
    List<double>^ priorEstimations = gcnew List<double>;

    /** 全時刻-事後状態推定確率分布_推定誤差事後分散. */
    List<double>^ posterioriErrorDeviations = gcnew List<double>;

    /** 全時刻-真値. */
    List<double>^ trueValues = gcnew List<double>;

    /** 全時刻-観測. 複数センサ. */
    List<List<double>^>^ observationsMultiSensor = gcnew List<List<double>^>;

    /** 全時刻-観測分散. 複数センサ. */
    List<List<double>^>^ varObservationsMultiSensor = gcnew List<List<double>^>;
    
    /** 事後状態推定値と真値の誤差. */
    List<double>^ errsBtwnPosterioriEstimationAndTruth = gcnew List<double>;

    /** 事後状態推定値から予測した観測値と実観測値の誤差. */
    List<double>^ errsBtwnPosterioriEstimationAndObservation = gcnew List<double>;

    // PF用
    /** 全時刻-事後粒子状態推定確率分布_状態. */
    List<List<double>^>^ posterioriParticlesStates = gcnew List<List<double>^>;

    /** 全時刻-事後粒子状態推定確率分布_重み. */
    List<List<double>^>^ posterioriParticlesWeights = gcnew List<List<double>^>;

    /** 全時刻-事前粒子状態推定確率分布_状態. */
    List<List<double>^>^ priorParticlesStates = gcnew List<List<double>^>;

    /** 全時刻-事前粒子状態推定確率分布_重み. */
    List<List<double>^>^priorParticlesWeights = gcnew List<List<double>^>;

    /** 全時刻-pf MAP 事後推定値*/
    List<List<double>^>^posterioriPfMap = gcnew List<List<double>^>;

    /** 全時刻-リサンプリングタイミングビット. true; 実行, false: 不実行. */
    List<bool>^ resampleBit = gcnew List<bool>;

    /** 観測状態. */
    List<int>^ statesObserve = gcnew List<int>;

    /** 逸脱度. */
    List<double>^ deviance_ = gcnew List<double>;

    /** 対立仮説尤度. */
    List<double>^ likelihoodsH0_ = gcnew List<double>;
    
    /** 帰無仮説尤度. */
    List<double>^ likelihoodsH1_ = gcnew List<double>;

  };

  /**
   * @brief 状態推定フィルタ基底クラス.
   *
   * @author  Morita
   * @date  2015/06/17
  **/
  public ref class AbstractStateEstimateFilter abstract
  {
  public:	// constructors & destoructors
    /**
    * Default constructor.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    AbstractStateEstimateFilter();

    /**
    * Constructor.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_stateSpaceModel  状態空間モデル一式.
    **/
    AbstractStateEstimateFilter(
      StateSpaceModel^ i_stateSpaceModel);

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
    AbstractStateEstimateFilter(
      ProcessModelEquation^ i_processModelEquation
    , List<ObservationModelEquation^>^ i_observationModelEquation
    , const cv::Mat i_vecStateTrueInit);

  public:	// methods
    /**
    * 事前・事後・MAP推定を行う. Template Method パターン.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_vecInputCur 制御入力ベクトル.
    **/
    void Estimate(
      const cv::Mat i_vecInputCur);

    /**
    * 事前・事後・MAP推定を行う. Template Method パターン.
    *
    * @author  Morita
    * @date  2015/11/06
    *
    * @param i_vecInputCur 制御入力ベクトル.
    **/
    void Estimate(
      const double i_vecInputCur);

    /**
     * 一段先予測. 運動モデルに従って予測する.
     *
     * @author  Morita
     * @date  2015/06/17
     *
     * @param i_vecInputCur 制御入力ベクトル.
    **/
    virtual void Predict(
      const cv::Mat i_vecInputCur) = 0;

    /**
     * フィルタリング. 観測の結果を考慮する.
     *
     * @author  Morita
     * @date  2015/06/17
    **/
    virtual void Filter() = 0;

    /**
    * 内部変数を次の時刻のものに更新する. 真値, 観測値等.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_vecInput  入力ベクトル.
    **/
    virtual void UpdateToNextTime(
      const cv::Mat i_vecInput);

    /**
    * 内部変数を次の時刻のものに更新する. 真値, 観測値等.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_vecInput  入力ベクトル.
    **/
    virtual void UpdateToNextTime(
      const double i_vecInput);

    /**
    * 真値, 観測値を外部変数によって更新する.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_truth       真値リスト.
    * @param i_observation 観測値リスト.
    **/
    virtual void UpdateToNextTimeFromOuterValue(
      double i_truth,
      List<double>^ i_observation);

    /**
    * シミュレーションを行い, 結果を返す.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_count             ループ回数.
    * @param i_estimationResult  推定結果.
    **/
    virtual void Loop(
      const int i_count,
      EstimationResult ^i_estimationResult);

    /**
    * シミュレーションを行い, 結果を返す. 真値と観測を外部から与える.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_truthAndObservations  全時刻真値と観測値.
    *
    * @return  推定結果.
    *
    * ### remarks  Morita, 2015/06/17.
    **/
    virtual EstimationResult^ Loop(
      List<TruthAndObservation^>^ i_truthAndObservations);

    /**
    * 推定結果を格納する.
    *
    * @author  Morita
    * @date  2015/06/17
    *
    * @param i_estimationResult  推定結果の格納先バッファ.
    **/
    virtual void AcquireEstimationResult(
      EstimationResult^ i_estimationResult);

    /**
    * パラメータにデフォルト値を設定する.
    *
    * @author  Morita
    * @date  2015/06/17
    **/
    void ConfigureDefaultParameters();

  public:	// properties
    /**
     * プロセスモデルのプロパティ.processModel_のget/setを提供.
     *
     * @return  The state space model.
    **/
    property StateSpaceModel^ StateSpaceModel_
    {
      StateSpaceModel^ get();
      void set(StateSpaceModel^ value);
    }

    /**
     * xhat_k-1 1時刻前-事後状態推定値分布. ^posterioriEstimationOld_のget/setを提供.
     *
     * @return  The posteriori estimation old.
    **/
    property ProbabilityDistribution^ PosterioriEstimationOld_
    {
      ProbabilityDistribution^ get();
      void set(ProbabilityDistribution^ value);
    }

    /**
     * xhat-_k 現時刻-事前状態推定値分布. ^priorEstimationCur_のget/setを提供.
     *
     * @return  The prior estimation current.
    **/
    property ProbabilityDistribution^ PriorEstimationCur_
    {
      ProbabilityDistribution^ get();
      void set(ProbabilityDistribution^ value);
    }

    /**
     * xhat_k 現時刻-事後状態推定値分布. ^posterioriEstimationCur_のget/setを提供.
     *
     * @return  The posteriori estimation current.
    **/
    property ProbabilityDistribution^ PosterioriEstimationCur_
    {
      ProbabilityDistribution^ get();
      void set(ProbabilityDistribution^ value);
    }

    /**
     * 現在時刻のプロパティ. timeCur_のset/getを提供.
     *
     * @return  The time current.
    **/
    property int TimeCur_
    {
      int get();
      void set(int value);
    }

    /**
     * 事後状態推定値と真値の誤差. 事後状態推定値と真値誤差のgetを提供.
     *
     * @return  The error between posteriori estimation and truth.
    **/
    property double ErrorBtwnPosterioriEstimationAndTruth
    {
      double get();
    }

    /**
     * 事後状態推定値から予測した観測値と実観測値の誤差.
     *
     * @return  The error between posteriori estimation and observation.
    **/
    property double ErrorBtwnPosterioriEstimationAndObservation
    {
      virtual double get();
    }

  private:  // fields
    /** システムモデル. */
    StateSpaceModel^ stateSpaceModel_;

    // 状態
    /** xhat_k-1 1時刻前-事後状態推定値分布. */
    ProbabilityDistribution^ posterioriEstimationOld_ = nullptr;
    /** xhat-_k 現時刻-事前状態推定値分布. */
    ProbabilityDistribution^ priorEstimationCur_ = nullptr;
    /** xhat_k 現時刻-事後状態推定値分布. */
    ProbabilityDistribution^ posterioriEstimationCur_ = nullptr;

    /** 現在時刻. */
    int timeCur_ = 0;
  };
}
