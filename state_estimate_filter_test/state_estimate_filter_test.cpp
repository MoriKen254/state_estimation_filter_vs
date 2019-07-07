#include "stdafx.h"
#include <vector>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/core/core.hpp>

#include "state_estimate_filter\state_estimate_filters.h"
#include "state_estimate_filter\state_estimater_factory.h"
#include "state_estimate_filter\low_variance_sampling.h"
#include "state_estimate_filter\truth_observation_generator.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace Microsoft::VisualStudio::TestTools::UnitTesting;

using namespace state_estimate_filter;

namespace state_estimate_filter_test
{
	[TestClass]
	public ref class UnitTest
	{
	private:
		TestContext^ testContextInstance;

	public: 
		/// <summary>
		///現在のテストの実行についての情報および機能を
		///提供するテスト コンテキストを取得または設定します。
		///</summary>
		property Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ TestContext
		{
			Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ get()
			{
				return testContextInstance;
			}
			System::Void set(Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ value)
			{
				testContextInstance = value;
			}
		};

		#pragma region Additional test attributes
		//
		//テストを作成する際には、次の追加属性を使用できます:
		//
		//クラス内で最初のテストを実行する前に、ClassInitialize を使用してコードを実行してください
		//[ClassInitialize()]
		//static void MyClassInitialize(TestContext^ testContext) {};
		//
		//クラス内のテストをすべて実行したら、ClassCleanup を使用してコードを実行してください
		//[ClassCleanup()]
		//static void MyClassCleanup() {};
		//
		//各テストを実行する前に、TestInitialize を使用してコードを実行してください
		//[TestInitialize()]
		//void MyTestInitialize() {};
		//
		//各テストを実行した後に、TestCleanup を使用してコードを実行してください
		//[TestCleanup()]
		//void MyTestCleanup() {};
		//
		#pragma endregion 

    /////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////0
    [TestMethod]
    void TestNormalDistribution()
    {
      // arrange
      // 平均3, 標準偏差2の正規分布
      cv::Mat vecMean = (cv::Mat_<double>(1, 1) << 3);
      cv::Mat matVariance = (cv::Mat_<double>(1, 1) << 2 * 2);
      NormalDistribution normalDistribution(vecMean, matVariance);
      Assert::AreEqual(vecMean.at<double>(0, 0), normalDistribution.VecMean_.at<double>(0, 0));
      Assert::AreEqual(matVariance.at<double>(0, 0), normalDistribution.MatVariance_.at<double>(0, 0));

      // act
      cv::Mat vecValue = (cv::Mat_<double>(1, 1) << 2);
      cv::Mat vecResult = cv::Mat::zeros(1, 1, CV_64F);
      normalDistribution.CalcDencityFunction(vecValue, &vecResult);

      // assert
      // 答えは約0.176。許容誤差は0.001。
      double expected = 0.176;
      double actual = vecResult.at<double>(0, 0);
      double tolerance = 0.001;

      Assert::AreEqual(expected, actual, tolerance);

      // 対数版
      normalDistribution.CalcLogDencityFunction(vecValue, &vecResult);
      expected = -1.7373;
      actual = vecResult.at<double>(0, 0);
      Assert::AreEqual(expected, actual, tolerance);


      NormalDistribution normalDistribution2(3, 2 * 2);
      // assert
      // 答えは約0.176。許容誤差は0.001。
      expected = 0.176;
      actual = normalDistribution2.CalcDencityFunction(2);
      Assert::AreEqual(expected, actual, tolerance);
    }

    /////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////0
    [TestMethod]
    void TestLinearKalmanFilter()
    {
      // パラメータ設定
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 0.);  // 事後推定初期値: 平均11, 分散1
      double stateTureInit = 0.;  // モデル真値初期値: 0

      // モデルはランダムウォーク
      //-- プロセス
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessRandomWalk(), 0., 1.);
      //-- 観測
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveRandomWalk(), 0., 2.));

      // パラメータ格納
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
        posterioriEstimationInit,
        stateTureInit, 
        modelParamProcess, 
        modelParamsObservation);

      // 線形カルマンフィルタをファクトリクラスで生成
      AbstractStateEstimateFilterFactory^ factory = gcnew LinearKalmanFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);
      
      // 答え
      cv::Point3d Answers[4] =
      {
        // xhat-[k],   p-[k],    y[k],    g[k],     xhat[k],  p[k]
        { /*   -,       1,     0.3162,*/  1 / 3,    0.1054,  0.6667 },
        { /*   -,      5/3,   -1.5581,*/  5 / 11,  -0.6507,  0.9091 },
        { /*   -,     21/11,  -0.0182,*/ 21 / 43,  -0.3418,  0.9767 },
        { /*   -,     85/43,   0.4997,*/ 85 / 171,  0.0765,  0.9942 },
      };

      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // カルマンフィルタ開始。時刻k=1からk=4まで実施   
      for (int k = 0; k < 4; k++)
      { 
        // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
        // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
        filter->UpdateToNextTime(vecInputCur);
        filter->Estimate(vecInputCur); // 入力値を元に推定

        // Assert
        double tolerance = 0.001; // 許容誤差
        //-- 事後推定値
        double expected = Answers[k].y;
        double actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        //-- 誤差共分散
        expected = Answers[k].z;
        actual = filter->PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }

    [TestMethod]
    void TestEKFFunctions()
    {
      cv::Mat posteriorOld = (cv::Mat_<double>(1, 1) << 11.0);
      cv::Mat priorCur = cv::Mat::zeros(1, 1, CV_64F);
      cv::Mat matACur = cv::Mat::zeros(1, 1, CV_64F);
      const int k = 0;
      const double tolerance = 0.001;
#if MODELDEL
      FunctionList^ functionList = gcnew EKFTestFunctionList();

      priorCur = functionList->ProcessFuntion(posteriorOld, k);
      double expected = 12.36078836;
      double actual = priorCur.at<double>(0, 0);
      Assert::AreEqual(expected, actual, tolerance);

      matACur = functionList->ProcessDerivedFuntion(priorCur, k);
      expected = 0.716649;
      actual = matACur.at<double>(0, 0);
      Assert::AreEqual(expected, actual, tolerance);
#endif
    }

    [TestMethod]
    void TestExtendedKalmanFilter()
    {
      // パラメータ設定
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(11., 1.);  // 事後推定初期値: 平均11, 分散1
      double stateTureInit = 10.;  // モデル真値初期値: 10

      // モデルはカルマンフィルタの基礎で出てきたEKFのサンプル関数
      //-- プロセス: ノイズ　平均0, 分散1
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessEKFSample(), 0., 1.);
      //-- 観測: ノイズ　平均0, 分散100
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveEKFSample(), 0., 100.));

      // パラメータ格納
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
        posterioriEstimationInit, 
        stateTureInit,
        modelParamProcess, 
        modelParamsObservation);

      // 拡張カルマンフィルタをファクトリクラスで生成
      AbstractStateEstimateFilterFactory^ factory = gcnew ExtendedKalmanFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);

      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter->StateSpaceModel_, 4);

      // 入力は常に0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // 答え
      cv::Point3d Answers[4] =
      {
        // xhat-[k],   p-[k],     y[k],        g[k],       xhat[k],                     p[k]
        { /*   -,*/  1.513586, /*0.3162,*/   0.002181, 11.6990182094355    /*, 0.000475812953301316*/ },
        { /*   -,*/  1.000241, /*-1.5581,*/  0.002012, 12.0700238407473    /*, 0.000404857301792413*/ },
        { /*   -,*/  1.000204, /*-0.0182,*/  0.001931, 14.4789625719249    /*, 0.000372860020947514*/ },
        { /*   -,*/  1.000183, /*0.4997,*/   0.001512, 14.2262173029902    /*, 0.000228630591662933*/ },
      };

      // 拡張カルマンフィルタ開始。時刻k=1からk=4まで実施   
      for (int k = 0; k < 4; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
        // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
        //filter->UpdateToNextTime(vecInputCur);
        filter->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_);
        filter->Estimate(vecInputCur); // 入力値を元に推定

        // Assert
        double tolerance = 0.001; // 許容誤差
        //-- 事前誤差共分散
        //double expected = Answers[k].x;
        //double actual = (static_cast<ExtendedKalmanFilter^>(extendedKalmanFilter)->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        //Assert::AreEqual(expected, actual, tolerance);
        //-- 事後推定値
        double expected = Answers[k].z;
        double actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        
      }
    }

    [TestMethod]
    void TestUnscentedKalmanFilter()
    {
      // パラメータ設定
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 1.);  // 事後推定初期値: 平均0, 分散1
      double stateTureInit = 0.;  // モデル真値初期値: 0

      // モデルはベンチマーク非線形関数
      //-- プロセス: ノイズ　平均0, 分散1
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessBenchMark(), 0., 1.);
      //-- 観測: ノイズ　平均0, 分散3
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 0., 3.));

      // パラメータ格納
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
        posterioriEstimationInit,
        stateTureInit,
        modelParamProcess,
        modelParamsObservation);

      // アンセンテッドカルマンフィルタをファクトリクラスで生成
      AbstractStateEstimateFilterFactory^ factory = gcnew UnscentedKalmanFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);

      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter->StateSpaceModel_, 4);

      // 入力は常に0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // 答え
      cv::Point3d Answers[4] =
      {
        // xhat-[k],   p-[k],     y[k],          g[k],           xhat[k],             p[k]
        { /*   -,*/  42.6025, /* 0.3162,*/  /* 0.788881,*/  1.59269560169013, 32.859912826995 },
        { /*   -,*/  38.0845, /*-1.5581,*/  /* 0.594531,*/ -0.11617605500893, 34.028773659220 },
        { /*   -,*/   9.5248, /*-0.0182,*/  /*-0.764308,*/ -6.03516377961187,  2.901133486898 },
        { /*   -,*/   2.1923, /* 0.4997,*/  /*-0.301484,*/ -5.23683377529549,  1.870116571249 },
      };

      // アンセンテッドカルマンフィルタ開始。時刻k=1からk=4まで実施   
      for (int k = 0; k < 4; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
        // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
        //filter->UpdateToNextTime(vecInputCur);
        filter->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_); 
        filter->Estimate(vecInputCur); // 入力値を元に推定

        double obsevation =
          filter->StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);

        // Assert
        double tolerance = 0.001; // 許容誤差
        //-- 事前誤差共分散
        double expected = Answers[k].x;
        double actual = filter->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        //-- 事後推定値
        expected = Answers[k].y;
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        //-- 事後誤差共分散
        expected = Answers[k].z;
        actual = filter->PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }

    [TestMethod]
    void TestParticleFilterThree()
    {
      // パラメータ設定
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 1.);  // 事後推定初期値: 平均0, 分散1
      double stateTureInit = 0.;  // モデル真値初期値: 0
      
      // モデルはベンチマーク非線形関数
      //-- プロセス: ノイズ　平均0, 分散1
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessBenchMark(), 0., 1.);
      //-- 観測: ノイズ　平均0, 分散3
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 0., 3.));

      //-- PF用パラメータ
      int countPartiles = 3;  // パーティクル数: 3
      AbstractPointEstimation^ pointEstimation = gcnew MMSEPointEstimation();
      DistributionParam^ randomDistributionForPredict = gcnew DistributionParam(0., 1.);  // 粒子事前予測用: 平均0, 分散1
      AbstractResampling^ resampling = gcnew LowVarianceSampling();

      // パラメータ格納
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
         posterioriEstimationInit,
         stateTureInit,
         modelParamProcess,
         modelParamsObservation,
         countPartiles,
         pointEstimation,
         randomDistributionForPredict,
         resampling);

      // パーティクルフィルタをファクトリクラスで生成
      AbstractStateEstimateFilterFactory^ factory = gcnew ParticleFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);
      //-- リサンプリングアルゴリズムの登録(やらなければ、LowVarianceSamplingがデフォルトで指定される)
      //static_cast<ParticleFilter^>(filter)->Resampling_ = gcnew LowVarianceSampling();

#if MY_DEBUG
      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter, 100 + 1);

      vector<double> truths(truthAndObservations->Count);
      vector<double> observations(truthAndObservations->Count);
      for (int i = 0; i < truthAndObservations->Count; i++)
      {
        truths[i] = truthAndObservations[i]->truth_[0];
        observations[i] = truthAndObservations[i]->observation_[0];
      }
#endif
      // 入力は常に0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // 答え
      cv::Point3d AnswersMMSE[3] =
      {
        //      w(1)-[k],        w(1)[k],          mmse[k]
        { 1. / 3.,           0.305155277850409, 3.01641240 },
        { 0.305155277850409, 0.087114739994414, 0.53722541 },
        { 0.087114739994414, 0.351721489323793, -1.2550691 },
      };
      double AnswersPfMap[3] =
      { 2.4702660458708, -0.50798456751785714, 0.53697060403095254 };

      // フィルタ開始。時刻k=1からk=3まで実施   
      for (int k = 0; k < 3; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
        // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
        filter->UpdateToNextTime(vecInputCur);
        filter->Estimate(vecInputCur); // 入力値を元に推定

        // 点推定
        //-- MMSE
        static_cast<ParticleFilter^>(filter)->EstimateMAP(gcnew MMSEPointEstimation());
        //-- デフォルト(MMSE)
        //static_cast<ParticleFilter^>(filter)->EstimateMAP(nullptr);

        double obsevation = 
          filter->StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);

        // Assert
        double tolerance = 0.001; // 許容誤差
        //---- 一時刻前重み
        double expected = AnswersMMSE[k].x;
        //double actual = particleFilter->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        double actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationOld_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- 正規化後重み
        expected = AnswersMMSE[k].y;
        actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationCur_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- MMSE推定値
        expected = AnswersMMSE[k].z;
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);

        //-- pfMap 推定
        static_cast<ParticleFilter^>(filter)->EstimateMAP(gcnew PfMapPointEstimation());
        //---- 事後推定値
        expected = AnswersPfMap[k];
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }

    [TestMethod]
    void TestParticleFilterThreeWithTwoSensors()
    {
      // パラメータ設定
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 1.);  // 事後推定初期値: 平均0, 分散1
      double stateTureInit = 0.;  // モデル真値初期値: 0

      // モデルはベンチマーク非線形関数
      //-- プロセス
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessBenchMark(), 0., 1.);
      //-- 観測: ノイズ　平均0, 分散1
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      //---- センサ1: ノイズ　平均0, 分散3
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 0., 3.));
      //---- センサ2: ノイズ　平均5, 分散4
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 2., 2.));

      //-- PF用パラメータ
      int countPartiles = 3;  // パーティクル数: 3
      AbstractPointEstimation^ pointEstimation = gcnew MMSEPointEstimation();
      DistributionParam^ randomDistributionForPredict = gcnew DistributionParam(0., 5.);  // 粒子事前予測用: 平均0, 分散1
      AbstractResampling^ resampling = gcnew LowVarianceSampling(0.33);

      // パラメータ格納
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
          posterioriEstimationInit,
          stateTureInit,
          modelParamProcess,
          modelParamsObservation,
          countPartiles,
          pointEstimation,
          randomDistributionForPredict,
          resampling);

      // パーティクルフィルタをファクトリクラスで生成
      AbstractStateEstimateFilterFactory^ factory = gcnew ParticleFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);
      //-- リサンプリングアルゴリズムの登録(やらなければ、LowVarianceSamplingがデフォルトで指定される)
      //static_cast<ParticleFilter^>(filter)->Resampling_ = gcnew LowVarianceSampling();
#if 0      
      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter->StateSpaceModel_, 4);
#endif
#if MY_DEBUG
      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter, 100 + 1);

      vector<double> truths(truthAndObservations->Count);
      vector<double> observations(truthAndObservations->Count);
      for (int i = 0; i < truthAndObservations->Count; i++)
      {
        truths[i] = truthAndObservations[i]->truth_[0];
        observations[i] = truthAndObservations[i]->observation_[0];
      }
#endif
      // 入力は常に0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // 答え
      cv::Point3d AnswersMMSE[3] =
      {
        //      w(1)-[k],             w(1)[k],            mmse[k]
        { 1. / 3.,             0.46231291746248310, 1.1610698873496470 },
        { 0.46231291746248310, 0.012890907999986,   5.6166166740532617 },
        { 0.012890907999986,   0.99983216979533163, 0.83458579066711702 },
      };
      double AnswersPfMap[3] =
      { 2.4702660458708, -0.50798456751785714, 0.53697060403095254 };

      // フィルタ開始。時刻k=1からk=3まで実施   
      for (int k = 0; k < 3; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2回入力を入れていて冗長なように見えるが、入力は外部の概念なのでフィルタリングクラスのフィールドには
        // 加えない。情報の更新と推定は役務が異なるので、敢えて分割している。
        filter->UpdateToNextTime(vecInputCur);
        filter->Estimate(vecInputCur); // 入力値を元に推定

        // 点推定
        //-- MMSE
        static_cast<ParticleFilter^>(filter)->EstimateMAP(gcnew MMSEPointEstimation());
        //-- デフォルト(MMSE)
        //static_cast<ParticleFilter^>(filter)->EstimateMAP(nullptr);

        // Assert
        double tolerance = 0.001; // 許容誤差
        //---- 一時刻前重み
        double expected = AnswersMMSE[k].x;
        //double actual = particleFilter->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        double actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationOld_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- 正規化後重み
        expected = AnswersMMSE[k].y;
        actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationCur_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- MMSE推定値
        expected = AnswersMMSE[k].z;
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }
	};
}
