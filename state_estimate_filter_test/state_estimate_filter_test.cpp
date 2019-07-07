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
		///���݂̃e�X�g�̎��s�ɂ��Ă̏�񂨂�ы@�\��
		///�񋟂���e�X�g �R���e�L�X�g���擾�܂��͐ݒ肵�܂��B
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
		//�e�X�g���쐬����ۂɂ́A���̒ǉ��������g�p�ł��܂�:
		//
		//�N���X���ōŏ��̃e�X�g�����s����O�ɁAClassInitialize ���g�p���ăR�[�h�����s���Ă�������
		//[ClassInitialize()]
		//static void MyClassInitialize(TestContext^ testContext) {};
		//
		//�N���X���̃e�X�g�����ׂĎ��s������AClassCleanup ���g�p���ăR�[�h�����s���Ă�������
		//[ClassCleanup()]
		//static void MyClassCleanup() {};
		//
		//�e�e�X�g�����s����O�ɁATestInitialize ���g�p���ăR�[�h�����s���Ă�������
		//[TestInitialize()]
		//void MyTestInitialize() {};
		//
		//�e�e�X�g�����s������ɁATestCleanup ���g�p���ăR�[�h�����s���Ă�������
		//[TestCleanup()]
		//void MyTestCleanup() {};
		//
		#pragma endregion 

    /////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////0
    [TestMethod]
    void TestNormalDistribution()
    {
      // arrange
      // ����3, �W���΍�2�̐��K���z
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
      // �����͖�0.176�B���e�덷��0.001�B
      double expected = 0.176;
      double actual = vecResult.at<double>(0, 0);
      double tolerance = 0.001;

      Assert::AreEqual(expected, actual, tolerance);

      // �ΐ���
      normalDistribution.CalcLogDencityFunction(vecValue, &vecResult);
      expected = -1.7373;
      actual = vecResult.at<double>(0, 0);
      Assert::AreEqual(expected, actual, tolerance);


      NormalDistribution normalDistribution2(3, 2 * 2);
      // assert
      // �����͖�0.176�B���e�덷��0.001�B
      expected = 0.176;
      actual = normalDistribution2.CalcDencityFunction(2);
      Assert::AreEqual(expected, actual, tolerance);
    }

    /////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////0
    [TestMethod]
    void TestLinearKalmanFilter()
    {
      // �p�����[�^�ݒ�
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 0.);  // ���㐄�菉���l: ����11, ���U1
      double stateTureInit = 0.;  // ���f���^�l�����l: 0

      // ���f���̓����_���E�H�[�N
      //-- �v���Z�X
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessRandomWalk(), 0., 1.);
      //-- �ϑ�
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveRandomWalk(), 0., 2.));

      // �p�����[�^�i�[
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
        posterioriEstimationInit,
        stateTureInit, 
        modelParamProcess, 
        modelParamsObservation);

      // ���`�J���}���t�B���^���t�@�N�g���N���X�Ő���
      AbstractStateEstimateFilterFactory^ factory = gcnew LinearKalmanFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);
      
      // ����
      cv::Point3d Answers[4] =
      {
        // xhat-[k],   p-[k],    y[k],    g[k],     xhat[k],  p[k]
        { /*   -,       1,     0.3162,*/  1 / 3,    0.1054,  0.6667 },
        { /*   -,      5/3,   -1.5581,*/  5 / 11,  -0.6507,  0.9091 },
        { /*   -,     21/11,  -0.0182,*/ 21 / 43,  -0.3418,  0.9767 },
        { /*   -,     85/43,   0.4997,*/ 85 / 171,  0.0765,  0.9942 },
      };

      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // �J���}���t�B���^�J�n�B����k=1����k=4�܂Ŏ��{   
      for (int k = 0; k < 4; k++)
      { 
        // 2����͂����Ă��ď璷�Ȃ悤�Ɍ����邪�A���͂͊O���̊T�O�Ȃ̂Ńt�B���^�����O�N���X�̃t�B�[���h�ɂ�
        // �����Ȃ��B���̍X�V�Ɛ���͖𖱂��قȂ�̂ŁA�����ĕ������Ă���B
        filter->UpdateToNextTime(vecInputCur);
        filter->Estimate(vecInputCur); // ���͒l�����ɐ���

        // Assert
        double tolerance = 0.001; // ���e�덷
        //-- ���㐄��l
        double expected = Answers[k].y;
        double actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        //-- �덷�����U
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
      // �p�����[�^�ݒ�
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(11., 1.);  // ���㐄�菉���l: ����11, ���U1
      double stateTureInit = 10.;  // ���f���^�l�����l: 10

      // ���f���̓J���}���t�B���^�̊�b�ŏo�Ă���EKF�̃T���v���֐�
      //-- �v���Z�X: �m�C�Y�@����0, ���U1
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessEKFSample(), 0., 1.);
      //-- �ϑ�: �m�C�Y�@����0, ���U100
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveEKFSample(), 0., 100.));

      // �p�����[�^�i�[
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
        posterioriEstimationInit, 
        stateTureInit,
        modelParamProcess, 
        modelParamsObservation);

      // �g���J���}���t�B���^���t�@�N�g���N���X�Ő���
      AbstractStateEstimateFilterFactory^ factory = gcnew ExtendedKalmanFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);

      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter->StateSpaceModel_, 4);

      // ���͂͏��0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // ����
      cv::Point3d Answers[4] =
      {
        // xhat-[k],   p-[k],     y[k],        g[k],       xhat[k],                     p[k]
        { /*   -,*/  1.513586, /*0.3162,*/   0.002181, 11.6990182094355    /*, 0.000475812953301316*/ },
        { /*   -,*/  1.000241, /*-1.5581,*/  0.002012, 12.0700238407473    /*, 0.000404857301792413*/ },
        { /*   -,*/  1.000204, /*-0.0182,*/  0.001931, 14.4789625719249    /*, 0.000372860020947514*/ },
        { /*   -,*/  1.000183, /*0.4997,*/   0.001512, 14.2262173029902    /*, 0.000228630591662933*/ },
      };

      // �g���J���}���t�B���^�J�n�B����k=1����k=4�܂Ŏ��{   
      for (int k = 0; k < 4; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2����͂����Ă��ď璷�Ȃ悤�Ɍ����邪�A���͂͊O���̊T�O�Ȃ̂Ńt�B���^�����O�N���X�̃t�B�[���h�ɂ�
        // �����Ȃ��B���̍X�V�Ɛ���͖𖱂��قȂ�̂ŁA�����ĕ������Ă���B
        //filter->UpdateToNextTime(vecInputCur);
        filter->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_);
        filter->Estimate(vecInputCur); // ���͒l�����ɐ���

        // Assert
        double tolerance = 0.001; // ���e�덷
        //-- ���O�덷�����U
        //double expected = Answers[k].x;
        //double actual = (static_cast<ExtendedKalmanFilter^>(extendedKalmanFilter)->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        //Assert::AreEqual(expected, actual, tolerance);
        //-- ���㐄��l
        double expected = Answers[k].z;
        double actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        
      }
    }

    [TestMethod]
    void TestUnscentedKalmanFilter()
    {
      // �p�����[�^�ݒ�
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 1.);  // ���㐄�菉���l: ����0, ���U1
      double stateTureInit = 0.;  // ���f���^�l�����l: 0

      // ���f���̓x���`�}�[�N����`�֐�
      //-- �v���Z�X: �m�C�Y�@����0, ���U1
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessBenchMark(), 0., 1.);
      //-- �ϑ�: �m�C�Y�@����0, ���U3
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 0., 3.));

      // �p�����[�^�i�[
      StateEstimaterParam^ stateEstimaterParam =
        gcnew StateEstimaterParam(
        posterioriEstimationInit,
        stateTureInit,
        modelParamProcess,
        modelParamsObservation);

      // �A���Z���e�b�h�J���}���t�B���^���t�@�N�g���N���X�Ő���
      AbstractStateEstimateFilterFactory^ factory = gcnew UnscentedKalmanFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);

      TruthAndObservationGenerator^ truthAndObservationGenerator = gcnew TruthAndObservationGenerator();
      List<TruthAndObservation^>^ truthAndObservations = truthAndObservationGenerator->Generate(filter->StateSpaceModel_, 4);

      // ���͂͏��0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // ����
      cv::Point3d Answers[4] =
      {
        // xhat-[k],   p-[k],     y[k],          g[k],           xhat[k],             p[k]
        { /*   -,*/  42.6025, /* 0.3162,*/  /* 0.788881,*/  1.59269560169013, 32.859912826995 },
        { /*   -,*/  38.0845, /*-1.5581,*/  /* 0.594531,*/ -0.11617605500893, 34.028773659220 },
        { /*   -,*/   9.5248, /*-0.0182,*/  /*-0.764308,*/ -6.03516377961187,  2.901133486898 },
        { /*   -,*/   2.1923, /* 0.4997,*/  /*-0.301484,*/ -5.23683377529549,  1.870116571249 },
      };

      // �A���Z���e�b�h�J���}���t�B���^�J�n�B����k=1����k=4�܂Ŏ��{   
      for (int k = 0; k < 4; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2����͂����Ă��ď璷�Ȃ悤�Ɍ����邪�A���͂͊O���̊T�O�Ȃ̂Ńt�B���^�����O�N���X�̃t�B�[���h�ɂ�
        // �����Ȃ��B���̍X�V�Ɛ���͖𖱂��قȂ�̂ŁA�����ĕ������Ă���B
        //filter->UpdateToNextTime(vecInputCur);
        filter->UpdateToNextTimeFromOuterValue(truthAndObservations[k]->truth_, truthAndObservations[k]->observations_); 
        filter->Estimate(vecInputCur); // ���͒l�����ɐ���

        double obsevation =
          filter->StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);

        // Assert
        double tolerance = 0.001; // ���e�덷
        //-- ���O�덷�����U
        double expected = Answers[k].x;
        double actual = filter->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        //-- ���㐄��l
        expected = Answers[k].y;
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
        //-- ����덷�����U
        expected = Answers[k].z;
        actual = filter->PosterioriEstimationCur_->MatVariance_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }

    [TestMethod]
    void TestParticleFilterThree()
    {
      // �p�����[�^�ݒ�
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 1.);  // ���㐄�菉���l: ����0, ���U1
      double stateTureInit = 0.;  // ���f���^�l�����l: 0
      
      // ���f���̓x���`�}�[�N����`�֐�
      //-- �v���Z�X: �m�C�Y�@����0, ���U1
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessBenchMark(), 0., 1.);
      //-- �ϑ�: �m�C�Y�@����0, ���U3
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 0., 3.));

      //-- PF�p�p�����[�^
      int countPartiles = 3;  // �p�[�e�B�N����: 3
      AbstractPointEstimation^ pointEstimation = gcnew MMSEPointEstimation();
      DistributionParam^ randomDistributionForPredict = gcnew DistributionParam(0., 1.);  // ���q���O�\���p: ����0, ���U1
      AbstractResampling^ resampling = gcnew LowVarianceSampling();

      // �p�����[�^�i�[
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

      // �p�[�e�B�N���t�B���^���t�@�N�g���N���X�Ő���
      AbstractStateEstimateFilterFactory^ factory = gcnew ParticleFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);
      //-- ���T���v�����O�A���S���Y���̓o�^(���Ȃ���΁ALowVarianceSampling���f�t�H���g�Ŏw�肳���)
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
      // ���͂͏��0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // ����
      cv::Point3d AnswersMMSE[3] =
      {
        //      w(1)-[k],        w(1)[k],          mmse[k]
        { 1. / 3.,           0.305155277850409, 3.01641240 },
        { 0.305155277850409, 0.087114739994414, 0.53722541 },
        { 0.087114739994414, 0.351721489323793, -1.2550691 },
      };
      double AnswersPfMap[3] =
      { 2.4702660458708, -0.50798456751785714, 0.53697060403095254 };

      // �t�B���^�J�n�B����k=1����k=3�܂Ŏ��{   
      for (int k = 0; k < 3; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2����͂����Ă��ď璷�Ȃ悤�Ɍ����邪�A���͂͊O���̊T�O�Ȃ̂Ńt�B���^�����O�N���X�̃t�B�[���h�ɂ�
        // �����Ȃ��B���̍X�V�Ɛ���͖𖱂��قȂ�̂ŁA�����ĕ������Ă���B
        filter->UpdateToNextTime(vecInputCur);
        filter->Estimate(vecInputCur); // ���͒l�����ɐ���

        // �_����
        //-- MMSE
        static_cast<ParticleFilter^>(filter)->EstimateMAP(gcnew MMSEPointEstimation());
        //-- �f�t�H���g(MMSE)
        //static_cast<ParticleFilter^>(filter)->EstimateMAP(nullptr);

        double obsevation = 
          filter->StateSpaceModel_->ObservationModelEquations_[0]->observationCur_->VecMean_.at<double>(0, 0);

        // Assert
        double tolerance = 0.001; // ���e�덷
        //---- �ꎞ���O�d��
        double expected = AnswersMMSE[k].x;
        //double actual = particleFilter->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        double actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationOld_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- ���K����d��
        expected = AnswersMMSE[k].y;
        actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationCur_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- MMSE����l
        expected = AnswersMMSE[k].z;
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);

        //-- pfMap ����
        static_cast<ParticleFilter^>(filter)->EstimateMAP(gcnew PfMapPointEstimation());
        //---- ���㐄��l
        expected = AnswersPfMap[k];
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }

    [TestMethod]
    void TestParticleFilterThreeWithTwoSensors()
    {
      // �p�����[�^�ݒ�
      DistributionParam^ posterioriEstimationInit = gcnew DistributionParam(0., 1.);  // ���㐄�菉���l: ����0, ���U1
      double stateTureInit = 0.;  // ���f���^�l�����l: 0

      // ���f���̓x���`�}�[�N����`�֐�
      //-- �v���Z�X
      ModelParam^ modelParamProcess = gcnew ModelParam(gcnew ModelFunctionSetProcessBenchMark(), 0., 1.);
      //-- �ϑ�: �m�C�Y�@����0, ���U1
      List<ModelParam^>^ modelParamsObservation = gcnew List<ModelParam^>();
      //---- �Z���T1: �m�C�Y�@����0, ���U3
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 0., 3.));
      //---- �Z���T2: �m�C�Y�@����5, ���U4
      modelParamsObservation->Add(gcnew ModelParam(gcnew ModelFunctionSetObserveBenchMark(), 2., 2.));

      //-- PF�p�p�����[�^
      int countPartiles = 3;  // �p�[�e�B�N����: 3
      AbstractPointEstimation^ pointEstimation = gcnew MMSEPointEstimation();
      DistributionParam^ randomDistributionForPredict = gcnew DistributionParam(0., 5.);  // ���q���O�\���p: ����0, ���U1
      AbstractResampling^ resampling = gcnew LowVarianceSampling(0.33);

      // �p�����[�^�i�[
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

      // �p�[�e�B�N���t�B���^���t�@�N�g���N���X�Ő���
      AbstractStateEstimateFilterFactory^ factory = gcnew ParticleFilterFactory();
      AbstractStateEstimateFilter ^filter = factory->Create(stateEstimaterParam);
      //-- ���T���v�����O�A���S���Y���̓o�^(���Ȃ���΁ALowVarianceSampling���f�t�H���g�Ŏw�肳���)
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
      // ���͂͏��0
      cv::Mat vecInputCur = (cv::Mat_<double>(1, 1) << 0.0);

      // ����
      cv::Point3d AnswersMMSE[3] =
      {
        //      w(1)-[k],             w(1)[k],            mmse[k]
        { 1. / 3.,             0.46231291746248310, 1.1610698873496470 },
        { 0.46231291746248310, 0.012890907999986,   5.6166166740532617 },
        { 0.012890907999986,   0.99983216979533163, 0.83458579066711702 },
      };
      double AnswersPfMap[3] =
      { 2.4702660458708, -0.50798456751785714, 0.53697060403095254 };

      // �t�B���^�J�n�B����k=1����k=3�܂Ŏ��{   
      for (int k = 0; k < 3; k++)
      {
        filter->TimeCur_ = k + 1;
        // 2����͂����Ă��ď璷�Ȃ悤�Ɍ����邪�A���͂͊O���̊T�O�Ȃ̂Ńt�B���^�����O�N���X�̃t�B�[���h�ɂ�
        // �����Ȃ��B���̍X�V�Ɛ���͖𖱂��قȂ�̂ŁA�����ĕ������Ă���B
        filter->UpdateToNextTime(vecInputCur);
        filter->Estimate(vecInputCur); // ���͒l�����ɐ���

        // �_����
        //-- MMSE
        static_cast<ParticleFilter^>(filter)->EstimateMAP(gcnew MMSEPointEstimation());
        //-- �f�t�H���g(MMSE)
        //static_cast<ParticleFilter^>(filter)->EstimateMAP(nullptr);

        // Assert
        double tolerance = 0.001; // ���e�덷
        //---- �ꎞ���O�d��
        double expected = AnswersMMSE[k].x;
        //double actual = particleFilter->PriorEstimationCur_->MatVariance_.at<double>(0, 0);
        double actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationOld_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- ���K����d��
        expected = AnswersMMSE[k].y;
        actual = static_cast<ParticleDistribution^>(filter->PosterioriEstimationCur_)->Particles_[0]->Weight_;
        Assert::AreEqual(expected, actual, tolerance);
        //---- MMSE����l
        expected = AnswersMMSE[k].z;
        actual = filter->PosterioriEstimationCur_->VecMean_.at<double>(0, 0);
        Assert::AreEqual(expected, actual, tolerance);
      }
    }
	};
}
