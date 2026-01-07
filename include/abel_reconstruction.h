#ifndef ABEL_RECONSTRUCTION_H
#define ABEL_RECONSTRUCTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class AbelReconstructor {
private:
	std::string inputPath;
	std::string outputPath;
	std::string intermediatePath;

	cv::Mat processedImage;
	cv::Mat reconstructedImage;
	int imageWidth;
	int imageHeight;

	// 工具函数
	bool saveMatTxt(const cv::Mat& mat, const std::string& filename, bool isComplex = false) const;

	// 步骤a: 构造0~180°范围内的完备投影数据
	// 返回投影数据，并通过maxValidRadius返回有效数据的最大半径
	cv::Mat constructProjectionData(int columnIndex, int& maxValidRadius) const;

	// 步骤b: 基于完备数据利用Abel逆变换重建二维分布
	// maxValidRadius: 掩膜处理后的原始投影数据的有效半径，用于限制圆盘大小
	cv::Mat reconstruct2DFromProjection(const cv::Mat& proj1D, int size, int maxValidRadius) const;

	// 对单列执行完整的Abel反演流程（步骤a + 步骤b）
	cv::Mat reconstructAbelFromColumn(int columnIndex, bool saveIntermediate = false) const;

public:
	// 构造函数
	AbelReconstructor();
	AbelReconstructor(const std::string& inputDir, const std::string& outputDir,
		const std::string& intermediateDir);

	// 设置路径
	void setInputPath(const std::string& path);
	void setOutputPath(const std::string& path);
	void setIntermediatePath(const std::string& path);

	// 加载预处理后的图像
	bool loadProcessedImage(const std::string& filename);

	// 执行3D重建：对所有列执行Abel反演，生成点云（步骤c）
	bool perform3DReconstruction(const std::string& inputImage, bool saveIntermediate = true);

	// 测试函数：只重建中间一列
	bool testMiddleColumnReconstructionComplex(const std::string& inputImage);

	// 获取中间切片
	cv::Mat getMiddleSlice() const;

	// 保存结果
	bool saveResults(const std::string& baseName);
};

#endif
