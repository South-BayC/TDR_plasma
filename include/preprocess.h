#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class ImagePreprocessor {
private:
	std::string inputPath;
	std::string outputPath;
	int targetSize;

	// ˽�и�������
	cv::Mat resizeAndPad(const cv::Mat& input);
	cv::Mat applyMedianFilter(const cv::Mat& input, int kernelSize = 3);
	cv::Mat createMask(const cv::Mat& input, double threshold = 0.1);

public:
	// ���캯��
	ImagePreprocessor();
	ImagePreprocessor(const std::string& inputDir, const std::string& outputDir);

	// ����·��
	void setInputPath(const std::string& path);
	void setOutputPath(const std::string& path);

	// ���������
	bool processImage(const std::string& filename, bool saveIntermediate = true);

	// ��������
	bool processAllImages(bool saveIntermediate = true);

	// ��ȡ������ͼ��
	cv::Mat getProcessedImage() const;
	cv::Mat getMask() const;

	// ���ò���
	void setTargetSize(int size);
};

#endif