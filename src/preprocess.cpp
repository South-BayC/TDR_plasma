#include "preprocess.h"
#include <iostream>
#include <windows.h>  // 使用Windows API
#include <algorithm>
#include <direct.h>   // 用于_mkdir
#include <fstream>
#include <chrono>

// 如果宏未定义，使用默认值
#ifndef TARGET_IMAGE_SIZE
#define TARGET_IMAGE_SIZE 512
#endif

#ifndef MEDIAN_FILTER_KERNEL_SIZE
#define MEDIAN_FILTER_KERNEL_SIZE 3
#endif
#ifndef MASK_THRESHOLD
#define MASK_THRESHOLD 0.1
#endif

// #region agent log helper
static void agent_debug_log(const std::string& hypothesisId,
	const std::string& location,
	const std::string& message,
	const std::string& data) {
	std::ofstream ofs("d:\\\\Desktop\\\\TDR_plasma\\\\.cursor\\\\debug.log", std::ios::app);
	if (!ofs) return;
	auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	ofs << "{\"sessionId\":\"debug-session\",\"runId\":\"pre-fix\","
		"\"hypothesisId\":\"" << hypothesisId << "\","
		"\"location\":\"" << location << "\","
		"\"message\":\"" << message << "\","
		"\"data\":" << data << ","
		"\"timestamp\":" << now << "}"
		<< std::endl;
}
// #endregion

// 检查目录是否存在
bool directoryExists(const std::string& path) {
	DWORD dwAttrib = GetFileAttributesA(path.c_str());
	return (dwAttrib != INVALID_FILE_ATTRIBUTES &&
		(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

// 创建目录（如果不存在）
bool createDirectory(const std::string& path) {
	return _mkdir(path.c_str()) == 0;
}

// 获取文件扩展名
std::string getFileExtension(const std::string& filename) {
	size_t dotPos = filename.find_last_of(".");
	if (dotPos != std::string::npos) {
		return filename.substr(dotPos + 1);
	}
	return "";
}

// 将字符串转换为小写
std::string toLower(const std::string& str) {
	std::string result = str;
	std::transform(result.begin(), result.end(), result.begin(), ::tolower);
	return result;
}

// 构造函数
ImagePreprocessor::ImagePreprocessor()
	: targetSize(TARGET_IMAGE_SIZE) {
	// 设置默认路径
	inputPath = "data/input/";
	outputPath = "data/proceed/";

	// 创建输出目录
	createDirectory(outputPath);
}

ImagePreprocessor::ImagePreprocessor(const std::string& inputDir, const std::string& outputDir)
	: inputPath(inputDir), outputPath(outputDir), targetSize(TARGET_IMAGE_SIZE) {

	// 确保路径以斜杠结尾
	if (!inputPath.empty() && inputPath.back() != '\\' && inputPath.back() != '/') {
		inputPath += '\\';
	}
	if (!outputPath.empty() && outputPath.back() != '\\' && outputPath.back() != '/') {
		outputPath += '\\';
	}

	// 创建输出目录
	createDirectory(outputPath);
}

// 设置路径
void ImagePreprocessor::setInputPath(const std::string& path) {
	inputPath = path;
	// 确保路径以斜杠结尾
	if (!inputPath.empty() && inputPath.back() != '\\' && inputPath.back() != '/') {
		inputPath += '\\';
	}
}

void ImagePreprocessor::setOutputPath(const std::string& path) {
	outputPath = path;
	// 确保路径以斜杠结尾
	if (!outputPath.empty() && outputPath.back() != '\\' && outputPath.back() != '/') {
		outputPath += '\\';
	}
	// 创建目录
	createDirectory(outputPath);
}

// 调整大小并填充
cv::Mat ImagePreprocessor::resizeAndPad(const cv::Mat& input) {
	cv::Mat output;

	// 如果图像已经是目标尺寸，直接返回
	if (input.rows == targetSize && input.cols == targetSize) {
		return input.clone();
	}

	// 创建目标尺寸的黑色画布
	output = cv::Mat::zeros(targetSize, targetSize, input.type());

	// 计算原始图像在目标画布中的位置（居中）
	int x = (targetSize - input.cols) / 2;
	int y = (targetSize - input.rows) / 2;

	// 确保位置有效
	x = std::max(0, x);
	y = std::max(0, y);

	// 确保原始图像不会超出目标范围
	int width = std::min(input.cols, targetSize - x);
	int height = std::min(input.rows, targetSize - y);

	// #region agent log
	agent_debug_log("H1", "preprocess.cpp:resizeAndPad", "computed placement",
		"{\"inRows\":" + std::to_string(input.rows) +
		",\"inCols\":" + std::to_string(input.cols) +
		",\"target\":" + std::to_string(targetSize) +
		",\"width\":" + std::to_string(width) +
		",\"height\":" + std::to_string(height) + "}");
	// #endregion

	if (width <= 0 || height <= 0) {
		std::cerr << "Error: Cannot fit image into target size." << std::endl;
		return cv::Mat();
	}

	// 将原始图像复制到目标位置
	cv::Mat roi = output(cv::Rect(x, y, width, height));
	cv::Mat inputRoi = input(cv::Rect(0, 0, width, height));
	inputRoi.copyTo(roi);

	return output;
}

// 应用中值滤波
cv::Mat ImagePreprocessor::applyMedianFilter(const cv::Mat& input, int kernelSize) {
	cv::Mat filtered;

	// 确保核大小为奇数
	if (kernelSize % 2 == 0) {
		kernelSize++;
	}

	cv::medianBlur(input, filtered, kernelSize);
	return filtered;
}

// 创建掩膜（二值化）
cv::Mat ImagePreprocessor::createMask(const cv::Mat& input, double threshold) {
	if (input.empty()) {
		return cv::Mat();
	}

	cv::Mat gray;
	if (input.channels() == 3) {
		cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
	}
	else {
		gray = input.clone();
	}

	// 归一化到0-1范围
	cv::Mat normalized;
	gray.convertTo(normalized, CV_32F, 1.0 / 255.0);

	// 应用阈值
	cv::Mat mask;
	cv::threshold(normalized, mask, threshold, 1.0, cv::THRESH_BINARY);

	// 转换回8位
	cv::Mat mask8u;
	mask.convertTo(mask8u, CV_8U, 255);

	return mask8u;
}

// 澶勭悊鍥惧儚
bool ImagePreprocessor::processImage(const std::string& filename, bool saveIntermediate) {
	std::string fullInputPath = inputPath + filename;

	// #region agent log
	agent_debug_log("H1", "preprocess.cpp:processImage", "enter processImage",
		"{\"file\":\"" + filename + "\",\"saveIntermediate\":" + (saveIntermediate ? "true" : "false") + "}");
	// #endregion

	// 妫€鏌ユ枃浠舵槸鍚﹀瓨鍦紝浣跨敤Windows API
	DWORD dwAttrib = GetFileAttributesA(fullInputPath.c_str());
	if (dwAttrib == INVALID_FILE_ATTRIBUTES || (dwAttrib & FILE_ATTRIBUTE_DIRECTORY)) {
		std::cerr << "Error: File not found - " << fullInputPath << std::endl;
		return false;
	}

	// 读取图像
	cv::Mat image = cv::imread(fullInputPath, cv::IMREAD_GRAYSCALE);
	if (image.empty()) {
		std::cerr << "Error: Failed to load image - " << fullInputPath << std::endl;
		return false;
	}

	// #region agent log
	agent_debug_log("H2", "preprocess.cpp:processImage", "loaded image",
		"{\"cols\":" + std::to_string(image.cols) + ",\"rows\":" + std::to_string(image.rows) + "}");
	// #endregion

	std::cout << "Processing: " << filename << std::endl;
	std::cout << "  Original size: " << image.cols << "x" << image.rows << std::endl;

	// Step1: 调整尺寸到目标大小
	cv::Mat resized = resizeAndPad(image);
	if (resized.empty()) {
		return false;
	}

	if (saveIntermediate) {
		std::string resizedPath = outputPath + "resized_" + filename;
		cv::imwrite(resizedPath, resized);
		std::cout << "  Saved resized image: " << resizedPath << std::endl;
	}

	// Step2: 中值滤波（去噪）
	cv::Mat filtered = applyMedianFilter(resized, MEDIAN_FILTER_KERNEL_SIZE);

	if (saveIntermediate) {
		std::string filteredPath = outputPath + "filtered_" + filename;
		cv::imwrite(filteredPath, filtered);
		std::cout << "  Saved filtered image: " << filteredPath << std::endl;
	}

	// Step3: 创建掩膜
	cv::Mat mask = createMask(filtered, MASK_THRESHOLD);

	if (!mask.empty() && saveIntermediate) {
		std::string maskPath = outputPath + "mask_" + filename;
		cv::imwrite(maskPath, mask);
		std::cout << "  Saved mask: " << maskPath << std::endl;
	}

	// Step4: 应用掩膜
	cv::Mat finalImage;
	if (!mask.empty()) {
		// 将掩膜转换为与图像类型一致
		cv::Mat maskFloat;
		mask.convertTo(maskFloat, filtered.type(), 1.0 / 255.0);

		// 应用掩膜
		cv::multiply(filtered, maskFloat, finalImage);
	}
	else {
		finalImage = filtered.clone();
	}

	// 保存最终结果
	std::string finalPath = outputPath + "processed_" + filename;
	cv::imwrite(finalPath, finalImage);
	std::cout << "  Saved processed image: " << finalPath << std::endl;

	std::cout << "  Processing completed successfully." << std::endl;
	return true;
}

// 处理全部图像
bool ImagePreprocessor::processAllImages(bool saveIntermediate) {
	// 检查输入目录是否存在
	if (!directoryExists(inputPath)) {
		std::cerr << "Error: Input directory not found - " << inputPath << std::endl;
		return false;
	}

	std::cout << "Processing all images in: " << inputPath << std::endl;

	bool allSuccess = true;
	int processedCount = 0;

	// #region agent log
	agent_debug_log("H1", "preprocess.cpp:processAllImages", "enter processAllImages",
		"{\"inputPath\":\"" + inputPath + "\",\"saveIntermediate\":" + (saveIntermediate ? "true" : "false") + "}");
	// #endregion

	// 搜索目录中的所有文件
	std::string searchPath = inputPath + "*.*";
	WIN32_FIND_DATAA findFileData;
	HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findFileData);

	if (hFind == INVALID_HANDLE_VALUE) {
		std::cerr << "Error: Cannot open directory - " << inputPath << std::endl;
		return false;
	}

	do {
		// 璺宠繃鐩綍
		if (findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			continue;
		}

		std::string filename = findFileData.cFileName;

		// 妫€鏌ユ槸鍚︿负鍥惧儚鏂囦欢锛堥€氳繃鎵╁睍鍚嶏級
		std::string ext = getFileExtension(filename);
		std::string lowerExt = toLower(ext);

		if (lowerExt == "png" || lowerExt == "jpg" || lowerExt == "jpeg" ||
			lowerExt == "bmp" || lowerExt == "tif" || lowerExt == "tiff") {

			if (processImage(filename, saveIntermediate)) {
				processedCount++;
			}
			else {
				allSuccess = false;
				std::cerr << "Failed to process: " << filename << std::endl;
			}
		}
	} while (FindNextFileA(hFind, &findFileData) != 0);

	FindClose(hFind);

	std::cout << "Total processed: " << processedCount << " images" << std::endl;
	return allSuccess;
}

// 璁剧疆鐩爣澶у皬
void ImagePreprocessor::setTargetSize(int size) {
	if (size > 0) {
		targetSize = size;
	}
	else {
		std::cerr << "Warning: Invalid target size. Using default " << TARGET_IMAGE_SIZE << "." << std::endl;
		targetSize = TARGET_IMAGE_SIZE;
	}
}

// 获取处理后的图像（目前未缓存实际图像，仅返回空矩阵占位）
cv::Mat ImagePreprocessor::getProcessedImage() const {
	return cv::Mat();
}

cv::Mat ImagePreprocessor::getMask() const {
	return cv::Mat();
}