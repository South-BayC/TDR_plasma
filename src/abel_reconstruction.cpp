#include "abel_reconstruction.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>

// 如果宏未定义，使用默认值
#ifndef PROJECTION_THRESHOLD
#define PROJECTION_THRESHOLD 1e-3f
#endif
#ifndef MIN_CONSECUTIVE_ZEROS
#define MIN_CONSECUTIVE_ZEROS 5
#endif
#ifndef EPSILON
#define EPSILON 1e-6f
#endif
#ifndef POINT_CLOUD_THRESHOLD_RATIO
#define POINT_CLOUD_THRESHOLD_RATIO 0.01f
#endif

// 点云颜色控制（可被 main.cpp 中宏覆盖）
#ifndef POINT_CLOUD_COLOR_MODE
// 0 = 按强度灰度映射；1 = 固定颜色
#define POINT_CLOUD_COLOR_MODE 0
#endif

#ifndef POINT_CLOUD_COLOR_R
#define POINT_CLOUD_COLOR_R 255
#endif

#ifndef POINT_CLOUD_COLOR_G
#define POINT_CLOUD_COLOR_G 255
#endif

#ifndef POINT_CLOUD_COLOR_B
#define POINT_CLOUD_COLOR_B 255
#endif

#ifndef ENABLE_POINT_CLOUD_VIEWER
// 是否在程序内直接显示点云（默认开启，可在main.cpp中通过宏关闭）
#define ENABLE_POINT_CLOUD_VIEWER 1
#endif

// #region agent log helper
static void abel_agent_log(const std::string& hypothesisId,
	const std::string& location,
	const std::string& message,
	const std::string& data) {
	std::ofstream ofs("d:\\\\Desktop\\\\TDR_plasma\\\\.cursor\\\\debug_runtime.log", std::ios::app);
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

// 构造函数
AbelReconstructor::AbelReconstructor()
	: reconstructedImage(), imageWidth(0), imageHeight(0) {
	inputPath = "../data/proceed/";
	outputPath = "../data/output/";
	intermediatePath = "../data/proceed/";
}

AbelReconstructor::AbelReconstructor(const std::string& inputDir, const std::string& outputDir,
	const std::string& intermediateDir)
	: inputPath(inputDir), outputPath(outputDir), intermediatePath(intermediateDir),
	reconstructedImage(), imageWidth(0), imageHeight(0) {
}

// 设置路径
void AbelReconstructor::setInputPath(const std::string& path) {
	inputPath = path;
	if (!inputPath.empty() && inputPath.back() != '/' && inputPath.back() != '\\') {
		inputPath += '/';
	}
}

void AbelReconstructor::setOutputPath(const std::string& path) {
	outputPath = path;
	if (!outputPath.empty() && outputPath.back() != '/' && outputPath.back() != '\\') {
		outputPath += '/';
	}
}

void AbelReconstructor::setIntermediatePath(const std::string& path) {
	intermediatePath = path;
	if (!intermediatePath.empty() && intermediatePath.back() != '/' && intermediatePath.back() != '\\') {
		intermediatePath += '/';
	}
}

// 加载预处理后的图像
bool AbelReconstructor::loadProcessedImage(const std::string& filename) {
	std::string fullPath = inputPath + filename;
	processedImage = cv::imread(fullPath, cv::IMREAD_GRAYSCALE);

	if (processedImage.empty()) {
		std::cerr << "Error: Failed to load processed image - " << fullPath << std::endl;
		return false;
	}

	imageWidth = processedImage.cols;
	imageHeight = processedImage.rows;

	std::cout << "Loaded image: " << filename << std::endl;
	std::cout << "  Size: " << imageWidth << "x" << imageHeight << std::endl;

	return true;
}

// 保存矩阵到TXT文件
bool AbelReconstructor::saveMatTxt(const cv::Mat& mat, const std::string& filename, bool isComplex) const {
	if (mat.empty()) {
		std::cerr << "Error: Empty matrix, cannot save to " << filename << std::endl;
		return false;
	}

	std::string fullPath = intermediatePath + filename;
	std::ofstream file(fullPath);
	if (!file.is_open()) {
		std::cerr << "Error: Cannot open file for writing: " << fullPath << std::endl;
		return false;
	}

	if (isComplex && mat.channels() == 2) {
		// 复数矩阵：保存实部和虚部
	cv::Mat planes[2];
		cv::split(mat, planes);
		for (int i = 0; i < mat.rows; i++) {
			for (int j = 0; j < mat.cols; j++) {
				file << planes[0].at<float>(i, j) << " " << planes[1].at<float>(i, j) << " ";
			}
			file << "\n";
			}
		}
		else {
		// 实数矩阵
		for (int i = 0; i < mat.rows; i++) {
			for (int j = 0; j < mat.cols; j++) {
				file << mat.at<float>(i, j) << " ";
			}
			file << "\n";
		}
	}

	file.close();
	std::cout << "  Saved matrix to: " << fullPath << std::endl;
	return true;
}

// ============================================================
// 步骤a: 取出等离子体图像的一列像素作为轴对称的等离子体
//        在各个方向上的投影，构造出0~180°范围内的完备数据
// ============================================================
cv::Mat AbelReconstructor::constructProjectionData(int columnIndex, int& maxValidRadius) const {
	if (processedImage.empty() || columnIndex < 0 || columnIndex >= processedImage.cols) {
		maxValidRadius = 0;
		return cv::Mat();
	}

	// 提取指定列的数据（整列）
	cv::Mat columnData = processedImage.col(columnIndex).clone();
	int columnHeight = columnData.rows;

	// 转换为浮点数
	cv::Mat floatColumn;
	columnData.convertTo(floatColumn, CV_32F);

	// 对于轴对称的等离子体，0~180°的投影数据是相同的
	// 因此使用这一列数据作为所有角度的投影数据
	// 构造一维投影 P(y)：从中心向外的径向投影
	int centerY = columnHeight / 2;
	int radialSize = centerY; // 半径采样点数

	cv::Mat proj1D(radialSize, 1, CV_32F, cv::Scalar(0));

	// 检测有效数据的最大半径（掩膜处理后的有效数据范围）
	// 使用更严格的方法：找到连续的有效数据段，当连续多个位置都小于阈值时，认为有效数据结束
	const float threshold = PROJECTION_THRESHOLD; // 阈值，使用宏定义
	const int minConsecutiveZeros = MIN_CONSECUTIVE_ZEROS; // 连续零计数阈值，使用宏定义
	maxValidRadius = 0;
	int consecutiveZeros = 0;

	for (int i = 0; i < radialSize; ++i) {
		int yTop = centerY - i - 1;
		int yBottom = centerY + i;

		float vTop = (yTop >= 0 && yTop < columnHeight) ? floatColumn.at<float>(yTop, 0) : 0.0f;
		float vBottom = (yBottom >= 0 && yBottom < columnHeight) ? floatColumn.at<float>(yBottom, 0) : 0.0f;

		// 上下对称平均，得到径向投影值
		float projValue = 0.5f * (vTop + vBottom);
		proj1D.at<float>(i, 0) = projValue;

		// 更严格的有效半径检测：需要连续多个位置都小于阈值才认为有效数据结束
		if (projValue > threshold) {
			maxValidRadius = i + 1; // +1 因为半径是从0开始的索引
			consecutiveZeros = 0; // 重置连续零计数
		}
		else {
			consecutiveZeros++;
			// 如果连续多个位置都小于阈值，且已经找到了有效数据，则停止更新
			// 这样可以避免外部噪声被计入有效半径
			if (consecutiveZeros >= minConsecutiveZeros && maxValidRadius > 0) {
				break; // 有效数据段已结束，停止扫描
			}
		}
	}

	return proj1D;
}

// ============================================================
// 步骤b: 基于构造出的完备数据利用Abel逆变换算法
//        重建出等离子体二维分布
// ============================================================
cv::Mat AbelReconstructor::reconstruct2DFromProjection(const cv::Mat& proj1D, int size, int maxValidRadius) const {
	if (proj1D.empty()) {
		return cv::Mat();
	}

	int radialSize = proj1D.rows;

	// 限制有效半径不超过投影数据的范围
	if (maxValidRadius <= 0) {
		maxValidRadius = radialSize; // 如果没有有效半径，使用全部数据
	}
	if (maxValidRadius > radialSize) {
		maxValidRadius = radialSize;
	}

	// 计算投影的导数 dP/dy
	std::vector<float> dP(radialSize, 0.0f);
	for (int y = 1; y < radialSize - 1; ++y) {
		dP[y] = proj1D.at<float>(y + 1, 0) - proj1D.at<float>(y - 1, 0);
		dP[y] *= 0.5f; // 中心差分
	}
	// 边界使用一阶差分
	if (radialSize >= 2) {
		dP[0] = proj1D.at<float>(1, 0) - proj1D.at<float>(0, 0);
		dP[radialSize - 1] = proj1D.at<float>(radialSize - 1, 0) - proj1D.at<float>(radialSize - 2, 0);
	}

	// 离散 Abel 反演，得到径向分布 f(r)
	// 使用公式：f(r) ≈ -1/π ∑_{y=r}^{R-1} (dP/dy)(y) / sqrt(y^2 - r^2)
	cv::Mat radialProfile(radialSize, 1, CV_32F, cv::Scalar(0));

	const float eps = EPSILON; // 数值稳定性参数，使用宏定义
	// 只计算到有效半径范围内的径向分布
	int validRadialSize = maxValidRadius;
	for (int r = 0; r < validRadialSize; ++r) {
		float sum = 0.0f;
		for (int y = r; y < validRadialSize; ++y) {
			float yy = static_cast<float>(y);
			float rr = static_cast<float>(r);
			float denomSq = yy * yy - rr * rr;
			if (denomSq <= 0.0f) {
				continue;
			}
			float denom = std::sqrt(denomSq + eps);
			sum += dP[y] / denom;
		}
		float fval = -sum / static_cast<float>(CV_PI);
		radialProfile.at<float>(r, 0) = fval;
	}

	// 由径向分布 f(r) 生成二维径向对称重建图像
	// 使用maxValidRadius限制圆盘半径，滤去多余数据
	cv::Mat reconstructed = cv::Mat::zeros(size, size, CV_32F);
	int centerX = size / 2;
	int centerYImg = size / 2;

	for (int y = 0; y < size; ++y) {
		for (int x = 0; x < size; ++x) {
			float dx = static_cast<float>(x - centerX);
			float dy = static_cast<float>(y - centerYImg);
			float r = std::sqrt(dx * dx + dy * dy);
			
			// 限制圆盘半径：只保留半径小于等于maxValidRadius的点
			if (r > static_cast<float>(maxValidRadius)) {
				reconstructed.at<float>(y, x) = 0.0f;
				continue;
			}

			int ri = static_cast<int>(std::floor(r + 0.5f));
			if (ri >= 0 && ri < radialSize) {
				reconstructed.at<float>(y, x) = radialProfile.at<float>(ri, 0);
			}
			else {
				reconstructed.at<float>(y, x) = 0.0f;
			}
		}
	}

	return reconstructed;
}

// 对单列执行完整的Abel反演流程（步骤a + 步骤b）
cv::Mat AbelReconstructor::reconstructAbelFromColumn(int columnIndex, bool saveIntermediate) const {
	if (processedImage.empty() || columnIndex < 0 || columnIndex >= processedImage.cols) {
		return cv::Mat();
	}

	// 步骤a: 构造0~180°范围内的完备投影数据
	int maxValidRadius = 0;
	cv::Mat proj1D = constructProjectionData(columnIndex, maxValidRadius);
	if (proj1D.empty()) {
		return cv::Mat();
	}

	// 步骤b: 基于完备数据利用Abel逆变换重建二维分布
	// 使用maxValidRadius限制圆盘半径
	int size = processedImage.rows; // 使用图像高度作为重建尺寸
	cv::Mat reconstructed = reconstruct2DFromProjection(proj1D, size, maxValidRadius);

	// 保存中间结果（如果启用）
	if (saveIntermediate) {
		// 保存投影数据
		saveMatTxt(proj1D, "projection_1d_col_" + std::to_string(columnIndex) + ".txt", false);
		std::cout << "    Column " << columnIndex << " - Max valid radius: " << maxValidRadius << std::endl;

		// 保存重建结果的可视化
		cv::Mat reconVis;
		double minVal, maxVal;
		cv::minMaxLoc(reconstructed, &minVal, &maxVal);
		if (maxVal > minVal) {
			reconVis = (reconstructed - minVal) * 255.0 / (maxVal - minVal);
			reconVis.convertTo(reconVis, CV_8U);
		}
		else {
			reconVis = cv::Mat::zeros(reconstructed.size(), CV_8U);
		}
		std::string slicePath = intermediatePath + "abel_slice_col_" + std::to_string(columnIndex) + ".png";
		cv::imwrite(slicePath, reconVis);
	}

	return reconstructed;
}

// ============================================================
// 步骤c: 重复上述过程，将每次重建出的二维分布沿着轴向堆叠
//        起来构成等离子体三维分布，最后输出保存点云文件（.ply）
// ============================================================
bool AbelReconstructor::perform3DReconstruction(const std::string& inputImage, bool saveIntermediate) {
	// #region agent log
	abel_agent_log("H1", "perform3DReconstruction", "enter perform3DReconstruction",
		"{\"inputImage\":\"" + inputImage + "\",\"enableViewer\":" + std::to_string(ENABLE_POINT_CLOUD_VIEWER) + "}");
	// #endregion

	if (!loadProcessedImage(inputImage)) {
		return false;
	}

	std::cout << "=====================================================" << std::endl;
	std::cout << "3D Abel Transform Reconstruction" << std::endl;
	std::cout << "=====================================================" << std::endl;
	std::cout << "Algorithm Flow:" << std::endl;
	std::cout << "  a) Extract column pixels as projections, construct 0~180° complete data" << std::endl;
	std::cout << "  b) Reconstruct 2D distribution using Abel inverse transform" << std::endl;
	std::cout << "  c) Stack all 2D distributions along axis to form 3D distribution" << std::endl;
	std::cout << "=====================================================" << std::endl;
	std::cout << "Starting 3D reconstruction..." << std::endl;
	std::cout << "  Image size: " << imageWidth << "x" << imageHeight << std::endl;
	std::cout << "  Number of columns: " << imageWidth << std::endl;

	int numColumns = processedImage.cols;
	int sliceSize = processedImage.rows;  // 每个切片的尺寸

	// 用于保存点云数据
	struct Point3D {
		float x;
		float y;
		float z;
		float intensity;
	};
	std::vector<Point3D> points;
	points.reserve(static_cast<size_t>(numColumns) * sliceSize * sliceSize / 4);  // 预分配空间

	float globalMaxIntensity = 0.0f;

	// 步骤c: 循环处理每一列，重复步骤a和b
	for (int col = 0; col < numColumns; ++col) {
		// 对当前列执行Abel反演（步骤a和b）
		cv::Mat slice2D = reconstructAbelFromColumn(col, saveIntermediate && (col == numColumns / 2));
		if (slice2D.empty()) {
			continue;
		}

		// 保存中间一列的结果到 reconstructedImage
		if (col == numColumns / 2) {
			reconstructedImage = slice2D.clone();

			// 保存中间列的二维重建结果
			if (saveIntermediate) {
				cv::Mat reconVis;
				double minVal, maxVal;
				cv::minMaxLoc(reconstructedImage, &minVal, &maxVal);
				if (maxVal > minVal) {
					reconVis = (reconstructedImage - minVal) * 255.0 / (maxVal - minVal);
					reconVis.convertTo(reconVis, CV_8U);
				}
				else {
					reconVis = cv::Mat::zeros(reconstructedImage.size(), CV_8U);
				}
				std::string reconPath = intermediatePath + "abel_reconstruction_middle.png";
				cv::imwrite(reconPath, reconVis);
				std::cout << "  Saved middle-column Abel reconstruction: " << reconPath << std::endl;
			}
		}

		// 将当前切片中的非零体素转成点云
		int centerXY = sliceSize / 2;
		float zCoord = static_cast<float>(col);  // z坐标就是列索引

		// 获取当前列的有效半径（需要重新计算，因为reconstructAbelFromColumn内部会计算）
		int maxValidRadius = 0;
		cv::Mat tempProj = constructProjectionData(col, maxValidRadius);
		if (tempProj.empty()) {
			continue;
		}

		// 计算当前切片的最大值，用于阈值判断
		double minSliceVal, maxSliceVal;
		cv::minMaxLoc(slice2D, &minSliceVal, &maxSliceVal);
		const float sliceMax = static_cast<float>(maxSliceVal);
		
		// 若本切片几乎没有信号，则跳过
		if (sliceMax <= 0.0f) {
			continue;
		}

		// 阈值比例，保留大于设定比例峰值的点，使用宏定义
		const float sliceThresholdRatio = POINT_CLOUD_THRESHOLD_RATIO;
		const float sliceThreshold = sliceThresholdRatio * sliceMax;

		// 应用半径限制：只保留半径在有效范围内的点
		const float maxRadius = static_cast<float>(maxValidRadius);

		for (int y = 0; y < sliceSize; ++y) {
			for (int x = 0; x < sliceSize; ++x) {
				float val = slice2D.at<float>(y, x);
				// 仅保留强度明显高于噪声的点
				if (val <= sliceThreshold) {
					continue;
				}

				// 计算当前点到圆盘中心的距离
				float dx = static_cast<float>(x - centerXY);
				float dy = static_cast<float>(y - centerXY);
				float r = std::sqrt(dx * dx + dy * dy);

				// 应用半径限制：只保留半径在有效范围内的点，剔除外部圆环
				if (r > maxRadius) {
					continue; // 超出有效半径，跳过
				}

				Point3D p;
				// 以圆盘中心为原点，像素尺寸为1
				p.x = dx;
				p.y = dy;
				p.z = zCoord;
				p.intensity = val;
				points.push_back(p);
				if (val > globalMaxIntensity) {
					globalMaxIntensity = val;
				}
			}
		}

		// 每处理50列输出一次进度
		if ((col + 1) % 50 == 0 || col == numColumns - 1) {
			std::cout << "  Processed column " << col + 1 << "/" << numColumns
				<< ", current points: " << points.size() << std::endl;
		}
	}

	if (points.empty()) {
		std::cerr << "No points generated for point cloud." << std::endl;
		// #region agent log
		abel_agent_log("H2", "perform3DReconstruction", "no points generated", "{\"points\":0}");
		// #endregion
		return false;
	}

	// 将点云保存为PLY文件
	std::string plyPath = outputPath + "abel_reconstruction_points.ply";
	std::ofstream ofs(plyPath);
	if (!ofs.is_open()) {
		std::cerr << "Failed to open PLY file for writing: " << plyPath << std::endl;
		return false;
	}

	// 写入PLY文件头
	ofs << "ply\n";
	ofs << "format ascii 1.0\n";
	ofs << "element vertex " << points.size() << "\n";
	ofs << "property float x\n";
	ofs << "property float y\n";
	ofs << "property float z\n";
	ofs << "property uchar red\n";
	ofs << "property uchar green\n";
	ofs << "property uchar blue\n";
	ofs << "end_header\n";

	// #region agent log
	abel_agent_log("H3", "perform3DReconstruction", "point cloud stats",
		"{\"points\":" + std::to_string(points.size()) +
		",\"enableViewer\":" + std::to_string(ENABLE_POINT_CLOUD_VIEWER) + "}");
	// #endregion

	// 写入点云数据
	// 颜色映射：发射系数（强度）越大，颜色越白
	// 使用改进的映射策略以增强对比度
	
	// 计算强度统计信息，用于更好的颜色映射
	float minIntensity = points[0].intensity;
	float maxIntensity = globalMaxIntensity;
	float sumIntensity = 0.0f;
	for (const auto& p : points) {
		if (p.intensity < minIntensity) minIntensity = p.intensity;
		sumIntensity += p.intensity;
	}
	float meanIntensity = sumIntensity / static_cast<float>(points.size());
	
	std::cout << "  Intensity statistics:" << std::endl;
	std::cout << "    Min: " << minIntensity << std::endl;
	std::cout << "    Max: " << maxIntensity << std::endl;
	std::cout << "    Mean: " << meanIntensity << std::endl;
	
	// 使用对数映射增强对比度，让不同强度的点有更明显的颜色差异
	const float intensityRange = maxIntensity - minIntensity;
	const float logBase = std::exp(1.0f) - 1.0f;

#if ENABLE_POINT_CLOUD_VIEWER
	// 准备PCL点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud->width = static_cast<uint32_t>(points.size());
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(points.size());
#endif

	for (size_t i = 0; i < points.size(); ++i) {
		const auto& p = points[i];

		// 计算灰度值（用于灰度模式或强度参考）
		float norm = (intensityRange > 0.0f)
			? ((p.intensity - minIntensity) / intensityRange)
			: 0.0f;
		if (norm < 0.0f) norm = 0.0f;
		if (norm > 1.0f) norm = 1.0f;

		float logNorm = std::log(1.0f + norm * logBase) / std::log(std::exp(1.0f));
		int gray = static_cast<int>(logNorm * 255.0f + 0.5f);
		if (gray < 0) gray = 0;
		if (gray > 255) gray = 255;

		// 根据宏选择颜色：0=灰度，1=固定颜色
		uint8_t r, g, b;
#if POINT_CLOUD_COLOR_MODE == 0
		r = static_cast<uint8_t>(gray);
		g = static_cast<uint8_t>(gray);
		b = static_cast<uint8_t>(gray);
#else
		r = static_cast<uint8_t>(POINT_CLOUD_COLOR_R);
		g = static_cast<uint8_t>(POINT_CLOUD_COLOR_G);
		b = static_cast<uint8_t>(POINT_CLOUD_COLOR_B);
#endif

		// 写入点坐标和RGB颜色值
		ofs << p.x << " " << p.y << " " << p.z << " "
			<< static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << "\n";

#if ENABLE_POINT_CLOUD_VIEWER
		// 同时填充到PCL点云
		pcl::PointXYZRGB pt;
		pt.x = p.x;
		pt.y = p.y;
		pt.z = p.z;
		pt.r = r;
		pt.g = g;
		pt.b = b;
		cloud->points[i] = pt;
#endif
	}

	ofs.close();

	std::cout << "3D Abel reconstruction completed." << std::endl;
	std::cout << "  Total points: " << points.size() << std::endl;
	std::cout << "  Point cloud saved to: " << plyPath << std::endl;

#if ENABLE_POINT_CLOUD_VIEWER
	// 使用PCL可视化点云
	std::cout << "  Launching PCL viewer for point cloud visualization..." << std::endl;
	// #region agent log
	abel_agent_log("H4", "perform3DReconstruction", "launch viewer", "{\"points\":" + std::to_string(points.size()) + "}");
	// #endregion
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Abel Reconstruction Point Cloud"));
	viewer->setBackgroundColor(0.0, 0.0, 0.0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "abel_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "abel_cloud");
	viewer->addCoordinateSystem(10.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
#else
	// 未启用 viewer 的提示
	std::cout << "  Point cloud viewer disabled (ENABLE_POINT_CLOUD_VIEWER=0)." << std::endl;
	// #region agent log
	abel_agent_log("H4", "perform3DReconstruction", "viewer disabled", "{\"enableViewer\":0}");
	// #endregion
#endif

	return true;
}

// 测试函数：只重建中间一列
bool AbelReconstructor::testMiddleColumnReconstructionComplex(const std::string& inputImage) {
	if (!loadProcessedImage(inputImage)) {
		return false;
	}

	std::cout << "Testing Abel Transform reconstruction (middle column)..." << std::endl;

	int middleCol = processedImage.cols / 2;
	std::cout << "Testing with middle column: " << middleCol << std::endl;

	// 执行Abel反演重建（会保存中间结果）
	reconstructedImage = reconstructAbelFromColumn(middleCol, true);
	if (reconstructedImage.empty()) {
		std::cerr << "Failed to reconstruct middle column." << std::endl;
		return false;
	}

	// 保存最终重建结果
	double minVal, maxVal;
	cv::minMaxLoc(reconstructedImage, &minVal, &maxVal);
	cv::Mat reconVis;
	if (maxVal > minVal) {
		reconVis = (reconstructedImage - minVal) * 255.0 / (maxVal - minVal);
		reconVis.convertTo(reconVis, CV_8U);
	}
	else {
		reconVis = cv::Mat::zeros(reconstructedImage.size(), CV_8U);
	}

	std::string slicePath = outputPath + "middle_slice_reconstruction.png";
	cv::imwrite(slicePath, reconVis);
	std::cout << "  Saved middle column Abel reconstruction to: " << slicePath << std::endl;

		return true;
	}

// 获取中间切片
cv::Mat AbelReconstructor::getMiddleSlice() const {
	if (reconstructedImage.empty()) {
		return cv::Mat();
	}

	double minVal, maxVal;
	cv::minMaxLoc(reconstructedImage, &minVal, &maxVal);
	cv::Mat vis;
	if (maxVal > minVal) {
		vis = (reconstructedImage - minVal) * 255.0 / (maxVal - minVal);
		vis.convertTo(vis, CV_8U);
	}
	else {
		vis = cv::Mat::zeros(reconstructedImage.size(), CV_8U);
	}
	return vis;
}

// 保存结果
bool AbelReconstructor::saveResults(const std::string& baseName) {
	if (reconstructedImage.empty()) {
		std::cerr << "No reconstruction available to save." << std::endl;
		return false;
	}

	cv::Mat vis = getMiddleSlice();
	std::string slicePath = outputPath + baseName + "_reconstruction.png";
	cv::imwrite(slicePath, vis);
	std::cout << "Saved reconstruction to: " << slicePath << std::endl;

	// 保存TXT数据
	saveMatTxt(reconstructedImage, outputPath + baseName + "_reconstruction.txt", false);
	std::cout << "Saved reconstruction TXT to: " << outputPath + baseName + "_reconstruction.txt" << std::endl;

	return true;
}
