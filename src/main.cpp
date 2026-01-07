#include <iostream>
#include "preprocess.h"
#include "abel_reconstruction.h"

// ============================================================================
// 可调参数宏定义 - 便于调试和调整
// ============================================================================

// 输入输出路径配置
#define INPUT_IMAGE_NAME          "9KV.png"              // 输入图像文件名（放在../data/input/目录下）
#define INPUT_PATH                "../data/input/"        // 输入图像路径
#define PROCESSED_PATH            "../data/proceed/"      // 预处理结果保存路径
#define OUTPUT_PATH               "../data/output/"       // 最终输出路径

// 图像预处理参数
#define TARGET_IMAGE_SIZE         512                    // 目标图像尺寸（预处理后图像大小）
#define MEDIAN_FILTER_KERNEL_SIZE 3                      // 中值滤波核大小（必须是奇数）
#define MASK_THRESHOLD            0.145                    // 掩膜阈值（0.0-1.0，用于创建掩膜）

// Abel重建参数
#define PROJECTION_THRESHOLD      1e-3f                  // 有效半径检测阈值（用于判断投影数据是否有效）
#define MIN_CONSECUTIVE_ZEROS     5                      // 连续零计数阈值（连续多少个位置小于阈值才认为有效数据结束）
#define EPSILON                   1e-6f                  // 数值稳定性参数（用于避免除零错误）
#define POINT_CLOUD_THRESHOLD_RATIO 0.01f                // 点云生成阈值比例（保留大于此比例峰值的点，0.01表示1%）

// 中间结果保存选项
#define SAVE_INTERMEDIATE_RESULTS  true                  // 是否保存中间处理结果（true/false）
#define TEST_MIDDLE_COLUMN         true                  // 是否测试中间列重建（true/false）

// 点云显示选项
#define ENABLE_POINT_CLOUD_VIEWER  1                     // 是否使用PCL在程序中直接显示点云（1=显示，0=不显示）

// 点云颜色选项
// POINT_CLOUD_COLOR_MODE: 0 = 按强度灰度映射（默认），1 = 使用固定颜色
#define POINT_CLOUD_COLOR_MODE     1                     // 默认固定颜色
#define POINT_CLOUD_COLOR_R        255                   // 固定颜色 R 分量（0-255）
#define POINT_CLOUD_COLOR_G        255                   // 固定颜色 G 分量（0-255）
#define POINT_CLOUD_COLOR_B        255                   // 固定颜色 B 分量（0-255）

// ============================================================================

int main() {
	std::cout << "Plasma 3D Reconstruction - Abel Transform Version" << std::endl;
	std::cout << "=====================================================" << std::endl;

	try {
		// 设置输入图像文件名
		std::string originalImage = INPUT_IMAGE_NAME;
		std::string processedImage = "processed_" + originalImage;

		std::cout << "\n[STEP 1] Image Preprocessing" << std::endl;
		std::cout << "------------------------------------------" << std::endl;

		// 模块1：图像预处理
		ImagePreprocessor preprocessor;
		preprocessor.setInputPath(INPUT_PATH);
		preprocessor.setOutputPath(PROCESSED_PATH);
		preprocessor.setTargetSize(TARGET_IMAGE_SIZE);

		std::cout << "Processing original image: " << originalImage << std::endl;
		std::cout << "  Input path: " << INPUT_PATH << std::endl;
		std::cout << "  Output path: " << PROCESSED_PATH << std::endl;
		std::cout << "  Target size: " << TARGET_IMAGE_SIZE << "x" << TARGET_IMAGE_SIZE << std::endl;

		if (!preprocessor.processImage(originalImage, SAVE_INTERMEDIATE_RESULTS)) {
			std::cerr << "Failed to process image. Exiting." << std::endl;
			return 1;
		}

		std::cout << "Image preprocessing completed successfully." << std::endl;
		std::cout << "Processed image saved as: " << processedImage << std::endl;

		std::cout << "\n[STEP 2] Abel 3D Reconstruction" << std::endl;
		std::cout << "-----------------------------------------------------------------" << std::endl;

		// 模块2：Abel 3D重建
		AbelReconstructor reconstructor;
		reconstructor.setInputPath(PROCESSED_PATH);
		reconstructor.setOutputPath(OUTPUT_PATH);
		reconstructor.setIntermediatePath(PROCESSED_PATH);

		std::cout << "Loading processed image: " << processedImage << std::endl;

		// 测试中间列重建（可选）
		if (TEST_MIDDLE_COLUMN) {
			std::cout << "\n[TEST] Testing middle column reconstruction..." << std::endl;
			if (reconstructor.testMiddleColumnReconstructionComplex(processedImage)) {
				std::cout << "Middle column test completed successfully." << std::endl;
			}
			else {
				std::cout << "Warning: Middle column test failed." << std::endl;
			}
		}

		// 执行完整3D重建
		std::cout << "\n[FULL RECONSTRUCTION] Starting full 3D reconstruction..." << std::endl;

		if (reconstructor.perform3DReconstruction(processedImage, SAVE_INTERMEDIATE_RESULTS)) {
			std::cout << "3D reconstruction completed successfully." << std::endl;

			// 保存最终结果
			std::cout << "\n[Saving Results]" << std::endl;
			reconstructor.saveResults("result");
			std::cout << "Results saved to " << OUTPUT_PATH << std::endl;
		}
		else {
			std::cerr << "3D reconstruction failed." << std::endl;
			return 1;
		}

		std::cout << "\n[COMPLETE] Pipeline completed successfully!" << std::endl;

	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}

	std::cout << "\nPress Enter to exit..." << std::endl;
	std::cin.get();

	return 0;
}
