## TDR_plasma 项目说明（Abel 逆变换等离子体重建）

本项目用于对**轴对称（圆柱对称）等离子体**的侧视投影图像进行数值重建，采用**离散 Abel 逆变换**从单角度投影恢复二维切片的发射系数分布，并将所有列的二维切片堆叠成三维分布，最终以**点云（PLY）**形式输出，便于在 CloudCompare / PCL Viewer 等软件中进行可视化分析。

项目主要依赖：
- **OpenCV**：图像读写、预处理、矩阵运算
- **PCL (Point Cloud Library)**：三维点云构建与可视化
- **Visual Studio + CMake / MSBuild**：Windows 下的编译与调试环境

---

## 算法与流程概述

整个重建流程可以概括为三大部分：

1. **图像预处理（`preprocess.cpp`）**
2. **Abel 逆变换二维切片重建（`abel_reconstruction.cpp`）**
3. **三维点云生成与输出（`abel_reconstruction.cpp` / `main.cpp`）**

所有可调参数统一通过 `main.cpp` 顶部的宏定义进行配置，便于一次性设置输入数据和重建参数，然后直接运行程序完成整个流程。

### 1. 图像预处理（Image Preprocessing）

预处理的目标是从原始等离子体侧视图中，得到一个**标准化、去噪、对齐的输入图像**，为后续 Abel 逆变换做准备。主要步骤包括：

- **尺寸统一**：使用宏 `TARGET_IMAGE_SIZE` 将输入图像缩放到统一分辨率（例如 512×512）。
- **去噪滤波**：使用中值滤波，核大小由 `MEDIAN_FILTER_KERNEL_SIZE` 控制。
- **掩膜与背景抑制**：通过阈值 `MASK_THRESHOLD` 去除背景噪声，保留主要等离子体区域。
- **结果保存**：中间结果（如 `resized_xx.png`, `filtered_xx.png`, `mask_xx.png`, `processed_xx.png` 等）保存到 `data/proceed/` 或 `data/output/` 目录中，便于检查预处理效果。

预处理结束后，会得到一幅**处理后的灰度图像**（通常命名为 `processed_*.png`），作为 Abel 逆变换的输入。

### 2. Abel 逆变换二维切片重建

对于**轴对称**等离子体，其侧视投影可以看作是对真实三维分布在某一方向上的积分。在每一个列方向上，这种积分在数学上满足 **Abel 投影/Abel 变换** 模型，因此可以通过 Abel 逆变换从一维投影恢复半径方向的发射系数分布。

在代码中，核心流程大致为：

1. **列投影提取（`constructProjectionData`）**
   - 从预处理后的图像中，取某一列的像素作为一维投影数据。
   - 通过上下对称平均，构造轴对称假设下的 1D 径向投影 \(P(y)\)。
   - 根据强度阈值自动确定**有效半径 `maxValidRadius`**，用于在重建时限制盘的半径，只使用掩膜内部的可靠数据。

2. **离散 Abel 逆变换（`reconstruct2DFromProjection`）**
   - 对一维投影 \(P(y)\) 进行差分，计算离散形式的导数 \(dP/dy\)。
   - 使用离散 Abel 逆公式计算径向发射系数 \(f(r)\)，典型形式为：
     \[
       f(r_i) \approx -\frac{1}{\pi} \sum_{j=i}^{N-1} \frac{dP}{dy}(y_j)\,\frac{1}{\sqrt{y_j^2 - r_i^2}}
     \]
   - 将一维的 \(f(r)\) 在二维平面上按半径复制，生成**二维轴对称分布切片**（一个发光盘）。
   - 在生成二维切片时，半径超过 `maxValidRadius` 的区域会被置零，从而去除掩膜外的无效数据。

3. **单列重建封装（`reconstructAbelFromColumn`）**
   - 对某一列索引 `colIndex`：
     - 调用 `constructProjectionData` 生成 1D 投影和有效半径。
     - 调用 `reconstruct2DFromProjection` 得到该列对应的二维切片图像。
   - 可以额外将某一列的结果保存为图像（例如 `abel_slice_col_256.png`）以便检查。

在早期版本中，曾经采用过：

- **1D FFT → 填频域圆盘 → 2D IFFT** 的方式尝试重建二维分布，

但这一流程严格对应的是**Radon 投影 + 中心切片定理**的情形，需要 0–180° 多角度投影数据，对于**仅有单角度投影、并且对象是轴对称的 Abel 场景**并不适用。因此在当前版本中，已经完全替换为**离散 Abel 逆变换**，避免了灰度发散、重建不正确等问题。

### 3. 三维点云生成与输出

在得到每一列的二维切片之后，通过对所有列进行重建并堆叠，可以得到等离子体在三维空间中的发射系数分布。主要步骤如下：

1. **对所有列循环重建（`perform3DReconstruction`）**
   - 对预处理后图像的每一列（或按步长选取部分列），调用 `reconstructAbelFromColumn` 得到一个二维切片。
   - 切片的坐标通常可以理解为：
     - \(x\)：沿图像列方向（对应装置轴向或某一物理方向）
     - \(y, z\)：由 Abel 逆变换得到的径向平面坐标

2. **点云构建**
   - 将二维切片中所有强度大于阈值的像素转换为三维点：
     - 点坐标：\((x, y, z)\)
     - 点颜色：采用**发射系数强度映射到灰度**（或由宏定义控制的颜色）
   - 使用 `pcl::PointCloud<pcl::PointXYZRGB>` 保存这些三维点。

3. **颜色映射与可视化**
   - 颜色映射采用**对数型映射**增强对比度：发射系数较高的点更接近白色。
   - 通过宏定义可以控制：
     - 是否启用 PCL Viewer 实时查看（`ENABLE_POINT_CLOUD_VIEWER`）
     - 点云颜色模式、基础 RGB（例如默认白色）

4. **PLY 文件输出**
   - 将最终点云保存为 `.ply` 文件，便于在 CloudCompare 等软件中进一步分析。
   - 在 CloudCompare 中可以利用标量场或 RGB 显示出强度的空间分布。

---

## 使用方法

### 1. 环境准备

- **操作系统**：Windows 10 / 11
- **编译器 / IDE**：Visual Studio（推荐 VS2015 及以上，已自带 MSVC 编译器）
- **依赖库**：
  - OpenCV（建议通过 vcpkg 或手动安装）
  - PCL（同样建议通过 vcpkg 安装）

典型的 vcpkg 安装示例（仅供参考）：

```bash
vcpkg install opencv pcl
```

然后在 Visual Studio 工程属性中，将 vcpkg 集成的 include / lib 目录正确添加到搜索路径中。

### 2. 数据准备

1. 将原始等离子体侧视图像（如 `6KV.png`, `7KV.png`, `8KV.png`, `9KV.png`）放入：
   - `data/input/`
2. 使用 `config.txt` 或在 `main.cpp` 的宏中指定：
   - 使用哪一张图像作为当前重建输入
   - 数据输出目录（通常为 `data/output/`）

### 3. 参数配置（宏定义）

所有可调参数统一集中在 `src/main.cpp` 的文件开头，通过 `#define` 宏定义设置。例如：

- **输入/输出路径相关**
  - 输入图像文件名
  - 预处理结果与重建结果保存路径

- **预处理相关**
  - `TARGET_IMAGE_SIZE`：目标图像尺寸（如 512）
  - `MEDIAN_FILTER_KERNEL_SIZE`：中值滤波核大小
  - `MASK_THRESHOLD`：掩膜阈值

- **Abel 重建相关**
  - 投影列范围、是否只重建中间列测试
  - 有效半径阈值、噪声过滤阈值等

- **点云与可视化相关**
  - `ENABLE_POINT_CLOUD_VIEWER`：是否打开 PCL Viewer 实时显示点云
  - `POINT_CLOUD_COLOR_MODE`：颜色模式
  - `POINT_CLOUD_COLOR_R/G/B`：默认点颜色（默认可设置为白色）

在修改这些宏之后，重新编译并运行程序即可生效，无需在运行时手动选择 `processedImage`。

### 4. 编译与运行

1. 使用 Visual Studio 打开 `TDR_plasma.sln`。
2. 确认工程引用的 OpenCV / PCL 库路径正确（可通过 vcpkg 集成或手动配置）。
3. 根据需要在 `main.cpp` 顶部调整宏定义参数。
4. 选择 `Debug` 或 `Release` 配置，生成解决方案（Build Solution）。
5. 运行可执行程序（例如 `x64/Debug/TDR_plasma.exe`）。

程序运行后，将自动：

1. 读取指定输入图像；
2. 执行预处理，并在 `data/proceed/` 中保存中间结果；
3. 对所有（或选定的）列执行 Abel 逆变换重建二维切片；
4. 将所有切片堆叠成三维点云，并保存为 `.ply` 文件；
5. 如启用 PCL Viewer，将弹出窗口显示三维点云。

### 5. 结果查看

- **中间二维结果**：
  - 在 `data/proceed/` 与 `data/output/` 中查看：
    - `resized_*.png` / `filtered_*.png` / `mask_*.png` / `processed_*.png`
    - 某些列的 Abel 切片结果，如 `abel_slice_col_256.png`
    - 整体中间切片投影重建图，如 `middle_slice_reconstruction.png`

- **三维点云结果**：
  - 在程序输出的 `.ply` 文件路径处，使用 CloudCompare 打开：
    - 可以按 RGB 或标量场方式显示点云颜色
    - 可通过点大小、着色方式等参数，进一步优化可视化效果

---

## 与其他重建方法的关系（简要说明）

项目当前版本采用的是**离散 Abel 逆变换**，其适用前提是：

- 被重建对象在某一轴向上具有**旋转对称（圆柱对称）**结构；
- 输入为该对象从某一固定方向看到的**单角度侧视投影**。

在此条件下：

- 单角度投影满足 Abel 模型，可以通过 Abel 逆变换恢复二维径向分布；
- 不需要 0–180° 多角度数据，也不适用标准 Radon + FBP 的多角度 CT 流程；
- 之前尝试的 **1D FFT → 填频域圆盘 → 2D IFFT** 更适合 Radon 变换场景，而非 Abel 场景，因此被完整替换。

如果后续需要对**非轴对称**对象进行成像（典型 CT 场景），则需要采集多角度投影并使用 Radon 逆变换（如 FBP）；这已经超出了当前项目的设计范围。

---

## 备注

- 所有源文件均应以 **UTF-8 编码** 保存，以避免中文注释或路径导致的编译问题。
- 若需进一步了解 Abel 逆变换、Radon 变换、FBP 等理论差异，可以在项目基础上配合相关文献进行验证与对比。

## 数据图片

![9KV](D:\Desktop\TDR_plasma\data\input\9KV.png)

## 重建的二维分布

![abel_slice_col_256](D:\Desktop\TDR_plasma\data\proceed\abel_slice_col_256.png)

## 重建图片

![9dc073d3798ef2e69f43cc5f0e35c190](C:\Users\16344\Documents\Tencent Files\1634499644\nt_qq\nt_data\Pic\2026-01\Ori\9dc073d3798ef2e69f43cc5f0e35c190.png)
