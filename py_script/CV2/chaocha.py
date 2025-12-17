import cv2
import numpy as np
import os
import json
from datetime import datetime
import matplotlib.pyplot as plt

class InstallationErrorAnalyzer:
    def __init__(self, camera_path, simulation_path, output_dir="./error_analysis"):
        """
        初始化安装误差分析器
        
        Args:
            camera_path: 相机实际拍摄图像路径
            simulation_path: 仿真设计图像路径
            output_dir: 输出目录
        """
        self.camera_path = camera_path
        self.simulation_path = simulation_path
        self.output_dir = output_dir
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 读取图像
        self.camera_img = cv2.imread(camera_path)
        self.simulation_img = cv2.imread(simulation_path)
        
        if self.camera_img is None or self.simulation_img is None:
            raise ValueError("无法加载图像文件")
        
        # 图像尺寸
        self.camera_height, self.camera_width = self.camera_img.shape[:2]
        self.sim_height, self.sim_width = self.simulation_img.shape[:2]
        
        # 存储结果
        self.results = {
            "analysis_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "camera_image": camera_path,
            "simulation_image": simulation_path,
            "image_dimensions": {
                "camera": {"width": self.camera_width, "height": self.camera_height},
                "simulation": {"width": self.sim_width, "height": self.sim_height}
            }
        }
    
    def analyze_with_feature_matching(self, threshold=30, min_matches=10):
        """
        使用特征点匹配进行详细误差分析
        
        Args:
            threshold: 像素差异阈值
            min_matches: 最小匹配点数量
            
        Returns:
            误差分析结果字典
        """
        print("开始特征点匹配分析...")
        
        # 转换为灰度图
        gray_camera = cv2.cvtColor(self.camera_img, cv2.COLOR_BGR2GRAY)
        gray_simulation = cv2.cvtColor(self.simulation_img, cv2.COLOR_BGR2GRAY)
        
        # 使用SIFT特征检测器（比ORB更稳定）
        sift = cv2.SIFT_create()
        kp1, des1 = sift.detectAndCompute(gray_camera, None)
        kp2, des2 = sift.detectAndCompute(gray_simulation, None)
        
        if des1 is None or des2 is None:
            print("警告: 无法检测到足够特征点")
            return self.analyze_with_simple_method(threshold)
        
        # FLANN匹配器
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)
        
        # 筛选好的匹配点
        good_matches = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)
        
        print(f"找到 {len(good_matches)} 个良好匹配点")
        
        if len(good_matches) < min_matches:
            print(f"警告: 匹配点不足({len(good_matches)} < {min_matches})，使用简单方法")
            return self.analyze_with_simple_method(threshold)
        
        # 获取匹配点坐标
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        # 计算单应性矩阵
        homography_matrix, mask = cv2.findHomography(
            dst_pts, src_pts, cv2.RANSAC, ransacReprojThreshold=5.0
        )
        
        if homography_matrix is None:
            print("警告: 无法计算单应性矩阵")
            return self.analyze_with_simple_method(threshold)
        
        # 应用透视变换
        projected_simulation = cv2.warpPerspective(
            self.simulation_img, 
            homography_matrix, 
            (self.camera_width, self.camera_height)
        )
        
        # 保存投影图像
        projected_path = os.path.join(self.output_dir, "projected_simulation.jpg")
        cv2.imwrite(projected_path, projected_simulation)
        
        # 计算像素级误差
        pixel_error = self._calculate_pixel_error(
            self.camera_img, projected_simulation, threshold
        )
        
        # 计算几何误差
        geometric_error = self._calculate_geometric_error(
            src_pts, dst_pts, homography_matrix, mask
        )
        
        # 生成可视化结果
        self._generate_visualizations(
            kp1, kp2, good_matches, mask,
            self.camera_img, projected_simulation,
            pixel_error["diff_image"]
        )
        
        # 合并结果
        self.results.update({
            "method": "feature_matching",
            "num_matches": len(good_matches),
            "homography_matrix": homography_matrix.tolist(),
            "pixel_error": pixel_error,
            "geometric_error": geometric_error,
            "projected_image": projected_path
        })
        
        return self.results
    
    def analyze_with_simple_method(self, threshold=30):
        """
        使用简单缩放方法进行误差分析
        """
        print("使用简单缩放方法分析...")
        
        # 计算缩放比例
        scale = min(self.camera_width / self.sim_width, 
                   self.camera_height / self.sim_height)
        
        # 缩放仿真图像
        new_width = int(self.sim_width * scale)
        new_height = int(self.sim_height * scale)
        resized_simulation = cv2.resize(self.simulation_img, (new_width, new_height))
        
        # 创建画布并居中放置
        canvas = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        start_y = (self.camera_height - new_height) // 2
        start_x = (self.camera_width - new_width) // 2
        canvas[start_y:start_y+new_height, start_x:start_x+new_width] = resized_simulation
        
        # 保存投影图像
        projected_path = os.path.join(self.output_dir, "projected_simulation_simple.jpg")
        cv2.imwrite(projected_path, canvas)
        
        # 计算像素级误差
        pixel_error = self._calculate_pixel_error(self.camera_img, canvas, threshold)
        
        # 计算缩放误差
        scale_error = {
            "scale_factor": scale,
            "translation_x": start_x,
            "translation_y": start_y,
            "rotation_angle": 0  # 简单方法假设无旋转
        }
        
        self.results.update({
            "method": "simple_scaling",
            "pixel_error": pixel_error,
            "geometric_error": scale_error,
            "projected_image": projected_path
        })
        
        return self.results
    
    def _calculate_pixel_error(self, img1, img2, threshold):
        """
        计算像素级误差
        """
        # 转换为灰度图
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # 计算绝对差值
        diff = cv2.absdiff(gray1, gray2)
        
        # 应用阈值
        _, thresholded = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
        
        # 计算统计信息
        total_pixels = diff.size
        error_pixels = np.count_nonzero(thresholded)
        error_percentage = (error_pixels / total_pixels) * 100
        
        # 计算平均误差和最大误差
        mean_error = np.mean(diff)
        max_error = np.max(diff)
        
        # 保存差异图像
        diff_path = os.path.join(self.output_dir, "pixel_difference.jpg")
        cv2.imwrite(diff_path, diff)
        
        thresholded_path = os.path.join(self.output_dir, "thresholded_difference.jpg")
        cv2.imwrite(thresholded_path, thresholded)
        
        return {
            "error_percentage": float(error_percentage),
            "error_pixels": int(error_pixels),
            "total_pixels": int(total_pixels),
            "mean_error": float(mean_error),
            "max_error": int(max_error),
            "diff_image": diff_path,
            "thresholded_image": thresholded_path
        }
    
    def _calculate_geometric_error(self, src_pts, dst_pts, homography, mask):
        """
        计算几何误差（平移、旋转、缩放）
        """
        if mask is None:
            return {"error": "无法计算几何误差"}
        
        # 筛选内点
        inlier_src = src_pts[mask.ravel() == 1]
        inlier_dst = dst_pts[mask.ravel() == 1]
        
        if len(inlier_src) < 4:
            return {"error": "内点不足"}
        
        # 计算平均平移误差
        translation_errors = []
        for s, d in zip(inlier_src, inlier_dst):
            # 将仿真点变换到相机坐标系
            point = np.array([d[0][0], d[0][1], 1])
            transformed = homography @ point
            transformed = transformed / transformed[2]
            
            # 计算误差
            error = np.sqrt((s[0][0] - transformed[0])**2 + (s[0][1] - transformed[1])**2)
            translation_errors.append(error)
        
        mean_translation_error = np.mean(translation_errors)
        max_translation_error = np.max(translation_errors)
        
        # 从单应性矩阵提取旋转角度
        # 假设小角度近似
        rotation_angle = np.arctan2(homography[1, 0], homography[0, 0]) * 180 / np.pi
        
        # 计算缩放因子
        scale_x = np.sqrt(homography[0, 0]**2 + homography[1, 0]**2)
        scale_y = np.sqrt(homography[0, 1]**2 + homography[1, 1]**2)
        mean_scale = (scale_x + scale_y) / 2
        
        return {
            "mean_translation_error_pixels": float(mean_translation_error),
            "max_translation_error_pixels": float(max_translation_error),
            "rotation_angle_degrees": float(rotation_angle),
            "scale_factor": float(mean_scale),
            "scale_x": float(scale_x),
            "scale_y": float(scale_y),
            "num_inliers": int(len(inlier_src))
        }
    
    def _generate_visualizations(self, kp1, kp2, matches, mask,
                               camera_img, projected_img, diff_img):
        """
        生成可视化结果
        """
        # 1. 绘制匹配点
        draw_params = dict(matchColor=(0, 255, 0),
                          singlePointColor=None,
                          matchesMask=mask.ravel().tolist() if mask is not None else None,
                          flags=2)
        
        match_img = cv2.drawMatches(
            camera_img, kp1, 
            self.simulation_img, kp2, 
            matches, None, **draw_params
        )
        
        match_path = os.path.join(self.output_dir, "feature_matches.jpg")
        cv2.imwrite(match_path, match_img)
        
        # 2. 创建对比图
        comparison = np.hstack([camera_img, projected_img])
        comparison_path = os.path.join(self.output_dir, "comparison.jpg")
        cv2.imwrite(comparison_path, comparison)
        
        # 3. 创建误差热力图
        diff = cv2.absdiff(
            cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY),
            cv2.cvtColor(projected_img, cv2.COLOR_BGR2GRAY)
        )
        
        # 归一化并应用颜色映射
        diff_normalized = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)
        heatmap = cv2.applyColorMap(diff_normalized.astype(np.uint8), cv2.COLORMAP_JET)
        
        heatmap_path = os.path.join(self.output_dir, "error_heatmap.jpg")
        cv2.imwrite(heatmap_path, heatmap)
        
        self.results["visualizations"] = {
            "feature_matches": match_path,
            "comparison": comparison_path,
            "error_heatmap": heatmap_path
        }
    
    def generate_report(self):
        """
        生成详细的误差分析报告
        """
        report_path = os.path.join(self.output_dir, "error_analysis_report.json")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        
        # 生成文本报告
        txt_report = self._generate_text_report()
        txt_path = os.path.join(self.output_dir, "error_summary.txt")
        
        with open(txt_path, 'w', encoding='utf-8') as f:
            f.write(txt_report)
        
        print(f"详细报告已保存到: {report_path}")
        print(f"摘要报告已保存到: {txt_path}")
        
        return report_path, txt_path
    
    def _generate_text_report(self):
        """
        生成文本格式的误差摘要
        """
        report = "=" * 60 + "\n"
        report += "工装安装误差分析报告\n"
        report += "=" * 60 + "\n\n"
        
        report += f"分析时间: {self.results.get('analysis_time', 'N/A')}\n"
        report += f"分析方法: {self.results.get('method', 'N/A')}\n\n"
        
        # 像素误差
        pixel_error = self.results.get('pixel_error', {})
        if pixel_error:
            report += "像素级误差分析:\n"
            report += f"  误差像素比例: {pixel_error.get('error_percentage', 0):.2f}%\n"
            report += f"  平均像素误差: {pixel_error.get('mean_error', 0):.2f}\n"
            report += f"  最大像素误差: {pixel_error.get('max_error', 0)}\n\n"
        
        # 几何误差
        geometric_error = self.results.get('geometric_error', {})
        if geometric_error and 'error' not in geometric_error:
            report += "几何误差分析:\n"
            report += f"  平均平移误差: {geometric_error.get('mean_translation_error_pixels', 0):.2f} 像素\n"
            report += f"  最大平移误差: {geometric_error.get('max_translation_error_pixels', 0):.2f} 像素\n"
            report += f"  旋转角度: {geometric_error.get('rotation_angle_degrees', 0):.2f} 度\n"
            report += f"  缩放因子: {geometric_error.get('scale_factor', 1):.4f}\n\n"
        
        # 判断是否超差
        tolerance = 5.0  # 5%误差阈值
        error_percent = pixel_error.get('error_percentage', 0)
        
        report += "安装质量评估:\n"
        if error_percent > tolerance:
            report += f"  ❌ 安装超差！误差({error_percent:.2f}%)超过阈值({tolerance}%)\n"
            report += "  建议: 需要重新调整工装安装位置\n"
        else:
            report += f"  ✅ 安装合格！误差({error_percent:.2f}%)在允许范围内\n"
            report += "  建议: 安装符合要求，可继续使用\n"
        
        report += "\n" + "=" * 60
        
        return report

def main():
    """
    主函数示例
    """
    # 图像路径
    camera_path = "./potor/img_v3_02sp_7a5ee115-b019-4260-81a5-9fbd05e0e76g.jpg"
    simulation_path = "./potor/img_v3_02sp_e51b3634-66a9-4d6b-9b3d-69e53271910g.jpg"
    
    try:
        # 创建分析器
        analyzer = InstallationErrorAnalyzer(
            camera_path=camera_path,
            simulation_path=simulation_path,
            output_dir="./error_analysis_results"
        )
        
        # 执行分析（优先使用特征点匹配）
        print("开始进行安装误差分析...")
        results = analyzer.analyze_with_feature_matching(threshold=30, min_matches=10)
        
                # 生成报告
        report_json, report_txt = analyzer.generate_report()
        
        # 打印摘要
        print("\n" + "="*60)
        print("安装误差分析完成！")
        print("="*60)
        
        # 从结果中提取关键信息
        pixel_error = results.get('pixel_error', {})
        geometric_error = results.get('geometric_error', {})
        
        if pixel_error:
            print(f"\n像素误差统计:")
            print(f"  误差比例: {pixel_error.get('error_percentage', 0):.2f}%")
            print(f"  误差像素数: {pixel_error.get('error_pixels', 0):,}")
            print(f"  平均误差: {pixel_error.get('mean_error', 0):.2f}")
            print(f"  最大误差: {pixel_error.get('max_error', 0)}")
        
        if geometric_error and 'error' not in geometric_error:
            print(f"\n几何误差统计:")
            print(f"  平移误差: {geometric_error.get('mean_translation_error_pixels', 0):.2f} 像素")
            print(f"  旋转角度: {geometric_error.get('rotation_angle_degrees', 0):.2f} 度")
            print(f"  缩放因子: {geometric_error.get('scale_factor', 1):.4f}")
        
        # 判断安装质量
        tolerance = 5.0
        error_percent = pixel_error.get('error_percentage', 0)
        
        print(f"\n安装质量评估:")
        if error_percent > tolerance:
            print(f"  ❌ 检测到安装超差！")
            print(f"  当前误差: {error_percent:.2f}% > 允许阈值: {tolerance}%")
            
            # 提供调整建议
            print(f"\n调整建议:")
            if geometric_error and 'error' not in geometric_error:
                rotation = geometric_error.get('rotation_angle_degrees', 0)
                if abs(rotation) > 1.0:
                    print(f"  1. 需要调整旋转角度约 {abs(rotation):.2f} 度")
                
                scale = geometric_error.get('scale_factor', 1)
                if abs(scale - 1) > 0.05:
                    print(f"  2. 需要调整安装距离，当前缩放因子: {scale:.4f}")
                
                trans_error = geometric_error.get('mean_translation_error_pixels', 0)
                if trans_error > 10:
                    print(f"  3. 需要调整位置偏移，当前平均偏移: {trans_error:.2f} 像素")
        else:
            print(f"  ✅ 安装符合要求！")
            print(f"  当前误差: {error_percent:.2f}% ≤ 允许阈值: {tolerance}%")
        
        print(f"\n详细报告已保存到: {report_json}")
        print(f"可视化结果保存在: ./error_analysis_results/")
        
    except Exception as e:
        print(f"分析过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

def batch_analysis(image_pairs):
    """
    批量分析多个图像对
    
    Args:
        image_pairs: 列表，每个元素为(camera_path, simulation_path, output_dir)
    """
    results_summary = []
    
    for i, (camera_path, simulation_path, output_dir) in enumerate(image_pairs, 1):
        print(f"\n{'='*60}")
        print(f"分析第 {i}/{len(image_pairs)} 对图像")
        print(f"相机图像: {camera_path}")
        print(f"仿真图像: {simulation_path}")
        print(f"{'='*60}")
        
        try:
            analyzer = InstallationErrorAnalyzer(
                camera_path=camera_path,
                simulation_path=simulation_path,
                output_dir=output_dir
            )
            
            # 尝试特征点匹配，失败则使用简单方法
            try:
                results = analyzer.analyze_with_feature_matching(threshold=30, min_matches=10)
            except:
                results = analyzer.analyze_with_simple_method(threshold=30)
            
            # 生成报告
            analyzer.generate_report()
            
            # 记录摘要
            error_percent = results.get('pixel_error', {}).get('error_percentage', 100)
            results_summary.append({
                'pair_id': i,
                'camera_image': camera_path,
                'simulation_image': simulation_path,
                'error_percentage': error_percent,
                'status': 'PASS' if error_percent <= 5.0 else 'FAIL',
                'output_dir': output_dir
            })
            
            print(f"✓ 第 {i} 对图像分析完成，误差: {error_percent:.2f}%")
            
        except Exception as e:
            print(f"✗ 第 {i} 对图像分析失败: {e}")
            results_summary.append({
                'pair_id': i,
                'camera_image': camera_path,
                'simulation_image': simulation_path,
                'error_percentage': None,
                'status': 'ERROR',
                'error_message': str(e)
            })
    
    # 生成批量分析报告
    print(f"\n{'='*60}")
    print("批量分析完成！")
    print(f"{'='*60}")
    
    pass_count = sum(1 for r in results_summary if r['status'] == 'PASS')
    fail_count = sum(1 for r in results_summary if r['status'] == 'FAIL')
    error_count = sum(1 for r in results_summary if r['status'] == 'ERROR')
    
    print(f"\n分析统计:")
    print(f"  总计: {len(results_summary)} 对")
    print(f"  合格: {pass_count} 对")
    print(f"  超差: {fail_count} 对")
    print(f"  错误: {error_count} 对")
    
    # 保存批量分析结果
    import pandas as pd
    df = pd.DataFrame(results_summary)
    summary_path = "./batch_analysis_summary.csv"
    df.to_csv(summary_path, index=False, encoding='utf-8-sig')
    
    print(f"\n批量分析摘要已保存到: {summary_path}")
    
    return results_summary

def calculate_with_physical_calibration(camera_path, simulation_path, 
                                       pixel_to_mm_ratio=0.1, 
                                       focal_length_mm=50,
                                       sensor_width_mm=36):
    """
    考虑物理校准的误差计算
    假设已知相机参数，可以将像素误差转换为物理误差
    
    Args:
        pixel_to_mm_ratio: 像素到毫米的转换比例
        focal_length_mm: 焦距（毫米）
        sensor_width_mm: 传感器宽度（毫米）
    """
    try:
        analyzer = InstallationErrorAnalyzer(camera_path, simulation_path)
        results = analyzer.analyze_with_feature_matching()
        
        pixel_error = results.get('pixel_error', {})
        geometric_error = results.get('geometric_error', {})
        
        # 将像素误差转换为物理误差
        physical_results = {
            'pixel_error_mm': pixel_error.get('mean_error', 0) * pixel_to_mm_ratio,
            'translation_error_mm': geometric_error.get('mean_translation_error_pixels', 0) * pixel_to_mm_ratio,
            'rotation_angle_degrees': geometric_error.get('rotation_angle_degrees', 0),
            'scale_error_percent': (geometric_error.get('scale_factor', 1) - 1) * 100
        }
        
        # 计算视野中的实际位置误差
        fov_horizontal = 2 * np.arctan(sensor_width_mm / (2 * focal_length_mm)) * 180 / np.pi
        pixel_per_degree = analyzer.camera_width / fov_horizontal
        
        physical_results.update({
            'fov_horizontal_degrees': fov_horizontal,
            'angular_error_degrees': geometric_error.get('mean_translation_error_pixels', 0) / pixel_per_degree,
            'pixel_to_mm_ratio': pixel_to_mm_ratio
        })
        
        print(f"\n物理误差分析:")
        print(f"  视野水平角度: {fov_horizontal:.2f}°")
        print(f"  平均像素误差: {pixel_error.get('mean_error', 0):.2f} px ≈ {physical_results['pixel_error_mm']:.3f} mm")
        print(f"  平均位置误差: {physical_results['translation_error_mm']:.3f} mm")
        print(f"  角度误差: {physical_results['angular_error_degrees']:.3f}°")
        print(f"  旋转误差: {physical_results['rotation_angle_degrees']:.2f}°")
        print(f"  缩放误差: {physical_results['scale_error_percent']:.2f}%")
        
        return physical_results
        
    except Exception as e:
        print(f"物理误差计算失败: {e}")
        return None

if __name__ == "__main__":
    # 示例1: 单对图像分析
    print("示例1: 单对图像安装误差分析")
    print("="*60)
    
    main()
    
    # 示例2: 批量分析（取消注释以使用）
    """
    print("\n\n示例2: 批量图像分析")
    print("="*60)
    
    image_pairs = [
        ("./potor/camera_1.jpg", "./potor/simulation_1.jpg", "./analysis_results/pair_1"),
        ("./potor/camera_2.jpg", "./potor/simulation_2.jpg", "./analysis_results/pair_2"),
        ("./potor/camera_3.jpg", "./potor/simulation_3.jpg", "./analysis_results/pair_3"),
    ]
    
    batch_results = batch_analysis(image_pairs)
    """
    
    # 示例3: 物理误差计算（需要校准参数）
    """
    print("\n\n示例3: 物理误差计算")
    print("="*60)
    
    physical_error = calculate_with_physical_calibration(
        camera_path="./potor/img_v3_02sp_7a5ee115-b019-4260-81a5-9fbd05e0e76g.jpg",
        simulation_path="./potor/img_v3_02sp_e51b3634-66a9-4d6b-9b3d-69e53271910g.jpg",
        pixel_to_mm_ratio=0.05,  # 假设每个像素对应0.05mm
        focal_length_mm=50,
        sensor_width_mm=36
    )
    """
