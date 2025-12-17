import cv2
import numpy as np
import os
from datetime import datetime
import json

class SimpleInstallationErrorAnalyzer:
    def __init__(self, camera_path, simulation_path, output_dir="./simple_analysis"):
        """
        简化工装安装误差分析器
        
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
        
        # 统一图像尺寸（将仿真图缩放到相机图大小）
        self._resize_images()
        
        # 存储结果
        self.results = {
            "analysis_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "camera_image": camera_path,
            "simulation_image": simulation_path,
            "image_dimensions": {
                "original_camera": self.camera_img.shape[:2][::-1],  # (width, height)
                "original_simulation": self.simulation_img.shape[:2][::-1],
                "resized": self.camera_img.shape[:2][::-1]
            }
        }
    
    def _resize_images(self):
        """将仿真图像缩放到相机图像尺寸"""
        h_cam, w_cam = self.camera_img.shape[:2]
        h_sim, w_sim = self.simulation_img.shape[:2]
        
        # 保持宽高比缩放
        scale = min(w_cam / w_sim, h_cam / h_sim)
        new_width = int(w_sim * scale)
        new_height = int(h_sim * scale)
        
        self.simulation_img = cv2.resize(self.simulation_img, (new_width, new_height))
        
        # 如果尺寸不完全匹配，填充到相同尺寸
        if new_width != w_cam or new_height != h_cam:
            canvas = np.zeros((h_cam, w_cam, 3), dtype=np.uint8)
            start_x = (w_cam - new_width) // 2
            start_y = (h_cam - new_height) // 2
            canvas[start_y:start_y+new_height, start_x:start_x+new_width] = self.simulation_img
            self.simulation_img = canvas
    
    def analyze_angle_error(self, use_corners=True):
        """
        分析角度误差（主要关注旋转偏差）
        
        Args:
            use_corners: 是否使用角点检测（True）或简单投影（False）
        """
        print("开始角度误差分析...")
        
        # 方法1：使用角点检测（更准确）
        if use_corners:
            return self._analyze_with_corners()
        # 方法2：简单投影映射
        else:
            return self._analyze_simple_projection()
    
    def _analyze_with_corners(self):
        """使用角点检测分析角度误差"""
        # 转换为灰度图
        gray_camera = cv2.cvtColor(self.camera_img, cv2.COLOR_BGR2GRAY)
        gray_simulation = cv2.cvtColor(self.simulation_img, cv2.COLOR_BGR2GRAY)
        
        # 检测角点
        corners_camera = cv2.goodFeaturesToTrack(gray_camera, maxCorners=100, 
                                                qualityLevel=0.01, minDistance=10)
        corners_simulation = cv2.goodFeaturesToTrack(gray_simulation, maxCorners=100, 
                                                    qualityLevel=0.01, minDistance=10)
        
        if corners_camera is None or corners_simulation is None:
            print("警告：无法检测到足够角点，使用简单投影方法")
            return self._analyze_simple_projection()
        
        # 计算角点的主方向（使用PCA）
        angle_camera = self._calculate_principal_angle(corners_camera)
        angle_simulation = self._calculate_principal_angle(corners_simulation)
        
        # 计算角度差
        angle_diff = abs(angle_camera - angle_simulation)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # 生成投影图像
        projected_img = self._generate_projection(angle_diff)
        
        # 保存结果
        self._save_results(angle_camera, angle_simulation, angle_diff, projected_img)
        
        return self.results
    
    def _calculate_principal_angle(self, corners):
        """使用PCA计算角点的主方向角度"""
        corners_flat = corners.reshape(-1, 2).astype(np.float32)
        
        # 计算均值
        mean = np.mean(corners_flat, axis=0)
        
        # 计算协方差矩阵
        cov_mat = np.cov(corners_flat.T)
        
        # 计算特征值和特征向量
        eigenvalues, eigenvectors = np.linalg.eig(cov_mat)
        
        # 获取最大特征值对应的特征向量
        max_eigenvalue_idx = np.argmax(eigenvalues)
        principal_vector = eigenvectors[:, max_eigenvalue_idx]
        
        # 计算角度（0-360度）
        angle = np.arctan2(principal_vector[1], principal_vector[0]) * 180 / np.pi
        if angle < 0:
            angle += 360
        
        return angle
    
    def _analyze_simple_projection(self):
        """简单投影映射分析"""
        print("使用简单投影映射分析...")
        
        # 假设工装是矩形，检测边缘
        gray_camera = cv2.cvtColor(self.camera_img, cv2.COLOR_BGR2GRAY)
        gray_simulation = cv2.cvtColor(self.simulation_img, cv2.COLOR_BGR2GRAY)
        
        # 边缘检测
        edges_camera = cv2.Canny(gray_camera, 50, 150)
        edges_simulation = cv2.Canny(gray_simulation, 50, 150)
        
        # 检测直线
        lines_camera = cv2.HoughLinesP(edges_camera, 1, np.pi/180, threshold=50, 
                                      minLineLength=30, maxLineGap=10)
        lines_simulation = cv2.HoughLinesP(edges_simulation, 1, np.pi/180, threshold=50, 
                                          minLineLength=30, maxLineGap=10)
        
        # 计算平均角度
        angle_camera = self._calculate_average_line_angle(lines_camera) if lines_camera is not None else 0
        angle_simulation = self._calculate_average_line_angle(lines_simulation) if lines_simulation is not None else 0
        
        # 计算角度差
        angle_diff = abs(angle_camera - angle_simulation)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # 生成投影图像
        projected_img = self._generate_projection(angle_diff)
        
        # 保存结果
        self._save_results(angle_camera, angle_simulation, angle_diff, projected_img)
        
        return self.results
    
    def _calculate_average_line_angle(self, lines):
        """计算直线的平均角度"""
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if angle < 0:
                angle += 180
            angles.append(angle)
        
        return np.mean(angles) if angles else 0
    
    def _generate_projection(self, angle_diff):
        """生成投影图像以可视化角度偏差"""
        h, w = self.camera_img.shape[:2]
        
        # 创建合成图像
        composite = np.zeros((h, w*2, 3), dtype=np.uint8)
        composite[:, :w] = self.camera_img
        composite[:, w:] = self.simulation_img
        
        # 在图像上绘制角度信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(composite, f"Camera", (10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(composite, f"Simulation", (w+10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(composite, f"Angle Difference: {angle_diff:.2f} angle", 
                   (w//2 - 100, h-20), font, 0.7, (0, 0, 255), 2)
        
        # 绘制角度指示线
        center_x, center_y = w//2, h//2
        length = min(w, h) // 3
        
        # 相机图像的角度指示
        cv2.line(composite, (center_x, center_y), 
                (center_x + int(length * np.cos(0)), center_y + int(length * np.sin(0))),
                (255, 0, 0), 3)
        
        # 仿真图像的角度指示（偏移角度）
        cv2.line(composite, (w + center_x, center_y), 
                (w + center_x + int(length * np.cos(np.radians(angle_diff))), 
                 center_y + int(length * np.sin(np.radians(angle_diff)))),
                (0, 0, 255), 3)
        
        return composite
    
    def _save_results(self, angle_camera, angle_simulation, angle_diff, projected_img):
        """保存分析结果"""
        # 保存投影图像
        projected_path = os.path.join(self.output_dir, "angle_projection.jpg")
        cv2.imwrite(projected_path, projected_img)
        
        # 保存角度差异图
        diff_path = os.path.join(self.output_dir, "angle_difference.jpg")
        self._save_angle_difference_plot(angle_camera, angle_simulation, angle_diff, diff_path)
        
        # 更新结果
        self.results.update({
            "method": "angle_analysis",
            "angles": {
                "camera_angle": float(angle_camera),
                "simulation_angle": float(angle_simulation),
                "angle_difference": float(angle_diff)
            },
            "projected_image": projected_path,
            "difference_plot": diff_path,
            "assessment": self._assess_installation(angle_diff)
        })
    
    def _save_angle_difference_plot(self, angle_cam, angle_sim, angle_diff, save_path):
        """保存角度差异图 - 修复版本"""
        try:
            import matplotlib.pyplot as plt
            
            # 方法1：使用极坐标图
            fig = plt.figure(figsize=(8, 6))
            ax = fig.add_subplot(111, projection='polar')  # 创建极坐标轴
            
            # 绘制角度线
            angles_rad = [np.radians(angle_cam), np.radians(angle_sim)]
            labels = [f'Camera: {angle_cam:.1f}°', f'Simulation: {angle_sim:.1f}°']
            colors = ['blue', 'red']
            
            for i, (angle_rad, label, color) in enumerate(zip(angles_rad, labels, colors)):
                ax.plot([angle_rad, angle_rad], [0, 1], 
                       color=color, linewidth=3, label=label)
            
            # 设置极坐标参数
            ax.set_theta_zero_location('E')  # 0度在右侧
            ax.set_theta_direction(1)  # 逆时针方向
            ax.set_ylim(0, 1.2)
            ax.set_title(f'Angle Difference: {angle_diff:.2f}°', fontsize=14, pad=20)
            ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
            
            plt.tight_layout()
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close()
            
        except Exception as e:
            print(f"保存角度差异图失败: {e}")
            # 如果极坐标图失败，使用简单的折线图
            self._save_simple_angle_plot(angle_cam, angle_sim, angle_diff, save_path)
    
    def _save_simple_angle_plot(self, angle_cam, angle_sim, angle_diff, save_path):
        """保存简单的角度差异图（不使用极坐标）"""
        import matplotlib.pyplot as plt
        
        fig, ax = plt.subplots(figsize=(8, 6))
        
        # 创建简单的条形图
        categories = ['Camera', 'Simulation']
        angles = [angle_cam, angle_sim]
        colors = ['blue', 'red']
        
        bars = ax.bar(categories, angles, color=colors, alpha=0.7)
        
        # 添加数值标签
        for bar, angle in zip(bars, angles):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                   f'{angle:.1f}°', ha='center', va='bottom', fontsize=12)
        
        # 设置图表属性
        ax.set_ylabel('Angle (degrees)', fontsize=12)
        ax.set_title(f'Angle Comparison\nDifference: {angle_diff:.2f}°', fontsize=14)
        ax.set_ylim(0, max(angles) + 20)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        plt.close()
    
    def _assess_installation(self, angle_diff):
        """评估安装质量"""
        tolerance = 2.0  # 2度容忍度
        
        if angle_diff <= tolerance:
            return {
                "status": "PASS",
                "message": f"安装角度偏差({angle_diff:.2f}°)在允许范围内(≤{tolerance}°)",
                "recommendation": "安装合格，无需调整"
            }
        else:
            return {
                "status": "FAIL",
                "message": f"安装角度偏差({angle_diff:.2f}°)超过允许范围(>{tolerance}°)",
                "recommendation": f"需要调整工装，旋转约{angle_diff:.2f}°"
            }
    
    def generate_report(self):
        """生成简单报告"""
        report_path = os.path.join(self.output_dir, "angle_analysis_report.json")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        
        # 生成文本摘要
        summary = self._generate_summary()
        summary_path = os.path.join(self.output_dir, "summary.txt")
        
        with open(summary_path, 'w', encoding='utf-8') as f:
            f.write(summary)
        
        print(f"报告已保存到: {report_path}")
        print(f"摘要已保存到: {summary_path}")
        
        return report_path, summary_path
    
    def _generate_summary(self):
        """生成文本摘要"""
        angles = self.results.get("angles", {})
        assessment = self.results.get("assessment", {})
        
        summary = "=" * 50 + "\n"
        summary += "工装安装角度误差分析报告\n"
        summary += "=" * 50 + "\n\n"
        
        summary += f"分析时间: {self.results.get('analysis_time')}\n"
        summary += f"相机图像: {os.path.basename(self.camera_path)}\n"
        summary += f"仿真图像: {os.path.basename(self.simulation_path)}\n\n"
        
        summary += "角度测量结果:\n"
        summary += f"  相机图像角度: {angles.get('camera_angle', 0):.2f}°\n"
        summary += f"  仿真图像角度: {angles.get('simulation_angle', 0):.2f}°\n"
        summary += f"  角度偏差: {angles.get('angle_difference', 0):.2f}°\n\n"
        
        summary += "安装评估:\n"
        summary += f"  状态: {assessment.get('status', 'UNKNOWN')}\n"
        summary += f"  结论: {assessment.get('message', '')}\n"
        summary += f"  建议: {assessment.get('recommendation', '')}\n\n"
        
        summary += "输出文件:\n"
        summary += f"  投影图像: {os.path.basename(self.results.get('projected_image', ''))}\n"
        summary += f"  角度差异图: {os.path.basename(self.results.get('difference_plot', ''))}\n"
        summary += f"  详细报告: angle_analysis_report.json\n"
        
        summary += "\n" + "=" * 50
        
        return summary

# 使用示例
def main():
    # 图像路径
    camera_path = "./potor/img_v3_02sp_7a5ee115-b019-4260-81a5-9fbd05e0e76g.jpg"  # 替换为实际路径
    simulation_path = "./potor/img_v3_02sp_e51b3634-66a9-4d6b-9b3d-69e53271910g.jpg"  # 替换为实际路径
    
    try:
        print("开始工装安装角度误差分析...")
        print("=" * 50)
        
        # 创建分析器
        analyzer = SimpleInstallationErrorAnalyzer(
            camera_path=camera_path,
            simulation_path=simulation_path,
            output_dir="./angle_analysis_results"
        )
        
        # 执行角度分析
        results = analyzer.analyze_angle_error(use_corners=True)
        
        # 生成报告
        analyzer.generate_report()
        
        # 打印关键结果
        angles = results.get("angles", {})

        assessment = results.get("assessment", {})

        print(f"\n分析完成！")
        print(f"角度偏差: {angles.get('angle_difference', 0):.2f}°")
        print(f"评估结果: {assessment.get('status', 'UNKNOWN')}")
        print(f"建议: {assessment.get('recommendation', '')}")
        
        print(f"\n可视化结果保存在: ./angle_analysis_results/")
        
    except Exception as e:
        print(f"分析失败: {e}")

# 批量分析函数
def batch_angle_analysis(image_pairs):
    """批量分析多个图像对的角度误差"""
    results = []
    
    for i, (camera_path, sim_path) in enumerate(image_pairs, 1):
        print(f"\n分析第 {i}/{len(image_pairs)} 对图像...")
        
        try:
            output_dir = f"./batch_results/pair_{i}"
            analyzer = SimpleInstallationErrorAnalyzer(
                camera_path=camera_path,
                simulation_path=sim_path,
                output_dir=output_dir
            )
            
            analyzer.analyze_angle_error(use_corners=True)
            analyzer.generate_report()
            
            angles = analyzer.results.get("angles", {})
            assessment = analyzer.results.get("assessment", {})
            
            results.append({
                "pair": i,
                "camera": os.path.basename(camera_path),
                "simulation": os.path.basename(sim_path),
                "angle_diff": angles.get("angle_difference", 0),
                "status": assessment.get("status", "ERROR"),
                "output_dir": output_dir
            })
            
            print(f"✓ 完成，角度偏差: {angles.get('angle_difference', 0):.2f}°")
            
        except Exception as e:
            print(f"✗ 失败: {e}")
            results.append({
                "pair": i,
                "camera": os.path.basename(camera_path),
                "simulation": os.path.basename(sim_path),
                "angle_diff": None,
                "status": "ERROR",
                "error": str(e)
            })
    
    # 生成批量报告
    print(f"\n{'='*50}")
    print("批量分析完成！")
    print(f"{'='*50}")
    
    pass_count = sum(1 for r in results if r.get("status") == "PASS")
    fail_count = sum(1 for r in results if r.get("status") == "FAIL")
    error_count = sum(1 for r in results if r.get("status") == "ERROR")
    
    print(f"\n统计结果:")
    print(f"  总计: {len(results)} 对")
    print(f"  合格: {pass_count} 对")
    print(f"  超差: {fail_count} 对")
    print(f"  错误: {error_count} 对")
    
    # 保存批量结果
    import csv
    csv_path = "./batch_angle_results.csv"
    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    
    print(f"\n批量结果已保存到: {csv_path}")
    
    return results

# 快速分析函数（最简版本）
def quick_angle_check(camera_path, simulation_path, show_result=True):
    """
    快速角度检查 - 最简版本
    只返回角度偏差，不保存文件
    """
    try:
        # 读取图像
        camera_img = cv2.imread(camera_path)
        sim_img = cv2.imread(simulation_path)
        
        if camera_img is None or sim_img is None:
            print("错误：无法读取图像")
            return None
        
        # 统一尺寸
        h_cam, w_cam = camera_img.shape[:2]
        sim_img = cv2.resize(sim_img, (w_cam, h_cam))
        
        # 转换为灰度
        gray_cam = cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY)
        gray_sim = cv2.cvtColor(sim_img, cv2.COLOR_BGR2GRAY)
        
        # 简单边缘检测
        edges_cam = cv2.Canny(gray_cam, 50, 150)
        edges_sim = cv2.Canny(gray_sim, 50, 150)
        
        # 检测主要直线
        def get_main_angle(edges):
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
            if lines is None:
                return 0
            
            # 找到最长的直线
            longest_length = 0
            main_angle = 0
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                if length > longest_length:
                    longest_length = length
                    angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
                    if angle < 0:
                        angle += 180
                    main_angle = angle
            
            return main_angle
        
        angle_cam = get_main_angle(edges_cam)
        angle_sim = get_main_angle(edges_sim)
        
        # 计算角度差
        angle_diff = abs(angle_cam - angle_sim)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # 判断是否合格
        tolerance = 2.0
        is_pass = angle_diff <= tolerance
        
        if show_result:
            print(f"\n快速角度检查结果:")
            print(f"  相机角度: {angle_cam:.2f}°")
            print(f"  仿真角度: {angle_sim:.2f}°")
            print(f"  角度偏差: {angle_diff:.2f}°")
            print(f"  允许偏差: {tolerance}°")
            print(f"  结果: {'✓ 合格' if is_pass else '✗ 超差'}")
        
        return {
            "camera_angle": angle_cam,
            "simulation_angle": angle_sim,
            "angle_difference": angle_diff,
            "is_pass": is_pass,
            "tolerance": tolerance
        }
        
    except Exception as e:
        print(f"快速检查失败: {e}")
        return None

def visualize_projection(camera_path, simulation_path, angle_diff=None, show_window=False):

    camera_img = cv2.imread(camera_path)
    sim_img = cv2.imread(simulation_path)
    
    if camera_img is None or sim_img is None:
        print("错误：无法读取图像")
        return None
    
    # 统一尺寸
    h, w = camera_img.shape[:2]
    sim_img = cv2.resize(sim_img, (w, h))
    
    # 创建合成图像
    composite = np.zeros((h, w*2, 3), dtype=np.uint8)
    composite[:, :w] = camera_img
    composite[:, w:] = sim_img
    
    # 添加文字
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(composite, "Camera", (10, 30), font, 1, (0, 255, 0), 2)
    cv2.putText(composite, "Simulation", (w+10, 30), font, 1, (0, 255, 0), 2)
    
    if angle_diff is not None:
        cv2.putText(composite, f"Angle Diff: {angle_diff:.2f}°", 
                   (w//2 - 100, h-20), font, 0.8, (0, 0, 255), 2)
    
    # 保存文件
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = f"./projection_{timestamp}.jpg"
    cv2.imwrite(save_path, composite)
    print(f"✓ 投影图像已保存: {save_path}")
    
    # 可选：显示窗口（在无GUI环境会跳过）
    if show_window:
        try:
            cv2.imshow("Projection Comparison", composite)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"⚠ 无法显示窗口（无GUI环境），但图像已保存")
    
    return composite


# 可视化投影函数
# def visualize_projection(camera_path, simulation_path, angle_diff=None):
#     """
#     可视化投影效果
#     """
#     camera_img = cv2.imread(camera_path)
#     sim_img = cv2.imread(simulation_path)
    
#     if camera_img is None or sim_img is None:
#         return None
    
#     # 统一尺寸
#     h, w = camera_img.shape[:2]
#     sim_img = cv2.resize(sim_img, (w, h))
    
#     # 创建合成图像
#     composite = np.zeros((h, w*2, 3), dtype=np.uint8)
#     composite[:, :w] = camera_img
#     composite[:, w:] = sim_img
    
#     # 添加文字
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     cv2.putText(composite, "Camera", (10, 30), font, 1, (0, 255, 0), 2)
#     cv2.putText(composite, "Simulation", (w+10, 30), font, 1, (0, 255, 0), 2)
    
#     if angle_diff is not None:
#         cv2.putText(composite, f"Angle Diff: {angle_diff:.2f}°", 
#                    (w//2 - 100, h-20), font, 0.8, (0, 0, 255), 2)
    
#     # 显示图像
#     cv2.imshow("Projection Comparison", composite)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     return composite

def analyze_angle_with_direction(camera_path, simulation_path):
    """
    改进版：能判断偏左还是偏右
    """
    import cv2
    import numpy as np
    
    # 读取图像
    camera_img = cv2.imread(camera_path)
    sim_img = cv2.imread(simulation_path)
    
    def get_main_angle(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # 检测直线
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
        
        if lines is None:
            return 0
        
        # 计算平均角度
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
            # 标准化到 0-180 度
            if angle < 0:
                angle += 180
            angles.append(angle)
        
        return np.mean(angles) if angles else 0

    angle_cam = get_main_angle(camera_img)
    angle_sim = get_main_angle(sim_img)
    
    # 关键改进：计算有符号的角度差
    signed_diff = angle_cam - angle_sim
    
    # 方法2：考虑360°循环的更准确计算
    def signed_angle_diff(angle1, angle2):
        diff = angle1 - angle2
        # 调整到 [-180, 180] 范围内
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff
    
    signed_diff = signed_angle_diff(angle_cam, angle_sim)
    
    # 判断偏差方向
    if signed_diff > 0:
        direction = "逆时针偏转（偏左）"
    elif signed_diff < 0:
        direction = "顺时针偏转（偏右）"
    else:
        direction = "无偏差"
    
    # 创建可视化
    h, w = camera_img.shape[:2]
    composite = np.zeros((h, w*2, 3), dtype=np.uint8)
    composite[:, :w] = camera_img
    composite[:, w:] = sim_img
    
    # 添加文字说明
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(composite, f"Camera: {angle_cam:.2f}°", (10, 30), font, 0.8, (0, 255, 0), 2)
    cv2.putText(composite, f"Sim: {angle_sim:.2f}°", (w+10, 30), font, 0.8, (0, 255, 0), 2)
    cv2.putText(composite, f"Diff: {signed_diff:+.2f}°", (w//2-50, h-50), font, 1, (0, 0, 255), 2)
    cv2.putText(composite, direction, (w//2-100, h-20), font, 0.7, (255, 0, 0), 2)
    
    cv2.imwrite("angle_with_direction.jpg", composite)
    
    return {
        "camera_angle": angle_cam,
        "simulation_angle": angle_sim,
        "signed_difference": signed_diff,  # 有正负号！
        "absolute_difference": abs(signed_diff),
        "direction": direction,
        "visualization": "angle_with_direction.jpg"
    }


# 主程序
if __name__ == "__main__":
    print("工装安装角度误差分析工具")
    print("=" * 50)
    
    # 示例1: 快速检查
    print("\n1. 快速角度检查示例:")
    camera_img = "./potor/  20251203-153210.jpg"  # 替换为实际路径
    sim_img = "./potor/img_v3_02sp_e51b3634-66a9-4d6b-9b3d-69e53271910g.jpg"  # 替换为实际路径
    
    result = quick_angle_check(camera_img, sim_img, show_result=True)
    
    if result:
        # 示例2: 详细分析
        print("\n2. 详细角度分析示例:")
        analyzer = SimpleInstallationErrorAnalyzer(
            camera_path=camera_img,
            simulation_path=sim_img,
            output_dir="./detailed_analysis"
        )
        
        analyzer.analyze_angle_error(use_corners=True)
        analyzer.generate_report()
        
        # 示例3: 可视化投影
        # 修改主程序的调用方式
        print("\n3. 生成投影图像:")
        # 方法1：直接调用，不显示窗口
        composite = visualize_projection(camera_img, sim_img, 
                                        result["angle_difference"])

        # 然后立即保存（因为原函数会显示窗口导致崩溃）
        if composite is not None:
            save_path = "./projection_no_window.jpg"
            cv2.imwrite(save_path, composite)
            print(f"投影图像已保存到: {save_path}")
    
    # 示例4: 批量分析（取消注释以使用）
    """
    print("\n4. 批量分析示例:")
    image_pairs = [
        ("./potor/camera_1.jpg", "./potor/sim_1.jpg"),
        ("./potor/camera_2.jpg", "./potor/sim_2.jpg"),
        ("./potor/camera_3.jpg", "./potor/sim_3.jpg"),
    ]
    
    batch_results = batch_angle_analysis(image_pairs)
    """
    
    print("\n" + "=" * 50)
    print("分析完成！")

# 使用说明
"""
使用方法：

1. 快速检查（只返回角度）:
   result = quick_angle_check("camera.jpg", "simulation.jpg")

2. 详细分析（保存报告和图像）:
   analyzer = SimpleInstallationErrorAnalyzer("camera.jpg", "simulation.jpg")
   analyzer.analyze_angle_error()
   analyzer.generate_report()

3. 批量分析:
   batch_angle_analysis([("cam1.jpg", "sim1.jpg"), ("cam2.jpg", "sim2.jpg")])

4. 可视化投影:
   visualize_projection("camera.jpg", "simulation.jpg", angle_diff=1.5)

主要特点：
- 只关注角度偏差，不进行复杂的特征匹配
- 自动统一图像尺寸
- 提供快速检查和详细分析两种模式
- 生成可视化投影图像
- 判断安装是否合格（默认2度容忍度）
"""

# analyze_angle_with_direction(camera_path, simulation_path)