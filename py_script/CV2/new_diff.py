import cv2
import numpy as np

class SensorInstallationAngleDetector:
    def __init__(self):
        # 针对传感器安装面的颜色范围（通常是金属或深色）
        self.installation_surface_color = {
            'lower': np.array([0, 0, 0]),      # 黑色/深色安装面
            'upper': np.array([180, 100, 100])  # 调整这个值
        }
        
        # 安装面通常是直线边缘，所以重点检测直线
        self.min_line_length = 50  # 最小直线长度
        self.max_line_gap = 10     # 最大线段间隙
        
    def detect_installation_angle(self, image_path, expected_angle=90, debug=False):
        """
        专门检测传感器安装角度
        """
        # 1. 读取图像
        img = cv2.imread(image_path)
        if img is None:
            print("无法读取图像")
            return None
        
        if debug:
            cv2.imwrite("0_original.jpg", img)
        
        # 2. 提取安装面区域（ROI）
        roi_img, roi_coords = self._extract_installation_roi(img, debug)
        
        # 3. 增强安装面边缘
        enhanced_edges = self._enhance_installation_edges(roi_img, debug)
        
        # 4. 检测安装面直线
        installation_lines = self._detect_installation_lines(enhanced_edges, debug)
        
        # 5. 计算安装角度
        angle = self._calculate_installation_angle(installation_lines, expected_angle)
        
        # 6. 可视化结果
        if angle is not None and debug:
            self._visualize_installation_angle(img, roi_coords, installation_lines, angle)
        
        return angle
    
    def _extract_installation_roi(self, img, debug=False):
        """
        提取传感器安装面区域（ROI）
        安装面通常在传感器底部或侧面
        """
        height, width = img.shape[:2]
        
        # 方法1：手动指定ROI（根据你的图像调整）
        # 安装面通常在图像下半部分
        roi_y_start = int(height * 0.6)  # 从60%高度开始
        roi_y_end = height - 20          # 到底部留20像素
        roi_x_start = int(width * 0.3)   # 从30%宽度开始
        roi_x_end = int(width * 0.7)     # 到70%宽度结束
        
        # 方法2：自动检测安装面区域
        # 安装面通常是深色区域
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到最大的深色区域
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # 调整ROI为区域底部（安装面通常在底部）
            roi_y_start = y + h - 50  # 取底部50像素
            roi_y_end = y + h
            roi_x_start = x
            roi_x_end = x + w
        
        roi = img[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        roi_coords = (roi_x_start, roi_y_start, roi_x_end, roi_y_end)
        
        if debug:
            cv2.imwrite("1_roi_extracted.jpg", roi)
            print(f"ROI坐标: {roi_coords}")
        
        return roi, roi_coords
    
    def _enhance_installation_edges(self, roi_img, debug=False):
        """
        增强安装面边缘
        """
        # 转换为灰度
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        
        # 方法1：使用Sobel算子增强水平/垂直边缘
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        
        # 计算梯度幅值
        gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
        gradient_magnitude = np.uint8(np.clip(gradient_magnitude, 0, 255))
        
        # 方法2：使用Canny边缘检测
        edges = cv2.Canny(gray, 50, 150)
        
        # 方法3：结合两种方法
        combined = cv2.bitwise_or(edges, gradient_magnitude)
        
        # 形态学操作增强直线
        kernel_horizontal = np.ones((1, 15), np.uint8)  # 水平核
        kernel_vertical = np.ones((15, 1), np.uint8)    # 垂直核
        
        enhanced_horizontal = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel_horizontal)
        enhanced_vertical = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel_vertical)
        
        final_enhanced = cv2.bitwise_or(enhanced_horizontal, enhanced_vertical)
        
        if debug:
            cv2.imwrite("2_gray.jpg", gray)
            cv2.imwrite("3_edges.jpg", edges)
            cv2.imwrite("4_gradient.jpg", gradient_magnitude)
            cv2.imwrite("5_enhanced.jpg", final_enhanced)
        
        return final_enhanced
    
    def _detect_installation_lines(self, edges, debug=False):
        """
        检测安装面直线
        """
        # 使用概率霍夫变换检测长直线
        lines = cv2.HoughLinesP(edges, 
                               rho=1, 
                               theta=np.pi/180, 
                               threshold=50,
                               minLineLength=self.min_line_length,
                               maxLineGap=self.max_line_gap)
        
        if lines is None:
            return []
        
        # 过滤和排序直线
        filtered_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 计算直线长度
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # 计算角度
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            
            # 只保留接近水平或垂直的直线（安装面通常是水平或垂直的）
            if abs(angle) < 30 or abs(angle - 90) < 30 or abs(angle + 90) < 30:
                filtered_lines.append({
                    'points': (x1, y1, x2, y2),
                    'length': length,
                    'angle': angle
                })
        
        # 按长度排序
        filtered_lines.sort(key=lambda x: x['length'], reverse=True)
        
        if debug:
            print(f"检测到 {len(filtered_lines)} 条安装面直线")
            for i, line in enumerate(filtered_lines[:5]):  # 显示前5条最长的
                print(f"  直线{i+1}: 角度={line['angle']:.1f}°, 长度={line['length']:.1f}")
        
        return filtered_lines
    
    def _calculate_installation_angle(self, lines, expected_angle):
        """
        计算安装角度
        """
        if not lines:
            return None
        
        # 方法1：使用最长的直线
        longest_line = lines[0]
        angle = longest_line['angle']
        
        # 调整角度到0-90度范围
        if angle < 0:
            angle = 180 + angle
        
        if angle > 90:
            angle = angle - 90
        
        # 方法2：使用所有直线的加权平均（长度作为权重）
        angles = []
        weights = []
        
        for line in lines[:5]:  # 使用前5条最长的直线
            raw_angle = line['angle']
            
            # 调整角度
            if raw_angle < 0:
                raw_angle = 180 + raw_angle
            
            if raw_angle > 90:
                raw_angle = raw_angle - 90
            
            angles.append(raw_angle)
            weights.append(line['length'])
        
        # 计算加权平均
        if len(angles) > 1:
            weighted_angle = np.average(angles, weights=weights)
            # 使用加权平均和最长直线的平均值
            angle = (angle + weighted_angle) / 2
        
        return angle
    
    def _visualize_installation_angle(self, img, roi_coords, lines, angle):
        """
        可视化安装角度检测结果
        """
        result_img = img.copy()
        roi_x1, roi_y1, roi_x2, roi_y2 = roi_coords
        
        # 绘制ROI区域
        cv2.rectangle(result_img, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)
        
        # 绘制检测到的直线
        for i, line in enumerate(lines[:3]):  # 绘制前3条最长的直线
            x1, y1, x2, y2 = line['points']
            # 将ROI坐标转换为图像坐标
            x1_img, y1_img = x1 + roi_x1, y1 + roi_y1
            x2_img, y2_img = x2 + roi_x1, y2 + roi_y1
            
            color = (0, 255, 0) if i == 0 else (0, 255, 255)  # 第一条绿色，其他黄色
            cv2.line(result_img, (x1_img, y1_img), (x2_img, y2_img), color, 2)
        
        # 绘制角度文本
        height, width = img.shape[:2]
        text = f"install angle is : {angle:.1f} *"
        cv2.putText(result_img, text, (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 绘制角度参考线
        center_x, center_y = width // 2, height // 2
        length = min(width, height) // 4
        
        # 检测到的角度线（绿色）
        angle_rad = np.radians(angle)
        end_x = int(center_x + length * np.cos(angle_rad))
        end_y = int(center_y - length * np.sin(angle_rad))  # 注意：图像y轴向下
        cv2.line(result_img, (center_x, center_y), (end_x, end_y), (0, 255, 0), 3)
        
        # 期望角度线（蓝色）
        expected_rad = np.radians(90)
        end_x_exp = int(center_x + length * np.cos(expected_rad))
        end_y_exp = int(center_y - length * np.sin(expected_rad))
        cv2.line(result_img, (center_x, center_y), (end_x_exp, end_y_exp), (255, 0, 0), 2)
        
        # 角度差
        angle_diff = abs(angle - 90)
        diff_text = f"diff angle {angle_diff:.1f} *"
        cv2.putText(result_img, diff_text, (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imwrite("installation_angle_result.jpg", result_img)
        print(f"结果已保存为 installation_angle_result.jpg")

# 使用示例
detector = SensorInstallationAngleDetector()

# 检测安装角度
angle = detector.detect_installation_angle(
    image_path='/root/work/py_script/CV2/potor/20251201-141230.jpg',
    expected_angle=90,
    debug=True  # 开启调试模式
)

if angle is not None:
    print(f"\n=== 检测结果 ===")
    print(f"传感器安装角度: {angle:.2f}°")
    
    # 判断是否合格
    expected = 90
    tolerance = 1
    
    deviation = abs(angle - expected)
    if deviation > tolerance:
        print(f"❌ 不合格！偏差: {deviation:.2f}° (允许: ±{tolerance}°)")
    else:
        print(f"✅ 合格！偏差: {deviation:.2f}°")
else:
    print("无法检测到安装角度")
