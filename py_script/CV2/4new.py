import cv2
import numpy as np
import math

class AutoSensorAngleChecker:
    def __init__(self, tolerance=1.0):
        """
        tolerance: 允许的角度公差（度）
        """
        self.tolerance = tolerance
        self.reference_angles = None  # 参考角度 (yaw, pitch, roll)
        
    def set_reference_angles(self, yaw=0.0, pitch=0.0, roll=0.0):
        """设置参考角度（理想安装角度）"""
        self.reference_angles = {
            'yaw': yaw,
            'pitch': pitch,
            'roll': roll
        }
        print(f"✅ 参考角度已设置: Yaw={yaw}°, Pitch={pitch}°, Roll={roll}°")
    
    def auto_angle_measurement(self, image_path):
        """
        自动测量传感器三角度
        只需框选传感器区域，程序自动计算所有角度
        """
        img = cv2.imread(image_path)
        if img is None:
            print("无法读取图像")
            return None
        
        print("\n" + "="*50)
        print("传感器三角度自动测量")
        print("="*50)
        
        # 1. 框选传感器区域
        print("\n步骤1: 框选传感器整体区域")
        print("用鼠标拖拽选择矩形区域，按空格确认")
        roi = cv2.selectROI("Select Sensor Area", img, False)
        cv2.destroyAllWindows()
        
        if roi[2] == 0 or roi[3] == 0:
            print("未选择区域")
            return None
        
        x, y, w, h = roi
        print(f"传感器区域: ({x}, {y}) {w}x{h}")
        
        # 2. 从ROI自动提取关键点并计算角度
        print("\n步骤2: 自动计算角度...")
        angles, points = self._calculate_angles_from_roi(roi)
        
        if angles is None:
            print("角度计算失败")
            return None
        
        yaw_angle, pitch_angle, roll_angle = angles
        yaw_points, pitch_points, roll_points = points
        
        print(f"Yaw角: {yaw_angle:.2f}°")
        print(f"Pitch角: {pitch_angle:.2f}°")
        print(f"Roll角: {roll_angle:.2f}°")
        
        # 3. 计算与参考角度的偏差
        deviations = {}
        if self.reference_angles:
            deviations = {
                'yaw': yaw_angle - self.reference_angles['yaw'],
                'pitch': pitch_angle - self.reference_angles['pitch'],
                'roll': roll_angle - self.reference_angles['roll']
            }
        
        # 4. 检查是否超差
        check_result = self._check_tolerance(deviations)
        
        # 5. 可视化结果
        result_img = self._visualize_results(
            img, roi, yaw_points, pitch_points, roll_points,
            yaw_angle, pitch_angle, roll_angle,
            deviations, check_result
        )
        
        # 6. 保存结果
        self._save_result(result_img)
        
        return {
            'angles': {
                'yaw': yaw_angle,
                'pitch': pitch_angle,
                'roll': roll_angle
            },
            'deviations': deviations,
            'check_result': check_result,
            'roi': roi,
            'points': points
        }
    
    def _calculate_angles_from_roi(self, roi):
        """
        从ROI自动计算三个角度
        基于传感器是矩形且正对相机的假设
        """
        x, y, w, h = roi
        
        # 计算ROI的四个角点
        corners = {
            'top_left': (x, y),
            'top_right': (x + w, y),
            'bottom_left': (x, y + h),
            'bottom_right': (x + w, y + h)
        }
        
        # 1. 计算Roll角（传感器自身旋转）
        # 使用对角线：左上到右下
        roll_points = (corners['top_left'], corners['bottom_right'])
        roll_angle_raw = self._calculate_angle(*roll_points)
        
        # 修正：理想对角线角度是45°，所以Roll = 测量值 - 45°
        roll_angle = roll_angle_raw - 45.0
        
        # 规范化到[-180, 180]
        if roll_angle > 180:
            roll_angle -= 360
        elif roll_angle < -180:
            roll_angle += 360
        
        # 2. 计算Yaw角（水平方向）
        # 使用上边缘：左上到右上
        yaw_points = (corners['top_left'], corners['top_right'])
        yaw_angle_raw = self._calculate_angle(*yaw_points)
        
        # 修正：理想上边缘角度是0°，所以Yaw = 测量值
        # 但根据你的描述，可能需要180°修正
        yaw_angle = yaw_angle_raw
        
        # 如果传感器是180°安装，可能需要调整
        # yaw_angle = 180.0 - yaw_angle_raw  # 如果需要180°修正
        
        # 规范化到[0, 180]
        if yaw_angle < 0:
            yaw_angle += 180
        elif yaw_angle > 180:
            yaw_angle -= 180
        
        # 3. 计算Pitch角（垂直方向）
        # 使用左边缘：左上到左下
        pitch_points = (corners['top_left'], corners['bottom_left'])
        pitch_angle_raw = self._calculate_angle(*pitch_points)
        
        # 修正：理想左边缘角度是90°，所以Pitch = 测量值
        pitch_angle = pitch_angle_raw
        
        # 规范化到[0, 180]
        if pitch_angle < 0:
            pitch_angle += 180
        elif pitch_angle > 180:
            pitch_angle -= 180
        
        # 4. 如果Roll角较大，需要修正Yaw和Pitch的测量点
        if abs(roll_angle) > 5.0:  # 如果旋转超过5度
            print(f"检测到传感器旋转 {roll_angle:.1f}°，正在修正Yaw/Pitch测量点...")
            
            # 根据Roll角度旋转测量点
            yaw_points = self._get_rotated_edge_points(corners, 'top', roll_angle)
            pitch_points = self._get_rotated_edge_points(corners, 'left', roll_angle)
            
            # 重新计算Yaw和Pitch
            yaw_angle = self._calculate_angle(*yaw_points)
            pitch_angle = self._calculate_angle(*pitch_points)
            
            # 规范化
            if yaw_angle < 0:
                yaw_angle += 180
            if pitch_angle < 0:
                pitch_angle += 180
        
        angles = (yaw_angle, pitch_angle, roll_angle)
        points = (yaw_points, pitch_points, roll_points)
        
        return angles, points
    
    def _get_rotated_edge_points(self, corners, edge, roll_angle):
        """
        获取旋转后的边缘点
        edge: 'top', 'bottom', 'left', 'right'
        """
        if edge == 'top':
            # 上边缘：考虑Roll旋转后的实际水平方向
            center_x = (corners['top_left'][0] + corners['top_right'][0]) / 2
            center_y = (corners['top_left'][1] + corners['top_right'][1]) / 2
            
            # 计算旋转后的点
            angle_rad = math.radians(roll_angle)
            length = corners['top_right'][0] - corners['top_left'][0]
            
            point1 = (
                int(center_x - length/2 * math.cos(angle_rad)),
                int(center_y - length/2 * math.sin(angle_rad))
            )
            point2 = (
                int(center_x + length/2 * math.cos(angle_rad)),
                int(center_y + length/2 * math.sin(angle_rad))
            )
            
            return (point1, point2)
        
        elif edge == 'left':
            # 左边缘：考虑Roll旋转后的实际垂直方向
            center_x = (corners['top_left'][0] + corners['bottom_left'][0]) / 2
            center_y = (corners['top_left'][1] + corners['bottom_left'][1]) / 2
            
            # 计算旋转后的点
            angle_rad = math.radians(roll_angle + 90)  # 垂直方向
            length = corners['bottom_left'][1] - corners['top_left'][1]
            
            point1 = (
                int(center_x - length/2 * math.cos(angle_rad)),
                int(center_y - length/2 * math.sin(angle_rad))
            )
            point2 = (
                int(center_x + length/2 * math.cos(angle_rad)),
                int(center_y + length/2 * math.sin(angle_rad))
            )
            
            return (point1, point2)
        
        # 默认返回原始边缘
        if edge == 'top':
            return (corners['top_left'], corners['top_right'])
        elif edge == 'left':
            return (corners['top_left'], corners['bottom_left'])
        else:
            return (corners['top_left'], corners['top_right'])
    
    def _calculate_angle(self, point1, point2):
        """计算两点连线的角度（度）"""
        x1, y1 = point1
        x2, y2 = point2
        
        # 计算角度（相对于水平线，-180到180度）
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        
        return angle
    
    def _check_tolerance(self, deviations):
        """检查角度偏差是否在公差范围内"""
        if not deviations:
            return {'all_ok': True, 'details': {}}
        
        result = {'all_ok': True, 'details': {}}
        
        for angle_name, deviation in deviations.items():
            abs_deviation = abs(deviation)
            is_ok = abs_deviation <= self.tolerance
            
            result['details'][angle_name] = {
                'deviation': deviation,
                'abs_deviation': abs_deviation,
                'is_ok': is_ok,
                'tolerance': self.tolerance
            }
            
            if not is_ok:
                result['all_ok'] = False
        
        return result
    
    def _visualize_results(self, img, roi, yaw_points, pitch_points, roll_points,
                          yaw_angle, pitch_angle, roll_angle,
                          deviations, check_result):
        """可视化所有结果"""
        result_img = img.copy()
        x, y, w, h = roi
        
        # 1. 绘制传感器区域
        cv2.rectangle(result_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
        # 2. 绘制测量点和线（不同颜色）
        colors = {
            'yaw': (0, 255, 0),    # 绿色
            'pitch': (255, 0, 0),  # 蓝色
            'roll': (0, 0, 255)    # 红色
        }
        
        points_sets = {
            'yaw': yaw_points,
            'pitch': pitch_points,
            'roll': roll_points
        }
        
        angles = {
            'yaw': yaw_angle,
            'pitch': pitch_angle,
            'roll': roll_angle
        }
        
        # 绘制每个角度的测量线和角度值
        for angle_name in ['yaw', 'pitch', 'roll']:
            points = points_sets[angle_name]
            color = colors[angle_name]
            
            # 绘制点和线
            for point in points:
                cv2.circle(result_img, point, 8, color, -1)
                cv2.circle(result_img, point, 10, (255, 255, 255), 2)
            
            cv2.line(result_img, points[0], points[1], color, 3)
            
            # 在线旁边显示角度
            mid_x = (points[0][0] + points[1][0]) // 2
            mid_y = (points[0][1] + points[1][1]) // 2
            
            angle_text = f"{angle_name.upper()}: {angles[angle_name]:.1f}°"
            cv2.putText(result_img, angle_text, 
                       (mid_x + 10, mid_y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # 3. 创建信息面板
        height, width = img.shape[:2]
        
        # 信息面板背景
        info_height = 220
        overlay = result_img.copy()
        cv2.rectangle(overlay, (10, 10), (500, info_height), (0, 0, 0), -1)
        result_img = cv2.addWeighted(overlay, 0.7, result_img, 0.3, 0)
        
        # 4. 显示角度信息
        y_pos = 40
        title = "SENSOR ANGLE AUTO CHECK"
        cv2.putText(result_img, title, (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        y_pos += 40
        
        # 显示三个角度
        for angle_name in ['yaw', 'pitch', 'roll']:
            angle_value = angles[angle_name]
            color = colors[angle_name]
            
            # 如果有偏差，显示偏差值
            if deviations and angle_name in deviations:
                deviation = deviations[angle_name]
                is_ok = check_result['details'][angle_name]['is_ok']
                
                status_color = (0, 255, 0) if is_ok else (0, 0, 255)
                status_text = "✓" if is_ok else "✗"
                
                text = f"{angle_name.upper()}: {angle_value:.2f}°"
                text += f" (Δ={deviation:+.2f}°) {status_text}"
                
                cv2.putText(result_img, text, (20, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            else:
                text = f"{angle_name.upper()}: {angle_value:.2f}°"
                cv2.putText(result_img, text, (20, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            y_pos += 30
        
        # 5. 显示总体结果
        y_pos += 10
        if check_result['all_ok']:
            result_text = f"✅ ALL ANGLES WITHIN TOLERANCE (±{self.tolerance}°)"
            result_color = (0, 255, 0)
        else:
            result_text = f"❌ SOME ANGLES EXCEED TOLERANCE (±{self.tolerance}°)"
            result_color = (0, 0, 255)
        
        cv2.putText(result_img, result_text, (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, result_color, 2)
        
        # 6. 显示图例
        legend_y = height - 150
        cv2.putText(result_img, "LEGEND:", (width - 200, legend_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        legend_items = [
            ("Yaw (Green)", (0, 255, 0)),
            ("Pitch (Blue)", (255, 0, 0)),
            ("Roll (Red)", (0, 0, 255)),
            ("Sensor ROI", (255, 0, 0))
        ]
        
        for i, (text, color) in enumerate(legend_items):
            cv2.putText(result_img, text, (width - 200, legend_y + 25 + i*25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return result_img
    
    def _save_result(self, result_img):
        """保存结果图像"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_angle_auto_check_{timestamp}.jpg"
        
        try:
            cv2.imwrite(filename, result_img)
            print(f"✅ 结果已保存: {filename}")
            return filename
        except Exception as e:
            print(f"❌ 保存失败: {e}")
            return None

# 使用示例
def main():
    # 创建自动检测器，设置公差为1度
    checker = AutoSensorAngleChecker(tolerance=1.0)
    
    # 设置参考角度（理想安装角度）
    # 根据你的描述，可能需要调整这些值
    # 如果传感器是180°安装，Yaw参考可能是180°
    checker.set_reference_angles(yaw=0.0, pitch=90.0, roll=0.0)
    
    # 图像路径
    image_path = '/home/qcraft/work/py_script/CV2/potor/20251201-161224.jpg'
    
    # 执行自动角度检测
    print("\n开始传感器角度自动检测...")
    result = checker.auto_angle_measurement(image_path)
    
    if result:
        print("检测结果汇总")
        print("="*50)
        
        angles = result['angles']
        deviations = result['deviations']
        check_result = result['check_result']
        
        print(f"\n测量角度:")
        print(f"  Yaw:   {angles['yaw']:.2f}")
        print(f"  Pitch: {angles['pitch']:.2f}")
        print(f"  Roll:  {angles['roll']:.2f}")
        
        if deviations:
            print(f"\n与参考角度的偏差:")
            for angle_name, deviation in deviations.items():
                is_ok = check_result['details'][angle_name]['is_ok']
                status = "✓ OK" if is_ok else "✗ NG"
                print(f"  {angle_name.upper()}: {deviation:+.2f}° {status}")
        
        print(f"\n总体结果: {'✓ 所有角度合格' if check_result['all_ok'] else '✗ 存在角度超差'}")
        
        # 显示测量点信息
        points = result['points']
        print(f"\n测量点坐标:")
        print(f"  Yaw点: {points[0][0]} -> {points[0][1]}")
        print(f"  Pitch点: {points[1][0]} -> {points[1][1]}")
        print(f"  Roll点: {points[2][0]} -> {points[2][1]}")
        
        # 显示ROI信息
        roi = result['roi']
        print(f"\n传感器ROI: x={roi[0]}, y={roi[1]}, w={roi[2]}, h={roi[3]}")
        
        # 等待用户查看结果
        print("\n按任意键关闭结果窗口...")
        # cv2.waitKey(0)
        cv2.destroyAllWindows()
# 在 main() 函数后面添加以下代码：

if __name__ == "__main__":
    main()
