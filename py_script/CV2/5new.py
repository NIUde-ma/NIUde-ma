import cv2
import numpy as np
import math

class ManualSensorAngleChecker:
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
    
    def manual_angle_measurement(self, image_path):
        """
        手动测量传感器三角度
        通过选择关键点来计算实际角度
        """
        img = cv2.imread(image_path)
        if img is None:
            print("无法读取图像")
            return None
        
        print("\n" + "="*50)
        print("传感器三角度手动测量")
        print("="*50)
        
        # 1. 框选传感器区域（可选，用于可视化）
        print("\n步骤1: 框选传感器整体区域（用于参考）")
        print("用鼠标拖拽选择矩形区域，按空格确认")
        roi = cv2.selectROI("Select Sensor Area", img, False)
        cv2.destroyAllWindows()
        
        if roi[2] == 0 or roi[3] == 0:
            print("未选择区域")
            return None
        
        x, y, w, h = roi
        print(f"传感器区域: ({x}, {y}) {w}x{h}")
        
        # 2. 选择关键点计算Roll角（传感器自身旋转）
        print("\n步骤2: 选择Roll角测量点")
        print("点击选择传感器上的两个特征点（如边缘、标记点）")
        print("这两个点应该能反映传感器的旋转角度")
        roll_points = self._select_two_points(img, "Select Roll Measurement Points")
        if roll_points is None:
            return None
        
        # 计算Roll角（相对于水平线）
        roll_angle = self._calculate_angle(roll_points[0], roll_points[1])
        print(f"Roll角（原始）: {roll_angle:.2f}°")
        
        # 3. 选择关键点计算Yaw角（水平方向）
        print("\n步骤3: 选择Yaw角测量点")
        print("点击选择两个点来测量水平方向角度")
        print("例如：传感器的左右边缘点")
        yaw_points = self._select_two_points(img, "Select Yaw Measurement Points")
        if yaw_points is None:
            return None
        
        yaw_angle = self._calculate_angle(yaw_points[0], yaw_points[1])
        print(f"Yaw角（原始）: {yaw_angle:.2f}°")
        
        # 4. 选择关键点计算Pitch角（垂直方向）
        print("\n步骤4: 选择Pitch角测量点")
        print("点击选择两个点来测量垂直方向角度")
        print("例如：传感器的上下边缘点")
        pitch_points = self._select_two_points(img, "Select Pitch Measurement Points")
        if pitch_points is None:
            return None
        
        pitch_angle = self._calculate_angle(pitch_points[0], pitch_points[1])
        print(f"Pitch角（原始）: {pitch_angle:.2f}°")
        
        # 5. 角度修正（根据你的需求调整）
        print("\n步骤5: 角度修正")
        
        # 根据你的描述，可能需要180°修正
        # 如果传感器是180°安装，Yaw需要180°修正
        yaw_angle_corrected = 180.0 - yaw_angle if yaw_angle > 90 else yaw_angle
        
        # Pitch角：理想垂直是90°，所以计算偏差
        pitch_angle_corrected = pitch_angle
        
        # Roll角：理想对角线是45°，所以计算偏差
        roll_angle_corrected = roll_angle - 45.0
        
        print(f"Yaw角（修正后）: {yaw_angle_corrected:.2f}°")
        print(f"Pitch角（修正后）: {pitch_angle_corrected:.2f}°")
        print(f"Roll角（修正后）: {roll_angle_corrected:.2f}°")
        
        # 6. 计算与参考角度的偏差
        deviations = {}
        if self.reference_angles:
            deviations = {
                'yaw': yaw_angle_corrected - self.reference_angles['yaw'],
                'pitch': pitch_angle_corrected - self.reference_angles['pitch'],
                'roll': roll_angle_corrected - self.reference_angles['roll']
            }
        
        # 7. 检查是否超差
        check_result = self._check_tolerance(deviations)
        
        # 8. 可视化结果
        result_img = self._visualize_results(
            img, roi, yaw_points, pitch_points, roll_points,
            yaw_angle_corrected, pitch_angle_corrected, roll_angle_corrected,
            deviations, check_result
        )
        
        # 9. 保存结果
        self._save_result(result_img)
        
        return {
            'angles': {
                'yaw': yaw_angle_corrected,
                'pitch': pitch_angle_corrected,
                'roll': roll_angle_corrected
            },
            'angles_raw': {
                'yaw': yaw_angle,
                'pitch': pitch_angle,
                'roll': roll_angle
            },
            'deviations': deviations,
            'check_result': check_result,
            'roi': roi,
            'points': {
                'yaw': yaw_points,
                'pitch': pitch_points,
                'roll': roll_points
            }
        }
    
    def _select_two_points(self, img, window_title):
        """选择两个点"""
        display_img = img.copy()
        points = []
        
        def mouse_callback(event, x, y, flags, param):
            nonlocal display_img, points
            
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(points) < 2:
                    points.append((x, y))
                    print(f"点{len(points)}: ({x}, {y})")
                    
                    # 绘制点
                    cv2.circle(display_img, (x, y), 5, (0, 255, 0), -1)
                    
                    # 如果已经有两个点，绘制线
                    if len(points) == 2:
                        cv2.line(display_img, points[0], points[1], (0, 255, 0), 2)
        
        cv2.namedWindow(window_title)
        cv2.setMouseCallback(window_title, mouse_callback)
        
        print(f"在 '{window_title}' 窗口中点击选择两个点")
        print("按 's' 保存，按 'r' 重选，按 'q' 退出")
        
        while True:
            cv2.imshow(window_title, display_img)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):  # 保存
                if len(points) == 2:
                    cv2.destroyWindow(window_title)
                    return points
                else:
                    print("请先选择两个点")
            
            elif key == ord('r'):  # 重选
                points = []
                display_img = img.copy()
                print("已重置，请重新选择")
            
            elif key == ord('q'):  # 退出
                cv2.destroyWindow(window_title)
                return None
    
    def _calculate_angle(self, point1, point2):
        """计算两点连线的角度（0-180度）"""
        x1, y1 = point1
        x2, y2 = point2
        
        # 计算角度（相对于水平线）
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        
        # 调整到0-180度
        if angle < 0:
            angle += 180
        
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
        
        # 1. 绘制传感器区域（参考）
        cv2.rectangle(result_img, (x, y), (x+w, y+h), (255, 0, 0), 1)
        
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
        title = "SENSOR ANGLE MANUAL CHECK"
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
        
        # 显示窗口
        cv2.imshow("Sensor Angle Results", result_img)
        
        return result_img
    
    def _save_result(self, result_img):
        """保存结果图像"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_angle_manual_check_{timestamp}.jpg"
        
        try:
            cv2.imwrite(filename, result_img)
            print(f"✅ 结果已保存: {filename}")
            return filename
        except Exception as e:
            print(f"保存失败: {e}")
            return None

# 使用示例
def main():
    # 创建检测器，设置公差为1度
    checker = ManualSensorAngleChecker(tolerance=1.0)
    
    # 设置参考角度（理想安装角度）
    # 根据你的描述调整这些值
    checker.set_reference_angles(yaw=0.0, pitch=90.0, roll=0.0)
    
    # 图像路径
    image_path = '/home/qcraft/work/py_script/CV2/potor/20251201-134902.jpg'
    
    # 执行角度检测
    print("\n开始传感器角度检测...")
    result = checker.manual_angle_measurement(image_path)
    
    if result:
        print("\n" + "="*50)
        print("检测结果汇总")
        print("="*50)
        
        angles = result['angles']
        angles_raw = result['angles_raw']
        deviations = result['deviations']
        check_result = result['check_result']
        
        print(f"\n原始测量角度:")
        print(f"  Yaw:   {angles_raw['yaw']:.2f}°")
        print(f"  Pitch: {angles_raw['pitch']:.2f}°")
        print(f"  Roll:  {angles_raw['roll']:.2f}°")
        
        print(f"\n修正后角度:")
        print(f"  Yaw:   {angles['yaw']:.2f}°")
        print(f"  Pitch: {angles['pitch']:.2f}°")
        print(f"  Roll:  {angles['roll']:.2f}°")
        
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
        print(f"  Yaw点: {points['yaw'][0]} -> {points['yaw'][1]}")
        print(f"  Pitch点: {points['pitch'][0]} -> {points['pitch'][1]}")
        print(f"  Roll点: {points['roll'][0]} -> {points['roll'][1]}")
        
        # 显示ROI信息
        roi = result['roi']
        print(f"\n传感器ROI: x={roi[0]}, y={roi[1]}, w={roi[2]}, h={roi[3]}")
        
        # 等待用户查看结果
        print("\n按任意键关闭结果窗口...")
        # cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        return result
    else:
        print("❌ 检测失败")
        return None

if __name__ == "__main__":
    main()
        