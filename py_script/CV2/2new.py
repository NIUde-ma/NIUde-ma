import cv2
import numpy as np

def detect_sensor_mounting_angle_simple(image_path):
    """
    直接检测传感器安装面角度
    """
    # 1. 读取图像
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 2. 手动选择安装面区域（关键步骤！）
    print("请用鼠标选择传感器安装面区域（矩形）")
    print("选择完成后按空格键确认，按ESC取消")
    
    roi = cv2.selectROI("Select Mounting Surface", img, False)
    cv2.destroyAllWindows()
    
    if roi[2] == 0 or roi[3] == 0:
        print("未选择区域")
        return None
    
    x, y, w, h = roi
    roi_img = gray[y:y+h, x:x+w]
    
    # 3. 增强安装面边缘
    # 安装面通常是水平或垂直的直线，所以增强这些方向的边缘
    kernel_h = np.ones((1, 20), np.uint8)  # 水平核
    kernel_v = np.ones((20, 1), np.uint8)  # 垂直核
    
    # 二值化
    _, binary = cv2.threshold(roi_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # 增强水平边缘
    horizontal = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_h)
    # 增强垂直边缘
    vertical = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_v)
    
    # 合并
    edges = cv2.bitwise_or(horizontal, vertical)
    
    # 4. 检测直线
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, 
                           minLineLength=w//2, maxLineGap=10)
    
    if lines is None:
        print("未检测到安装面直线")
        return None
    
    # 5. 计算角度
    angles = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        
        # 计算角度（相对于水平线）
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        
        # 调整到0-180度
        if angle < 0:
            angle += 180
        
        # 安装面通常是接近90度（垂直）或0度（水平）
        angles.append(angle)
    
    # 6. 找到最接近90度的角度（假设安装面应该是垂直的）
    angles_array = np.array(angles)
    
    # 计算每个角度与90度的差值
    diffs = np.abs(angles_array - 90)
    
    # 选择最接近90度的角度
    best_idx = np.argmin(diffs)
    best_angle = angles_array[best_idx]
    
    # 7. 可视化
    result_img = img.copy()
    
    # 绘制ROI
    cv2.rectangle(result_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    
    # 绘制检测到的直线
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(result_img, (x+x1, y+y1), (x+x2, y+y2), (0, 255, 0), 2)
    
    # 绘制最佳角度线
    best_line = lines[best_idx][0]
    x1, y1, x2, y2 = best_line
    cv2.line(result_img, (x+x1, y+y1), (x+x2, y+y2), (0, 0, 255), 3)
    
    # 显示角度
    text = f"Mounting Angle: {best_angle:.1f}^"
    cv2.putText(result_img, text, (50, 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imwrite("mounting_angle_simple.jpg", result_img)
    
    return best_angle

# 使用
angle = detect_sensor_mounting_angle_simple('/home/qcraft/work/py_script/CV2/potor/20251201-145308.jpg')
if angle is not None:
    print(f"安装面角度: {angle:.2f}°")