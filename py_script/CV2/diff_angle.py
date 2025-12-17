import cv2
import numpy as np

def detect_sensor_angle_roi(image_path, sensor_roi=None):
    """
    在指定ROI区域内检测传感器角度
    sensor_roi: (x, y, width, height) 传感器区域
    """
    
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print("无法读取图片")
        return None
    
    # 如果没有提供ROI，手动选择或使用默认值
    if sensor_roi is None:
        # 这里需要根据你的图片调整
        # 假设传感器在图像中心区域
        height, width = img.shape[:2]
        sensor_roi = (width//4, height//4, width//2, height//2)
    
    x, y, w, h = sensor_roi
    roi_img = img[y:y+h, x:x+w]
    
    # 转换为HSV颜色空间，更容易分离传感器
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    
    # 根据传感器颜色设置阈值（需要根据实际传感器颜色调整）
    # 假设传感器是深色（黑色或深灰色）
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 100])
    mask = cv2.inRange(hsv, lower, upper)
    
    # 形态学操作，去除噪点
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # 查找传感器轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        print("未找到传感器轮廓")
        return None
    
    # 选择最大的轮廓（假设传感器是最大的物体）
    largest_contour = max(contours, key=cv2.contourArea)
    
    # 计算最小外接矩形
    rect = cv2.minAreaRect(largest_contour)
    angle = rect[2]
    
    # 调整角度范围到0-90度
    if angle < -45:
        angle = 90 + angle
    elif angle > 45:
        angle = angle - 90
    
    return abs(angle)

# 使用示例
# angle = detect_sensor_angle_roi(image_path)
# print(f"传感器角度: {angle:.2f}°")



# 使用示例
image_path = '/root/work/py_script/CV2/potor/20251201-134136.jpg'
angles_need = detect_sensor_angle_roi(image_path)
print(angles_need)
need_angle = 90
diff = 2  # ±2°

if angles_need is not None:
    print(f"实际{angles_need} - 工装{need_angle}")
    bad = abs(angles_need - need_angle)
    if bad > 5:
        print(f"角度超差：{bad}°")
    else:
        print(f"角度符合预期：{int(need_angle)}°")
