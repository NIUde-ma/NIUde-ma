import cv2
import numpy as np
import os

def calculate_installation_error_simple(camera_path, simulation_path, output_path, threshold=30):
    """
    简化版本：计算相机视野图与仿真设计图的误差
    通过调整仿真图像大小来匹配相机图像，然后计算差值
    """
    # 读取图像
    camera_img = cv2.imread(camera_path)
    simulation_img = cv2.imread(simulation_path)
    
    if camera_img is None or simulation_img is None:
        raise ValueError("无法加载图像文件")
    
    # 获取相机图像的尺寸
    target_height, target_width = camera_img.shape[:2]
    
    # 调整仿真图像大小以匹配相机图像，保持宽高比并填充
    sim_height, sim_width = simulation_img.shape[:2]
    
    # 计算缩放比例
    scale = min(target_width / sim_width, target_height / sim_height)
    
    # 计算新尺寸
    new_width = int(sim_width * scale)
    new_height = int(sim_height * scale)
    
    # 缩放仿真图像
    resized_simulation = cv2.resize(simulation_img, (new_width, new_height))
    
    # 创建与相机图像相同大小的画布
    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    
    # 计算居中位置
    start_y = (target_height - new_height) // 2
    start_x = (target_width - new_width) // 2
    
    # 将调整大小后的仿真图像放置在画布中心
    canvas[start_y:start_y+new_height, start_x:start_x+new_width] = resized_simulation
    
    # 保存投影图像
    cv2.imwrite(output_path, canvas)
    print(f"投影图像已保存到: {output_path}")
    
    # 转换为灰度图进行比较
    gray_camera = cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY)
    gray_projected = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    
    # 计算绝对差值
    diff = cv2.absdiff(gray_projected, gray_camera)
    
    # 应用阈值以突出差异
    _, thresholded = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    
    # 计算误差百分比
    total_pixels = diff.size
    error_pixels = np.count_nonzero(thresholded)
    error_percentage = (error_pixels / total_pixels) * 100
    
    return error_percentage

def calculate_installation_error_with_registration(camera_path, simulation_path, output_path, threshold=30):
    """
    使用特征点匹配和透视变换的版本
    """
    # 读取图像
    camera_img = cv2.imread(camera_path)
    simulation_img = cv2.imread(simulation_path)
    
    if camera_img is None or simulation_img is None:
        raise ValueError("无法加载图像文件")
    
    # 转换为灰度图
    gray_camera = cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY)
    gray_simulation = cv2.cvtColor(simulation_img, cv2.COLOR_BGR2GRAY)
    
    # 创建ORB特征检测器
    orb = cv2.ORB_create(nfeatures=1000)
    
    # 检测关键点和描述符
    kp1, des1 = orb.detectAndCompute(gray_camera, None)
    kp2, des2 = orb.detectAndCompute(gray_simulation, None)
    
    if des1 is None or des2 is None:
        print("警告: 无法检测到特征点，使用简单的尺寸调整")
        return calculate_installation_error_simple(camera_path, simulation_path, output_path, threshold)
    
    # 特征点匹配
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    
    # 按距离排序
    matches = sorted(matches, key=lambda x: x.distance)
    
    # 获取最佳匹配点
    if len(matches) >= 4:
        # 获取匹配点的坐标
        src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # 计算单应性矩阵
        homography_matrix, mask = cv2.findHomography(
            dst_pts, src_pts, cv2.RANSAC, 5.0
        )
        
        if homography_matrix is not None:
            # 使用单应性矩阵进行透视变换
            h, w = camera_img.shape[:2]
            projected_simulation = cv2.warpPerspective(
                simulation_img, 
                homography_matrix, 
                (w, h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0)
            )
            
            # 保存投影图像
            cv2.imwrite(output_path, projected_simulation)
            print(f"投影图像已保存到: {output_path}")
            
            # 转换为灰度图进行比较
            gray_projected = cv2.cvtColor(projected_simulation, cv2.COLOR_BGR2GRAY)
            
            # 计算绝对差值
            diff = cv2.absdiff(gray_projected, gray_camera)
            
            # 应用阈值以突出差异
            _, thresholded = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
            
            # 计算误差百分比
            total_pixels = diff.size
            error_pixels = np.count_nonzero(thresholded)
            error_percentage = (error_pixels / total_pixels) * 100
            
            return error_percentage
        else:
            print("警告: 无法计算单应性矩阵，使用简单的尺寸调整")
            return calculate_installation_error_simple(camera_path, simulation_path, output_path, threshold)
    else:
        print("警告: 匹配点不足，使用简单的尺寸调整")
        return calculate_installation_error_simple(camera_path, simulation_path, output_path, threshold)

def main():
    # 示例使用
    camera_path = "./potor/img_v3_02sp_7a5ee115-b019-4260-81a5-9fbd05e0e76g.jpg"  # 相机视野图路径
    simulation_path = "./potor/img_v3_02sp_e51b3634-66a9-4d6b-9b3d-69e53271910g.jpg"  # 仿真设计图路径
    projected_output_path = "projected_simulation.jpg"  # 投影图像保存路径

    try:
        # 使用特征点匹配的方法
        diff = calculate_installation_error_with_registration(
            camera_path, 
            simulation_path, 
            projected_output_path,
            threshold=30
        )
        
        print(f"安装diff: {diff:.2f}%")
        
        # 判断是否超差
        tolerance_threshold = 5  # 假设5%为超差阈值
        if diff > tolerance_threshold:
            print(f"检测到安装超差！diff({diff:.2f}%)超过阈值({tolerance_threshold}%)")
        else:
            print(f"安装符合要求，diff({diff:.2f}%)在允许范围内")
            
    except Exception as e:
        print(f"计算过程中出现错误: {e}")
        print("请确保图像文件路径正确且图像可读")
        print("示例文件不存在，这里展示如何使用:")
        print("diff = calculate_installation_error_with_registration('your_camera.jpg', 'your_simulation.jpg', 'projected_output.jpg')")

if __name__ == "__main__":
    main()