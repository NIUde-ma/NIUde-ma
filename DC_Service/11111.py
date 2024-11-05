import cv2
import numpy as np
# 读取一张斯里兰卡拍摄的大象照片
img = cv2.imread('niude.jpg')

# 沿着横纵轴放大1.6倍，然后平移(-150,-240)，最后沿原图大小截取，等效于裁剪并放大
M_crop_elephant = np.array([
    [0.9, 0, -150],
    [0, 1.2, -240]
], dtype=np.float32)

img_elephant = cv2.warpAffine(img, M_crop_elephant, (400, 600))
cv2.imwrite('lanka_elephant.jpg', img_elephant)
cv2.imwrite("niude2.jpg",img,[cv2.IMWRITE_JPEG_QUALITY,100])

# x轴的剪切
theta = 15 * np.pi / 180
M_shear = np.array([
    [1, np.tan(theta), 0],
    [0, 1, 0]
], dtype=np.float32)

img_sheared = cv2.warpAffine(img, M_shear, (400, 600))
cv2.imwrite('lanka_safari_sheared.jpg', img_sheared)

# 顺时针旋转，角度15°
M_rotate = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0]
], dtype=np.float32)

img_rotated = cv2.warpAffine(img, M_rotate, (400, 600))
cv2.imwrite('lanka_safari_rotated.jpg', img_rotated)

# 某种变换，具体旋转+缩放+旋转组合可以通过SVD分解理解
M = np.array([
    [1, 1.5, -400],
    [0.5, 2, -100]
], dtype=np.float32)

img_transformed = cv2.warpAffine(img, M, (400, 600))
cv2.imwrite('lanka_safari_transformed.jpg', img_transformed)


def ma():
    import cv2
    
    image = cv2.imread('zhu.jpg')
    image1_1 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = image1_1.shape
    image1_2 = image1_1.copy()
    for i in range(h):
        for j in range(w):
            # image1_2[i, j] = 255 - image1_2[i, j]  # 黑字白底
            image1_2[i, j] = image1_2[i, j] - 255  # 白字黑底

    ret, image2 = cv2.threshold(image1_2, 100, 255, cv2.THRESH_BINARY)

    # 定义马赛克大小（每个正方形区域）
    mosaic_size = (10, 10)
    
    # 获取图像的高度、宽度和通道数
    height, width, channels = image.shape
    
    # 计算需要分成多少个马赛克单元格
    num_rows = height // mosaic_size[0] + 1
    num_cols = width // mosaic_size[1] + 1
    
    # 创建新的空白图像，与原始图像相同大小
    output_image = np.zeros((height, width, channels), dtype=np.uint8)
    
    # 遍历所有的马赛克单元格并将其合并为一个平均值
    for row in range(num_rows):
        for col in range(num_cols):
            # 确定当前马赛克单元格的左上角和右下角位置
            start_row = row * mosaic_size[0]
            end_row = min((row+1)*mosaic_size[0], height)
            
            start_col = col * mosaic_size[1]
            end_col = min((col+1)*mosaic_size[1], width)
            
            # 提取当前马赛克单元格内的像素
            cell_pixels = image[start_row:end_row, start_col:end_col].reshape(-1, channels).astype(float)
            
            # 计算该马赛克单元格内像素的平均值
            average_color = np.mean(cell_pixels, axis=0)
            
            # 设置输出图像中对应位置的像素值为平均色调
            output_image[start_row:end_row, start_col:end_col] = average_color.astype(int)
           
    # 保存结果图像
    cv2.imwrite('output_image.jpg', output_image)
    cv2.imwrite("zhu1.jpg",image,[cv2.IMWRITE_JPEG_LUMA_QUALITY,100, cv2.IMWRITE_JPEG_QUALITY,100])
    cv2.imwrite("a.jpg", image2)

    
if __name__ == '__main__':
    ma()
