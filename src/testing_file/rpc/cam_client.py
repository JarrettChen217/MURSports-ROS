import rpyc
from cv_bridge import CvBridge
import cv2
import numpy as np

# 连接到服务器
conn = rpyc.connect("localhost", 18860)

while True:
    # 从服务器获取图像数据
    # image_msg = conn.root.exposed_read_image()

    # if image_msg is None:
    #     print("No image received. Waiting...")
    #     continue  # 如果没有接收到图像，等待下一次获取

    image_bytes = conn.root.exposed_cv_image()
    if image_bytes is None:
        print("No image received. Waiting...")
        continue

    # 将字节流转换为OpenCV格式的图像
    image_array = np.frombuffer(image_bytes, dtype=np.uint8)
    cv_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

    try:        
        # 显示图像
        cv2.imshow("Frame", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # 按下 'q' 键退出
    except Exception as e:
        print(f"Error in converting image: {e}")

# 关闭连接和窗口
# conn.close()
# cv2.destroyAllWindows()
