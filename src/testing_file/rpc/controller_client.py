import rpyc

# 连接到服务器
conn = rpyc.connect("localhost", 18859)

# 设置线速度和角速度控制值，范围从-1到1
conn.root.exposed_set_control(0.5, -0.3)  # 例如，线速度为0.5，角速度为-0.3

# 关闭连接
# conn.close()
