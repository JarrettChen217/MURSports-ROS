import rpyc

# 连接到服务器
conn = rpyc.connect("localhost", 18861)

# 调用服务器的 add 函数
result = conn.root.add(3, 5)
print("Result from server:", result)  # 输出：Result from server: 8

# 发送关闭指令
conn.root.shutdown()
print("Shutdown command sent to the server.")

# 关闭客户端连接
conn.close()