import socket

# 连接到服务器
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(("localhost", 8000))

# 发送消息
client_socket.sendall(b"Hello from client!")
# 接收响应
data = client_socket.recv(1024)
print("Received from server:", data.decode())

client_socket.close()