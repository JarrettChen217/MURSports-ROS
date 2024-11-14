import socket

# 创建Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("localhost", 8000))
server_socket.listen(1)
print("Server is listening on port 8000...")

# 接受客户端连接
conn, addr = server_socket.accept()
print(f"Connected by {addr}")

while True:
    # 接收数据
    data = conn.recv(1024)
    if not data:
        break
    print("Received from client:", data.decode())
    # 发送响应
    conn.sendall(b"Message received")

conn.close()
server_socket.close()