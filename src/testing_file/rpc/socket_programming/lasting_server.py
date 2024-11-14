import socket

# 创建Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("localhost", 8000))
server_socket.listen(5)  # 允许最多5个等待连接的客户端
print("Server is listening on port 8000...")

while True:
    # 接受客户端连接
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    while True:
        # 接收数据
        data = conn.recv(1024)
        if not data:
            print(f"Client {addr} disconnected.")
            break
        print("Received from client:", data.decode())
        # 发送响应
        conn.sendall(b"Message received")

    conn.close()  # 关闭与当前客户端的连接


import socket

# 定义可用的函数
def add(x, y):
    return x + y

def subtract(x, y):
    return x - y

# 函数映射表
function_map = {
    "add": add,
    "subtract": subtract
}

# 创建Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("localhost", 8000))
server_socket.listen(1)
print("Server is listening on port 8000...")

while True:
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    while True:
        # 接收数据（假设数据格式为 "function_name,arg1,arg2"）
        data = conn.recv(1024)
        if not data:
            break

        # 解析数据
        decoded_data = data.decode()
        func_name, *args = decoded_data.split(",")
        args = map(int, args)  # 假设参数是整数，转换为int

        # 根据名称调用函数
        if func_name in function_map:
            result = function_map[func_name](*args)
            conn.sendall(f"Result: {result}".encode())
        else:
            conn.sendall(b"Function not found")

    conn.close()