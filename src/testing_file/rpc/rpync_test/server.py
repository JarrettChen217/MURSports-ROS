import rpyc
from rpyc.utils.server import ThreadedServer
import threading

# 定义包含 add 函数和关闭方法的服务
class AddService(rpyc.Service):
    def exposed_add(self, x, y):
        return x + y

    def exposed_shutdown(self):
        print("Shutdown command received. Server is shutting down.")
        # 设置一个标志来指示服务器应该关闭
        global should_shutdown
        should_shutdown = True

# 创建标志变量
should_shutdown = False

# 启动RPC服务的函数
def start_server():
    global server
    server = ThreadedServer(AddService, port=18861, hostname='0.0.0.0')
    print("Server is listening on port 18861...")
    server.start()

# 启动服务器线程
server_thread = threading.Thread(target=start_server)
server_thread.start()

# 主线程循环检测关闭标志
try:
    while not should_shutdown:
        pass  # 主线程等待直到 should_shutdown 为 True
finally:
    server.close()  # 关闭服务器
    server_thread.join()  # 等待服务器线程完成
    print("Server has been shut down.")