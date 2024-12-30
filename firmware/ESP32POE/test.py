import socket

TCP_IP = "192.168.1.236"  # The IP to bind to
TCP_PORT = 2718

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse the address/port
    sock.bind((TCP_IP, TCP_PORT))
    sock.listen(1)
    print(f"Server is listening on {TCP_IP}:{TCP_PORT}")
except socket.error as e:
    print(f"Socket error: {e}")
    exit(1)

while True:
    print("Waiting for a connection...")
    try:
        conn, addr = sock.accept()
        print(f"Connected by {addr}")
        data = conn.recv(1024)
        if data:
            print(f"Received data: {data.hex()}")
        conn.close()
    except Exception as e:
        print(f"Error during connection: {e}")