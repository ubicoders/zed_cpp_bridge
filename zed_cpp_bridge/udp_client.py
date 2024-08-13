import socket

def udp_client(message, server_ip='127.0.0.1', server_port=12345):
    # Create a UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send the message to the server
        udp_socket.sendto(message.encode('utf-8'), (server_ip, server_port))
        print(f"Sent message: {message} to {server_ip}:{server_port}")
        data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        udp_socket.sendto("rec".encode('utf-8'), addr)  # Echo back the received data
        message = data.decode('utf-8')
        print(f"Received: {message} from {addr}")

    finally:
        # Close the socket
        udp_socket.close()

if __name__ == '__main__':
    message = "Hello from UDP client"
    udp_client(message)
