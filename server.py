import socket
import threading
import time

HOST = '0.0.0.0'  # server host
PORT = 8090  # server port

debug_mode = False

CHARACTERS = ["All","Rocco","Eva","Lele","Carlotta","Peppe","Bianca","Cosimo"]
IPs = [HOST,"","","","","","",""]
EMOTIONS = {"B":"happy","E": "angry","D": "shocked","E": "sad","F": "relaxed", "G": "afraid","H": "cautious","I": "surprised","J": "annoyed","K": "embarrassed","L": "anxious"}
# create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# bind the socket to a specific address and port
sock.bind((HOST, PORT))

# list to store client connections
connections = []

#Choose debug mode
debug_mode = input("Choose mode (1 - Shows actual messages , 0 - Shows human language messages): ")
print("Example - Rocco says: 1A21" if debug_mode=="1" else "Example - Rocco is happy(1) with Eva")

def handle_client(conn, addr):
    """Thread function to handle a client connection"""
    print(f"Client connected from {addr[0]}:{addr[1]}")
    #conn.sendall("Welcome to 'Perfect Strangers' Simulation!".encode())

    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                break
            # send the message to all connected clients
            for c in connections:
                if c != conn:
                    time.sleep(5)
                    c.sendall(f"{data}".encode())#f"{addr[0]}:{addr[1]} says: {data}".encode())
            
            # log the communication to the console
            #print(f"{addr[0]}:{addr[1]} says: {data}")
            if debug_mode=="1":
                print(f"{CHARACTERS[int(data[0])]} says: {data}")#f"{addr[0]}:{addr[1]} says: {data}")
                IPs[int(data[0])] = addr[0]
            else: 
                print(f"{ CHARACTERS[int(data[0])]} is {EMOTIONS[data[1]]}({data[3]}) with {CHARACTERS[int(data[2])]}")
                
        except Exception as e:
            print(f"Error: {e}")
            break

    # remove the connection from the list and close it
    connections.remove(conn)
    conn.close()
    print(f"Connection closed with {addr[0]}:{addr[1]}")

# listen for incoming connections
sock.listen(5)
print(f"Server listening on {HOST}:{PORT}...")

# thread function to read console input
def console_input():
    while True:
        try:
            data = input()
            # send the message to all connected clients
            for c in connections:
                c.sendall(f"{data}".encode())
            # log the communication to the console
            #print(f"Server says: {data}")
        except Exception as e:
            print(f"Error: {e}")
            break

# start a thread to read console input
threading.Thread(target=console_input).start()

while True:
    # wait for a client to connect
    conn, addr = sock.accept()
    # add the connection to the list of connections
    connections.append(conn)
    # start a new thread to handle the client connection
    threading.Thread(target=handle_client, args=(conn, addr)).start()
