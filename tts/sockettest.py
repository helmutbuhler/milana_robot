# echo-server.py

import socket
import os, sys
#import signal
import sys

ip = socket.gethostbyname("127.0.0.1")
port = 60124  # port to listen on (non-privileged ports are > 1023)
#import signal

#signal.signal(signal.SIGINT, signal.SIG_DFL);

if True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

	    # This prevents: Address already in use
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        s.bind((socket.gethostbyname("0.0.0.0"), port))
        s.listen()
        while True:
            print("Waiting for connection")
            conn, addr = s.accept()
            input_string = ""
            try:
                with conn:
                    print(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        input_string += data.decode('utf-8')
                        print(input_string)
                        splits = input_string.split('#')
                        if len(splits) > 1:
                            for i in range(len(splits)-1):
                                print("got:", splits[i])
                            input_string = splits[-1]
                        ts = "bbbbbbbbbbbbbbblubbbbbbbbbbablub"
                        conn.sendall(ts.encode('utf-8'))
                        print("send done")
            except ConnectionResetError:
                pass
            print("Closed.")

else:

    conn = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    conn.connect((ip,port))
    while True:
        s = input("what to send")

        print("start to send")
        conn.sendall(s.encode('utf-8'))
        print("done")
        data = conn.recv(1024)
        if not data:
            break
        stringdata = data.decode('utf-8')
        print("recv:", stringdata)
    print("Disconnected")