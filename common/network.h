// Some platform-independent TCP socket helper functions.
// It doesn't fully wrap the sockets, so you can still use the real socket functions alongside if you want to.
// UDP is not supported yet.
// These functions make it especially easy when dealing with non-blocking sockets, but blocking sockets
// are supported as well.

#pragma once
#include "helper.h"

#ifdef _MSC_VER
#ifdef _WIN64
typedef unsigned __int64 SOCKET;
#else
typedef unsigned int SOCKET;
#endif
#else
typedef int SOCKET;
#endif

#ifndef _WINSOCKAPI_
const SOCKET INVALID_SOCKET = -1;
#endif

// Async connect functionality (None of these functions block)
// This also does the address resolution if the address is a host name.
// To connect, you first call net_connect and then keep polling net_connect_is_done.
// Once you successfully connected (net_connect_is_done returned true) or you want to cancel connection,
// you call net_connect_close.
// If keep_trying is true, the connection process keeps trying to connect until it succeeds.
struct NetConnecting;
NetConnecting* net_connect(const char* address, u16 port, bool verbose, bool keep_trying);
bool net_connect_is_done(NetConnecting* nc, SOCKET* out); // return true if connection is done (either successfully or not). If successful, out will be set to the connected socket.
void net_connect_close(NetConnecting* nc);

// Block until connected. You can optionally cancel the connection by setting the supplied running variable to false.
// Returns INVALID_SOCKET if the connection failed or was canceled.
SOCKET net_connect_blocking(const char* address, u16 port, bool verbose, bool keep_trying, bool* running);


// All sockets created are blocking by default. Use this to make them non-blocking.
// This can also be used with server sockets, so that accept calls are non-blocking.
bool net_set_socket_non_blocking(SOCKET s);

void net_close_socket(SOCKET s);

// return how many bytes were sent/received.
// return 0 if the connection was closed on the other side or an error occured.
// return -1 if the socket is non-blocking and no data can be sent/received at this time.
int net_recv(SOCKET s, void* buffer, int size);
int net_send(SOCKET s, const void* buffer, int size);

// Block until all data is received/sent, or an error occured. Don't use these on non-blocking sockets.
// Return true iff no error occured.
bool net_recv_all(SOCKET s, void* data, int len); 
bool net_send_all(SOCKET s, const void* data, int len);

// Create server socket that can accept client connections.
// If the port is blocked, this will wait a bit for the port to open.
SOCKET net_listen(u16 port, bool verbose, bool* running);

// Wait until a client connects.
// If the server socket is non-blocking and no client connects, it returns INVALID_SOCKET.
SOCKET net_accept(SOCKET server, bool verbose);

// Wait until a client connects, or running is set to false. Don't use this on non-blocking sockets.
SOCKET net_accept_blocking(SOCKET server, bool verbose, bool* running);

bool net_can_read_without_blocking(SOCKET s);

void net_startup(); // Needs to be called on startup before calling any other function
void net_shutdown(); // This can be called when you are done using sockets.
