
#include "network.h"
#include "time_helper.h"
#include <algorithm>
#include <thread>
#include <assert.h>
#include <cstring>
#include <mutex>

#ifdef _MSC_VER
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#define HOSTENT hostent
#endif



#ifndef _WINSOCKAPI_
#define closesocket(x) close(x)
#define SOCKADDR_IN sockaddr_in
#define SOCKADDR sockaddr
#endif

static void net_connect_thread(NetConnecting* nc);

struct NetConnecting
{
	std::mutex mutex;
	SOCKET s = INVALID_SOCKET;
	std::string address;
	u16 port;
	bool verbose;
	bool keep_trying;
	bool cancel = false;
	bool thread_done = false;
};

NetConnecting* net_connect(const char* address, u16 port, bool verbose, bool keep_trying)
{
	NetConnecting* nc = new NetConnecting;
	nc->address = address;
	nc->port = port;
	nc->verbose = verbose;
	nc->keep_trying = keep_trying;
	std::thread thread(net_connect_thread, nc);
	thread.detach();
	return nc;
}

bool net_connect_is_done(NetConnecting* nc, SOCKET* out)
{
	nc->mutex.lock();
	if (nc->thread_done)
	{
		*out = nc->s;
		nc->s = INVALID_SOCKET;
		nc->mutex.unlock();
		return true;
	}
	else
	{
		*out = INVALID_SOCKET;
		nc->mutex.unlock();
		return false;
	}
}

void net_connect_close(NetConnecting* nc)
{
	nc->mutex.lock();
	if (nc->thread_done)
	{
		// In case we connected right before the user cancelled or the user
		// never picked the socket up with net_connect_is_done, we close the connection ourself.
		if (nc->s != INVALID_SOCKET)
			net_close_socket(nc->s);
		nc->mutex.unlock();
		delete nc;
	}
	else
	{
		// TODO: Ideally we would signal the blocking gethostbyname and connect functions
		// here to abort, but it seems that's not possible. At least on Linux.
		// Instead we just abandon the thread and let it finish on its own.
		nc->cancel = true;
		nc->mutex.unlock();
		// thread will delete nc when it is done.
	}
}

static void net_connect_thread(NetConnecting* nc)
{
	bool verbose = nc->verbose;

	SOCKADDR_IN addr;
	memset(&addr, 0, sizeof(addr));

	addr.sin_addr.s_addr = inet_addr(nc->address.c_str());
	addr.sin_port = htons(nc->port);
	addr.sin_family = AF_INET;
	if (addr.sin_addr.s_addr == INADDR_NONE)
	{
		if (verbose)
		{
			printf("Resolve host %s ", nc->address.c_str());
			fflush(stdout);
		}
		GetHostAgain:
		HOSTENT* he = gethostbyname(nc->address.c_str());
		if (!he && !nc->cancel)
		{
			if (verbose)
			{
				printf(".");
				fflush(stdout);
			}
			goto GetHostAgain;
		}
		if (verbose) printf("\n");
		if (he) addr.sin_addr.s_addr = *((u32*)he->h_addr);
	}

	if (verbose && !nc->cancel)
	{
		printf("Connect to %s : %i", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
		fflush(stdout);
	}
	
	SOCKET s = INVALID_SOCKET;
	if (!nc->cancel) s = socket(AF_INET, SOCK_STREAM, 0);
	if (s != INVALID_SOCKET)
	{
		while (true)
		{
			int r = connect(s, (SOCKADDR*)&addr, sizeof(SOCKADDR));
			if (r < 0)
			{
				if (nc->keep_trying && !nc->cancel)
				{
					if (verbose)
					{
						printf(".");
						fflush(stdout);
					}
					imprecise_sleep(0.5);
					continue;
				}
				else
				{
					net_close_socket(s);
					s = INVALID_SOCKET;
				}
			}
			break;
		}
	}
	if (verbose && s != INVALID_SOCKET) printf("\nConnected!\n");

	nc->mutex.lock();
	if (nc->cancel)
	{
		if (s != INVALID_SOCKET)
			net_close_socket(s);
		nc->mutex.unlock();
		// The calling thread gave up on us, delete it ourself.
		delete nc;
	}
	else
	{
		nc->s = s;
		nc->thread_done = true;
		nc->mutex.unlock();
	}
}

SOCKET net_connect_blocking(const char* address, u16 port, bool verbose, bool keep_trying, bool* running)
{
	NetConnecting* connecting = net_connect(address, port, verbose, keep_trying);
	if (!connecting) return INVALID_SOCKET;

	SOCKET client = INVALID_SOCKET;
	while (true)
	{
		if (net_connect_is_done(connecting, &client))
			break;
		if (running && !*running)
			break;
		imprecise_sleep(0.1);
	}
	net_connect_close(connecting);
	return client;
}

bool net_set_socket_non_blocking(SOCKET s)
{
	// enable non-blocking mode
#ifdef WIN32
	unsigned long mode = 1;
	int r = ioctlsocket(s, FIONBIO, &mode);
#else
	int r = fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
	return r == 0;
}

void net_close_socket(SOCKET s)
{
	closesocket(s);
}

int net_recv(SOCKET s, void* buffer, int size)
{
	int r = recv(s, (char*)buffer, size, 0);
	if (r <= 0 || r > size)
	{
#ifdef WIN32
		int rr = WSAGetLastError();
		if (rr == WSAEWOULDBLOCK)
#else
		int rr = errno;
		if (rr == EAGAIN || rr == EWOULDBLOCK)
#endif
		{
			return -1;
		}
		r = 0;
	}
	return r;
}

int net_send(SOCKET s, const void* buffer, int size)
{
	int r = send(s, (const char*)buffer, size, 0);
#ifdef WIN32
	if (r < 0 && WSAGetLastError() == WSAEWOULDBLOCK)
#else
	if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
#endif
	{
		r = -1;
		// internal send queue is full, return -1.
	}
	else if (r < 0 || r > size)
	{
		r = 0;
	}
	return r;
}

bool net_recv_all(SOCKET s, void* data, int len) 
{
	assert(len >= 0);
	while(len != 0)
	{
		int r = net_recv(s, (char*)data, (size_t)len);
		if (r <= 0)
		{
			return false;
		}
		len -= r;
		data = (char*)data + r;
	}
	return true;
}

bool net_send_all(SOCKET s, const void* data, int len)
{
	assert(len >= 0);
	while(len != 0)
	{
		int r = net_send(s, (const char*)data, (size_t)len);
		if (r <= 0)
		{
			return false;
		}
		len -= r;
		data = (char*)data + r;
	}
	return true;
}

SOCKET net_listen(u16 port, bool verbose, bool* running)
{
	SOCKET server = socket(AF_INET, SOCK_STREAM, 0);
	if (server == INVALID_SOCKET)
	{
		return INVALID_SOCKET;
	}

#ifndef _MSC_VER
	// This prevents: Address already in use
	// This is pretty common in Linux systems when starting the server right after closing it.
	// It's very annoying when testing things, so we include this only in Linux.
	// (But the flag also works on Windows)
	// see https://stackoverflow.com/questions/15198834/bind-failed-address-already-in-use
	int enable = 1;
	if (setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (const char*)&enable, sizeof(int)) < 0)
	{
		return INVALID_SOCKET;
	}
#endif

	SOCKADDR_IN addr;
	memset(&addr, 0, sizeof(SOCKADDR_IN));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = INADDR_ANY;

	for (int i = 0; ; i++)
	{
		int r = bind(server, (SOCKADDR*)&addr, sizeof(addr));
		if (r >= 0)
			break;
		if (verbose) printf("Port %d is blocked. Trying to open it...\n", ntohs(addr.sin_port));
		if (i >= 15 || (running && !*running))
		{
			closesocket(server);
			return INVALID_SOCKET;
		}
		imprecise_sleep(5);
		if (i >= 15 || (running && !*running))
		{
			closesocket(server);
			return INVALID_SOCKET;
		}
	}

	int r = listen(server, 1);
	if (r < 0)
	{
		closesocket(server);
		return INVALID_SOCKET;
	}
	return server;
}

SOCKET net_accept(SOCKET server, bool verbose)
{
#ifdef _MSC_VER
	int addrsize = sizeof(SOCKADDR_IN);
#else
	socklen_t addrsize = sizeof(SOCKADDR_IN);
#endif
	SOCKADDR_IN addr;
	SOCKET client = accept(server, (SOCKADDR*)&addr, &addrsize);
	if (client != INVALID_SOCKET && verbose)
	{
		printf("Connected with %s! ", inet_ntoa(addr.sin_addr));
		fflush(stdout);
	}
	return client;
}

SOCKET net_accept_blocking(SOCKET server, bool verbose, bool* running)
{
	while (true)
	{
		if (!*running)
		{
			return INVALID_SOCKET;
		}
		timeval timeout = {1, 0};
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(server, &fds);

		int r = select((int)server+1, &fds, NULL, NULL, &timeout);
		if (r < 0)
		{
			return INVALID_SOCKET;
		}
		if (FD_ISSET(server, &fds))
		{
			return net_accept(server, verbose);
		}
	}
}

bool net_can_read_without_blocking(SOCKET s)
{
	timeval tv;
	tv.tv_sec  = 0;
	tv.tv_usec = 0;

	fd_set fd_set_read;
	FD_ZERO(&fd_set_read);
	FD_SET(s, &fd_set_read);
	int r = select((int)s+1, &fd_set_read, 0, 0, &tv);

	return r < 0 || FD_ISSET(s, &fd_set_read);
}


void net_startup()
{
#ifdef WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 0), &wsa);
#endif
}

void net_shutdown()
{
#ifdef WIN32
	WSACleanup();
#endif
}
