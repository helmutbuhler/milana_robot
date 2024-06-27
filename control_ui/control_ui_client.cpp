// Here we receive the MonitorData from the robot when it is connected
// and send the ControlData when the user is changing something.

#include "control_ui_client.h"
#include "control_ui.h"
#include "../common/network.h"

#include <algorithm>
#include <assert.h>
#include <cstring>


const char* host_address = "insert_hostname_or_ip_of_robot_here";

static SOCKET s = INVALID_SOCKET;
int cd_counter = -1;

NetConnecting* connecting = nullptr;
char receive_buffer[sizeof(MonitorData)];
int receive_buffer_pos = 0;

extern ControlData cd;

void client_disconnect()
{
	if (s != INVALID_SOCKET)
		net_close_socket(s);
	s = INVALID_SOCKET;
	receive_buffer_pos = 0;

	// Make sure we send controldata when we connect with robot.
	cd_counter = -1;
}

int client_get_connection_state()
{
	if (connecting)
		return 1;
	if (s != INVALID_SOCKET)
		return 2;
	return 0;
}

bool client_update()
{
	// start connecting when not connected
	if (s == INVALID_SOCKET && !connecting)
	{
		connecting = net_connect(host_address, port, true, true);
	}

	if (connecting && net_connect_is_done(connecting, &s))
	{
		net_connect_close(connecting);
		connecting = nullptr;
		if (s != INVALID_SOCKET)
		{
			int header[2];
			if (!net_recv(s, header, 8))
			{
				printf("Connection lost right away!\n");
				client_disconnect();
			}
			else if (header[0] != sizeof(MonitorData) ||
				header[1] != sizeof(ControlData))
			{
				printf("Server Client version mismatch!\n");
				client_disconnect();
			}
			else
				net_set_socket_non_blocking(s);
		}
	}

	while (s != INVALID_SOCKET)
	{
		int r = net_recv(s, receive_buffer+receive_buffer_pos, sizeof(MonitorData)-receive_buffer_pos);
		if (r > 0)
		{
			receive_buffer_pos += r;
			assert(receive_buffer_pos <= sizeof(MonitorData));
			if (receive_buffer_pos == sizeof(MonitorData))
			{
				MonitorData md;
				memcpy(&md, receive_buffer, sizeof(md));
				receive_buffer_pos = 0;
				on_new_monitor_data(md);
			}
		}
		else
		{
			if (r == 0) client_disconnect();
			break;
		}
	}

	if (s != INVALID_SOCKET && cd_counter != cd.counter)
	{
		// out control data changed since last frame, send it to the robot.

		int r = net_send(s, &cd, sizeof(cd));
		if (r == -1)
		{
			// Network is busy now, just wait and send it later
		}
		else if (r != sizeof(cd))
		{
			// This didn't ever happen in practice, but if
			// it does, we just reconnect and should be fine.
			printf("send fail %d\n", r);
			client_disconnect();
		}
		else
			cd_counter = cd.counter;
	}
	return true;
}

bool client_init()
{
	net_startup();
	cd_counter = -1;
	return true;
}

void client_close()
{
	client_disconnect();
	if (connecting)
	{
		net_connect_close(connecting);
		connecting = nullptr;
	}
	net_shutdown();
}
