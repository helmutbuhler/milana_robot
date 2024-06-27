#pragma once


bool client_update();
bool client_init();
void client_close();

int client_get_connection_state();

struct MonitorData;

void on_new_monitor_data(MonitorData& md);
