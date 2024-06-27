#pragma once

enum Command
{
	C_Play_music,
	C_Stop_music,
	C_Play_finish,
	C_Up,
	C_Down,
	C_Left,
	C_Right,
	C_Lean_left,
	C_Lean_right,
	C_Wink,
	num_commands
};

char const*const command_names[num_commands] =
{
	"%play_music%",
	"%stop_music%",
	"%play_finish%",
	"%up%",
	"%down%",
	"%left%",
	"%right%",
	"%leanleft%",
	"%leanright%",
	"%wink%",
};

bool command_init();
bool command_update();
void command_close();

void command_add(Command command);
bool command_is_pending();
void command_reset();
