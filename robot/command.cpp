// This handles some simple commands, that the LLM can trigger in its output.
// For example, when it outputs the following line:
// "Alright, fine. One. %wink% Two. %wink% Three. %wink% That's three winks in total"
// The robot will execute the wink commands while counting.
// That means it will execute the first wink command after saying "One". It will start saying "Two."
// only after the first wink command finished.
//
// To execute these commands over time, we have a simple state machine.

#include "command.h"
#include "../common/common.h"
#include "tts_client.h"
#include "main.h"
#include <vector>
#include <assert.h>

static std::vector<Command> command_queue;

void command_add(Command command)
{
	// The first two commands are handled by tts_client
	assert(command >= 2 && command < num_commands);
	command_queue.push_back(command);
}

bool command_is_pending()
{
	return !command_queue.empty();
}

bool command_init()
{
	command_queue.clear();
	md.command_timer = 0;
	md.command_running = -1;
	return true;
}

void command_reset()
{
	command_queue.clear();
	md.command_timer = 0;
	md.command_running = -1;
}

// returns true if the command is finished
bool handle_command(Command c)
{
	switch (c)
	{
	case C_Play_finish:
		tts_play_finish();
		return true;

	case C_Up:
		// hack: changing cd in robot code
		cd.common_servo_target_vel_user = -0.8f;
		if (md.command_timer > 1.0f || md.common_hip_pos_target < 0.5f) cd.common_servo_target_vel_user = 0;
		cd.common_leg_vel_target = -1;
		if (md.command_timer > 1.0f || md.common_leg_pos_target < 0.3f) cd.common_leg_vel_target = 0;
		return md.command_timer > 1.5f;
	case C_Down:
		// hack: changing cd in robot code
		cd.common_servo_target_vel_user = 0.8f;
		if (md.command_timer > 1.0f || md.common_hip_pos_target > 0.8f) cd.common_servo_target_vel_user = 0;
		cd.common_leg_vel_target = 1;
		if (md.command_timer > 1.0f || md.common_leg_pos_target > 0.9f) cd.common_leg_vel_target = 0;
		return md.command_timer > 1.5f;

	case C_Left:
	case C_Right:
		// hack: changing cd in robot code
		cd.balance_control_rotation_vel_user_target = (c == C_Left ? 0.8f : -0.8f);
		if (md.command_timer > 0.7f) cd.balance_control_rotation_vel_user_target = 0;
		return md.command_timer > 1.2f;

	case C_Lean_left:
	case C_Lean_right:
		md.leaning_target_angle_joystick = cd.leaning_max_angle * (c == C_Lean_left ? 1 : -1);
		if (md.command_timer > 1.0f) md.leaning_target_angle_joystick = 0;
		return md.command_timer > 1.5f;

	case C_Wink:
		static float original_r_arm_a, original_r_arm_b;
		switch (md.ik_winking_phase)
		{
		case 0:
			// init
			original_r_arm_a = md.ik_r_arm_a;
			original_r_arm_b = md.ik_r_arm_b;
			md.ik_winking_phase = 1;
			break;
		case 1:
			// move arm up
			md.ik_r_arm_a += 1.5f * md.delta_time;
			if (md.ik_r_arm_a >= to_rad(180))
			{
				md.ik_r_arm_a = to_rad(180);
				md.ik_winking_phase = 2;
			}
			break;
		case 2:
		case 4:
			// wink 1
			md.ik_r_arm_b += 3 * md.delta_time;
			if (md.ik_r_arm_b >= to_rad(40))
			{
				md.ik_r_arm_b = to_rad(40);
				md.ik_winking_phase++;
			}
			break;
		case 3:
		case 5:
			// wink 2
			md.ik_r_arm_b -= 3 * md.delta_time;
			if (md.ik_r_arm_b <= to_rad(-40))
			{
				md.ik_r_arm_b = to_rad(-40);
				md.ik_winking_phase++;
			}
			break;
		case 6:
			// go back to original arm_b
			md.ik_r_arm_b = move_towards(md.ik_r_arm_b, original_r_arm_b, 3*md.delta_time);
			if (md.ik_r_arm_b == original_r_arm_b)
			{
				md.ik_winking_phase = 7;
			}
			break;
		case 7:
			// go back to original arm_a
			md.ik_r_arm_a = move_towards(md.ik_r_arm_a, original_r_arm_a, 1.5f*md.delta_time);
			if (md.ik_r_arm_a == original_r_arm_a)
			{
				md.ik_winking_phase = 0;
				return true;
			}
			break;
		}
		return false;

	default:
		printf("Unknown command: %d!\n", (int)c);
		assert(0);
		return true;
	}
}

bool command_update()
{
	md.command_queue_size = command_queue.size();
	if (!md.tts_playback && command_queue.size())
	{
		md.command_running = (int)command_queue[0];
		if (md.command_timer == 0)
			printf("Start command: %s\n", command_names[command_queue[0]]);
		bool done = handle_command(command_queue[0]);
		md.command_timer += md.delta_time;
		if (done)
		{
			printf("Stop command: %s\n", command_names[command_queue[0]]);
			md.command_running = -1;
			command_queue.erase(command_queue.begin());
			md.command_timer = 0;
		}
	}
	else
	{
		md.command_running = -1;
		md.command_timer = 0;
	}
	return true;
}

void command_close()
{
}

