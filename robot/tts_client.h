#pragma once
bool tts_client_init();
bool tts_client_update();
void tts_client_close();
void tts_play_finish();
void tts_say_text(const char* text);
