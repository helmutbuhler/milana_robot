#pragma once
#include <functional>

bool chat_init();

// reply is the promt (from the user) for the llm
// ai_reply is then called with the answer from the llm.
// If it is long or contains commands, the answer is provided in chunks.
// (It's a callback to make the delay as small as possible)
bool chat_reply(const char* reply, std::function<void(const char*)> ai_reply);

void chat_close();
void chat_test();
