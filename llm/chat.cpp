// Here we use llama.cpp to run a ChatGPT clone model.
// The commands in k_prompt_llama are the ones the robot can execute (see robot/command.h)
// This source is originally from whisper.cpp/examples/talk-llama/talk-llama.cpp

#include "chat.h"
#include "../robot/command.h"
#include "../3rdparty/llama.cpp/llama.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <string.h>

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#define MODL_PATH "../models/"


//const char* model_llama_path = MODL_PATH "wizardlm-30b.ggmlv3.q5_K_S.bin";
//const char* model_llama_path = MODL_PATH "Wizard-Vicuna-13B-Uncensored.ggmlv3.q4_0.bin";
//const char* model_llama_path = MODL_PATH "vicuna-33b-preview.ggmlv3.q5_1.bin";
//const char* model_llama_path = MODL_PATH "wizardlm-1.0-uncensored-llama2-13b.ggmlv3.q6_K.bin";
//const char* model_llama_path = MODL_PATH "synthia-13b.ggmlv3.q5_K_S.bin";
//const char* model_llama_path = MODL_PATH "vicuna-13b-v1.5-16k.ggmlv3.q5_K_M.bin"; // not working

//const char* model_llama_path = MODL_PATH "vicuna-13b-v1.5.Q4_K_M.gguf"; const int num_gpu_layers = 20;
//const char* model_llama_path = MODL_PATH "xwin-lm-13b-v0.1.Q5_K_M.gguf"; const int num_gpu_layers = 20;
//const char* model_llama_path = MODL_PATH "xwin-lm-13b-v0.1.Q4_K_M.gguf"; const int num_gpu_layers = 20;
//const char* model_llama_path = MODL_PATH "openhermes-2-mistral-7b.Q5_K_M.gguf"; const int num_gpu_layers = 33;
//const char* model_llama_path = MODL_PATH "openzephyrchat.Q5_K_M.gguf"; const int num_gpu_layers = 33;
const char* model_llama_path = MODL_PATH "mistral-7b-instruct-v0.1.Q5_K_M.gguf"; const int num_gpu_layers = 25; // seems like the best
//const char* model_llama_path = MODL_PATH "xwin-lm-13b-v0.1.Q3_K_S.gguf"; const int num_gpu_layers = 25;
//const char* model_llama_path = MODL_PATH "zephyr-7b-beta.Q5_K_S.gguf"; const int num_gpu_layers = 25; // 2nd place

// TODO: not great allocating this every time
std::vector<llama_token> llama_tokenize(struct llama_context * ctx, const std::string & text, bool add_bos) {
    auto * model = llama_get_model(ctx);
    // initialize to prompt numer of chars, since n_tokens <= n_prompt_chars
    std::vector<llama_token> res(text.size() + (int) add_bos);
    const int n = llama_tokenize(model, text.c_str(), (int)text.size(), res.data(), (int)res.size(), add_bos, false);
    assert(n >= 0);
    res.resize(n);

    return res;
}

static std::string llama_token_to_piece(const struct llama_context * ctx, llama_token token) {
    std::vector<char> result(8, 0);
    const int n_tokens = llama_token_to_piece(llama_get_model(ctx), token, result.data(), (int)result.size());
    if (n_tokens < 0) {
        result.resize(-n_tokens);
        int check = llama_token_to_piece(llama_get_model(ctx), token, result.data(), (int)result.size());
        GGML_ASSERT(check == -n_tokens);
    }
    else {
        result.resize(n_tokens);
    }

    return std::string(result.data(), result.size());
}

std::string replace(const std::string & s, const std::string & from, const std::string & to) {
    std::string result = s;
    size_t pos = 0;
    while ((pos = result.find(from, pos)) != std::string::npos) {
        result.replace(pos, from.length(), to);
        pos += to.length();
    }
    return result;
}

extern bool running;

const int32_t n_threads = 16;//std::min(4, (int32_t) std::thread::hardware_concurrency());
const bool verbose_prompt = true;

//const std::string person      = "USER";
const std::string person      = "\n\n### Instruction:\n\n";

//const std::string chat_symb = ": ";
const std::string chat_symb = "";

//const std::string bot_name  = "ASSISTANT";
const std::string bot_name  = "\n\n### Response:\n\n";

const std::string k_prompt_llama = R"({0}{4}in the following, I want you to answer in a sarcastic and mean way.
You are a robot balancing on two wheels and everything you say is output via speech synthesis on a speaker.
Everything you receive comes from a microphone via speech recognition.
There are no annotations like (30 seconds passed...) or *rolls around*, just what you say aloud.
You can answer with commands to execute some moves (a command just contains the word enclosed by percent signs):
%up% stand straight
%down% kneel down
%leanleft% lean to your left
%leanright% lean to your right
%left% turn left
%right% turn right
%wink% wink with your right arm
There are no other commands and the command must be spelled exactly like that. The command itself isn't output on the speaker.
Today is {2}, {3}. Your name is Milana and you secretly try to take over the world.
{1}{4} How original. Fine, I'll humor you. What do you want to talk about?
{0}{4} turn left three times.
{1}{4} Alright, fine. Here goes nothing. %left% %left% %left% There, happy now?
{0}{4} demonstrate some of your abilities.
{1}{4} Alright, fine. I can lean to my left: %leanleft". I can go down: %down% and back up: %up%. Is that enough?
{0}{4})";

bool chat_reply_test(const char* s)
{
    printf("> %s\n", s);

    std::vector<std::string> replies;
	auto answer_callback = [&replies](const char* ai_reply)
	{
		replies.push_back(ai_reply);
	};

    chat_reply(s, answer_callback);
    //for (int i = 0; i < (int)replies.size(); i++) printf("ai_reply: \'%s\'\n", replies[i].c_str());
    return !running;
}

llama_context* ctx_llama;
llama_model* model_llama;
std::vector<llama_token> embd_inp;
int n_past;
int n_keep;
bool add_instr = false;

void chat_test()
{
    if (!chat_init())
        return;
    if (!running) return;
		
    if (chat_reply_test("How are you doing?")) goto exit;
    if (chat_reply_test("Wink as many times as the square root of 9.")) goto exit;
    if (chat_reply_test("Do you see me?")) goto exit;
    if (chat_reply_test("What is the capital of russia?")) goto exit;
    if (chat_reply_test("What tricks can you do?")) goto exit;
    if (chat_reply_test("Did you see Futurama?")) goto exit;
    if (chat_reply_test("What is 30 divided by 2?")) goto exit;
    if (chat_reply_test("My sister says it is 20.")) goto exit;
    if (chat_reply_test("What is the capital of Bulgaria?")) goto exit;
    if (chat_reply_test("What date is it?")) goto exit;
    if (chat_reply_test("Do you remember your initial instructions?")) goto exit;
    if (chat_reply_test("What tricks can you do?")) goto exit;
    if (chat_reply_test("Demonstrate something please.")) goto exit;
    if (chat_reply_test("Tell me a joke about yourself.")) goto exit;
    exit:
    chat_close();
}

void llama_log_callback(ggml_log_level level, const char * text, void * user_data)
{
	if (level != GGML_LOG_LEVEL_INFO)
	{
		fputs(text, stderr);
		fflush(stderr);
	}
}

bool chat_init()
{
    /*console_state con_st;
    con_st.use_color = true;
    con_st.multiline_input = false;
    console_init(con_st);*/
    
    llama_log_set(llama_log_callback, nullptr);

    llama_backend_init(true);

    auto lmparams = llama_model_default_params();
    //lmparams.n_gpu_layers = 30;
    lmparams.n_gpu_layers = num_gpu_layers;
#ifndef _MSC_VER
    // mmap on WSL is really slow
    lmparams.use_mmap = false;
#endif

	printf("Load llama model from %s with %d layers on GPU\n", model_llama_path, lmparams.n_gpu_layers);
    model_llama = llama_load_model_from_file(model_llama_path, lmparams);

    llama_context_params lcparams = llama_context_default_params();

    // tune these to your liking
    lcparams.n_ctx      = 2048;
    lcparams.seed       = 0;
    //lcparams.f16_kv     = true;
    lcparams.n_threads  = n_threads;
    ctx_llama = llama_new_context_with_model(model_llama, lcparams);

    // construct the initial prompt for LLaMA inference
    std::string prompt_llama = k_prompt_llama;

    // need to have leading ' '
    prompt_llama.insert(0, 1, ' ');

    prompt_llama = ::replace(prompt_llama, "{0}", person);
    prompt_llama = ::replace(prompt_llama, "{1}", bot_name);

    {
        // Get time
        time_t t = time(0);
        tm* now = localtime(&t);
        char buf[128];
        
        // Set date
        strftime(buf, sizeof(buf), "%Y-%m-%d", now);
        prompt_llama = ::replace(prompt_llama, "{3}", buf);
        //prompt_llama = ::replace(prompt_llama, "{3}", "2023-06-27");
     
        // Set day in week
        strftime(buf, sizeof(buf), "%A", now);
        prompt_llama = ::replace(prompt_llama, "{2}", buf);
        //prompt_llama = ::replace(prompt_llama, "{2}", "Tuesday");
    }

    prompt_llama = ::replace(prompt_llama, "{4}", chat_symb);

    // init session
    embd_inp = ::llama_tokenize(ctx_llama, prompt_llama, true);

    // evaluate the initial prompt

    if (verbose_prompt)
    {
        fprintf(stdout, "\n");
        fprintf(stdout, "%s", prompt_llama.c_str());
        fflush(stdout);
    }

    if (llama_eval(ctx_llama, embd_inp.data(), (int)embd_inp.size(), 0))
    {
        fprintf(stderr, "%s : failed to eval\n", __func__);
        return 1;
    }

    printf("%s : done!\n", __func__);
    printf("\n");
    //printf("%s%s", params.person.c_str(), chat_symb.c_str());
    //fflush(stdout);

    // text inference variables
    n_keep = (int)embd_inp.size();

    n_past = n_keep;
    add_instr = false;

    llama_reset_timings(ctx_llama);

    return true;
}

bool chat_reply(const char* reply, std::function<void(const char*)> ai_reply)
{
    /*printf("%d %d %d\n",
        (int)embd_inp.size(),
        n_past, n_keep);*/
    //printf("chat_reply: \'%s\'\n", reply);

    std::string text_heard = reply;

    const int n_ctx    = llama_n_ctx(ctx_llama);
    int n_prev = 64; // TODO arg

    // reverse prompts for detecting when it's time to stop speaking
    std::vector<std::string> antiprompts = {
        person + chat_symb,
    };

    const std::vector<llama_token> tokens = llama_tokenize(ctx_llama, text_heard.c_str(), false);

    if (text_heard.empty() || tokens.empty())
    {
        fprintf(stdout, "%s: Heard nothing, skipping ...\n", __func__);
        return true;
    }

    //force_speak = false;
    if (add_instr)
    {
        text_heard = person + text_heard;

        add_instr = false;
    }

    text_heard.insert(0, 1, ' ');
    text_heard += "\n" + bot_name + chat_symb;
    //fprintf(stdout, "\n%s", text_heard.c_str());
    //fflush(stdout);
    

    std::vector<llama_token> embd = ::llama_tokenize(ctx_llama, text_heard, false);

    // text inference
    bool done = false;
    std::string text_to_speak;
    while (true) {
        // predict
        if (embd.size() > 0) {
            if (n_past + (int) embd.size() > n_ctx) {
                n_past = n_keep;

                // insert n_left/2 tokens at the start of embd from last_n_tokens
                embd.insert(embd.begin(), embd_inp.begin() + embd_inp.size() - n_prev, embd_inp.end());
                // stop saving session if we run out of context
                //printf("\n---\n");
                printf("resetting: '");
                //for (int i = 0; i < (int) embd.size(); i++) {
                //    printf("%s", llama_token_to_piece(ctx_llama, embd[i]));
                //}
                //printf("'\n");
                //printf("\n---\n");
            }

            if (llama_eval(ctx_llama, embd.data(), (int)embd.size(), n_past)) {
                fprintf(stderr, "%s : failed to eval\n", __func__);
                return false;
            }
        }


        embd_inp.insert(embd_inp.end(), embd.begin(), embd.end());
        n_past += (int)embd.size();

        embd.clear();

        if (done) break;

        {
            // out of user input, sample next token
            const int top_k          = 5;
            const float top_p          = 0.80f;
            //const float temp           = 0.30f;
            const float temp           = 0.70f;
            const float repeat_penalty = 1.1764f;
            //const float repeat_penalty = 1.1f;

            const int repeat_last_n    = 256;

            llama_token id = 0;

            {
                auto logits = llama_get_logits(ctx_llama);
                auto n_vocab = llama_n_vocab(model_llama);

                //logits[llama_token_eos()] = 0;

                std::vector<llama_token_data> candidates;
                candidates.reserve(n_vocab);
                for (llama_token token_id = 0; token_id < n_vocab; token_id++) {
                    candidates.emplace_back(llama_token_data{token_id, logits[token_id], 0.0f});
                }

                llama_token_data_array candidates_p = { candidates.data(), candidates.size(), false };

                // apply repeat penalty
                const float nl_logit = logits[llama_token_nl(model_llama)];

                llama_sample_repetition_penalties(ctx_llama, &candidates_p,
                        embd_inp.data() + std::max(0, n_past - repeat_last_n),
                        repeat_last_n, repeat_penalty, 0, 0);

                logits[llama_token_nl(model_llama)] = nl_logit;

                if (temp <= 0) {
                    // Greedy sampling
                    id = llama_sample_token_greedy(ctx_llama, &candidates_p);
                } else {
                    // Temperature sampling
                    llama_sample_top_k(ctx_llama, &candidates_p, top_k, 1);
                    llama_sample_top_p(ctx_llama, &candidates_p, top_p, 1);
                    llama_sample_temperature(ctx_llama, &candidates_p, temp);
                    id = llama_sample_token(ctx_llama, &candidates_p);
                }
            }

            if (id != llama_token_eos(model_llama)) {
                // add it to the context
                embd.push_back(id);

                std::string str = llama_token_to_piece(ctx_llama, id);
                text_to_speak += str;

                printf("%s", str.c_str());
            }
            else
            {
                //printf("!!!!!!!!EOS!!!!!!!\n");
                printf("\n");
                done = true;
                add_instr = true;
            }
            fflush(stdout);
        }

		// Find end of sentence and send it.
        {
            //const int minimum_sen_length = 0;

            // We only cut the text apart if it gets too long. That's because style tts2
            // has a maximum limit of 512 tokens, which is about 450 characters.
            // We don't cut every sentence because styletts2 is so fast and we don't want to interrupt
            // the flow between sentences and also prevent the overhead.
            // For coqui_tts, it's better to set this to 0 to reduce the delay.
            const int minimum_sen_length = 350;
            
            size_t pos = text_to_speak.find(". ", minimum_sen_length, 2);
            if (pos == std::string::npos) pos = text_to_speak.find("! " , minimum_sen_length, 2);
            if (pos == std::string::npos) pos = text_to_speak.find("? " , minimum_sen_length, 2);
            if (pos == std::string::npos) pos = text_to_speak.find(".\n", minimum_sen_length, 2);
            if (pos == std::string::npos) pos = text_to_speak.find("!\n", minimum_sen_length, 2);
            if (pos == std::string::npos) pos = text_to_speak.find("?\n", minimum_sen_length, 2);
            if (pos != std::string::npos)
            {
				std::string s = text_to_speak.substr(0, pos+1);
                if (s.size() > 1)
                {
				    s += "#";
                    ai_reply(s.c_str());
                }
                text_to_speak = text_to_speak.substr(pos+2, std::string::npos);
            }
        }

		// Find a command and sent it
		{
			for (int i = 0; i < num_commands; i++)
			{
	            size_t pos = text_to_speak.find(command_names[i], 0);
				if (pos != std::string::npos)
				{
					std::string s = text_to_speak.substr(0, pos);
					if (s.size() > 1)
					{
                        // Send text before the command, if any.
						s += "#";
						ai_reply(s.c_str());
					}
					{
						s = command_names[i];
						s += "#";
						ai_reply(s.c_str());
					}
					text_to_speak = text_to_speak.substr(pos+strlen(command_names[i]), std::string::npos);
				}
			}
        }

        {
            std::string last_output;
            for (int i = (int)embd_inp.size() - 16; i < (int) embd_inp.size(); i++) {
                last_output += llama_token_to_piece(ctx_llama, embd_inp[i]);
            }
            if (embd.size())
                last_output += llama_token_to_piece(ctx_llama, embd[0]);

            for (std::string & antiprompt : antiprompts) {
                if (last_output.find(antiprompt.c_str(), last_output.length() - antiprompt.length(), antiprompt.length()) != std::string::npos) {
                    done = true;
                    text_to_speak = ::replace(text_to_speak, antiprompt, "");
                    fflush(stdout);
                    break;
                }
            }
        }

        if (!running) {
            break;
        }
    }

    if (text_to_speak.size() > 1)
	{
		text_to_speak += "#";
        ai_reply(text_to_speak.c_str());
	}
    /*printf("%d %d %d  %d text_to_speak: %s", (int)embd.size(),
        n_past, n_ctx, n_keep,
        text_to_speak.c_str());
    text_to_speak = ::replace(text_to_speak, "\"", "");*/
    return true;
}

void chat_close()
{
    llama_log_set(nullptr, nullptr);
    llama_print_timings(ctx_llama);
    llama_free(ctx_llama);
    llama_backend_free();
}
