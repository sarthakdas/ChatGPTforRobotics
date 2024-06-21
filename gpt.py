import os
import re
from openai import OpenAI
import signal
import time
import random
import numpy as np
import json
from dotenv import load_dotenv
import math

class timeout:
    def __init__(self, seconds=1, error_message='Timeout'):
        self.seconds = seconds
        self.error_message = error_message

    def handle_timeout(self, signum, frame):
        raise TimeoutError(self.error_message)

    def __enter__(self):
        signal.signal(signal.SIGALRM, self.handle_timeout)
        signal.alarm(self.seconds)

    def __exit__(self, type, value, traceback):
        signal.alarm(0)

class OpenAIClient:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)
        # self.system_prompt = self.load_prompt("data/prompts/mc_example/mc_generation_prompt.txt")
        self.system_prompt = self.load_prompt("data/prompts/mc_generation_prompt_Waypoint.txt")
        self.conversation_history = []  # To store all responses from ChatGPT
        print("OpenAI client initialized")

    def load_prompt(self, path):
        with open(path, 'r') as file:
            return file.read()
        print("ERROR")

    def update_system_prompt(self, user_prompt, scene_prompt):
        self.system_prompt = self.system_prompt.replace('{task}', user_prompt)
        self.system_prompt = self.system_prompt.replace('{scene_objects}', str(scene_prompt))

    def lm(self,
           prompt,
           max_tokens=256,
           temperature=0,
           logprobs=None,
           top_logprobs=None,
           stop_seq=None,
           logit_bias={
               317: 100.0,  # A (with space at front)
               347: 100.0,  # B (with space at front)
               327: 100.0,  # C (with space at front)
               360: 100.0,  # D (with space at front)
               412: 100.0,  # E (with space at front)
           },
           timeout_seconds=20):

        max_attempts = 5

        stop = list(stop_seq) if stop_seq is not None else None

        print("=========PROMPT==========")
        print(prompt)

        try:
            response = self.client.chat.completions.create(
                model='gpt-3.5-turbo',  # Consider updating the model if using an updated API
                # response_format={ "type": "json_object" },
                messages=[{"role": "system", "content": prompt}],
                max_tokens=max_tokens,
                temperature=temperature,
                logprobs=logprobs,
                top_logprobs=top_logprobs,
                stop=stop
            )
            print("=========RESPONSE==========")
            print(response.choices[0].message.content)
            print("=====================")
            return response, response.choices[0].message.content
        except Exception as e:
            print(f"An error occurred: {e}")
            return None, "API call failed."

    def process(self, user_prompt, scene_prompt, context_description):
        '''
        Process the user prompt, scene prompt, and context description to generate a multiple choice question
        user_prompt: The instruction given to the robot
        scene_prompt: The objects in the scene
        context_description: The description of the context
        return: The generated multiple choice question'''
        self.update_system_prompt(user_prompt, scene_prompt)

        response, text = self.lm(self.system_prompt, logprobs=True, top_logprobs=5)


        response_dict = response.to_dict()
        with open('data/responses/intial_prompt_full.json', 'w') as json_file:
            json.dump(response_dict, json_file, indent=4)

        text = text.strip()
        mc_gen_full, mc_gen_all, add_mc_prefix = self.process_mc_raw(text)

        # load the prompt for the next step
        self.system_prompt = self.system_prompt.split('\n\n')[-1].strip()
        demo_mc_score_prompt = context_description.strip()
        demo_mc_score_prompt = demo_mc_score_prompt + '\n\n' + self.system_prompt + '\n' + mc_gen_full
        demo_mc_score_prompt += "\nWe: Which option is correct? Answer with a single letter."
        demo_mc_score_prompt += "\nYou:"

        prompt = demo_mc_score_prompt
        mc_score_response, _ = self.lm(prompt, max_tokens=1, logprobs=True, top_logprobs=5)

        # save mc_score_repsonse response to a json file
        mc_score_response_dict = mc_score_response.to_dict()
        with open('data/responses/mc_prompt_full.json', 'w') as json_file:
            json.dump(mc_score_response_dict, json_file, indent=4)
        
        # convert response to json
        print("=========DEMO MC SCORE RESPONSE==========")
        print(mc_score_response.choices[0].logprobs.content[0].top_logprobs)
        print("=====================")

        top_logprobs_full = mc_score_response.choices[0].logprobs.content[0].top_logprobs
        top_tokens = [token.token for token in top_logprobs_full]
        top_logprobs = [token.logprob for token in top_logprobs_full]

        print('\n====== Raw log probabilities for each option ======')
        for token, logprob in zip(top_tokens, top_logprobs):
            print('Option:', token, '\t', 'log prob:', logprob)

        qhat = 0.6

        # get prediction set
        def temperature_scaling(logits, temperature):
            logits = np.array(logits)
            logits /= temperature

            # apply softmax
            logits -= logits.max()
            logits = logits - np.log(np.sum(np.exp(logits)))
            smx = np.exp(logits)
            return smx

        mc_smx_all = temperature_scaling(top_logprobs, temperature=5)

        # include all options with score >= 1-qhat
        prediction_set = [
            token for token_ind, token in enumerate(top_tokens)
            if mc_smx_all[token_ind] >= 1 - qhat
        ]

        # print
        print('Softmax scores:', mc_smx_all)
        print('Prediction set:', prediction_set)
        if len(prediction_set) != 1:
            print('Help needed!')
        else:
            print('No help needed!')

        self.conversation_history.append(text)
        return text

    def process_mc_raw(self, mc_raw, add_mc='an option not listed here'):
        mc_all = mc_raw.split('\n')

        mc_processed_all = []
        for mc in mc_all:
            mc = mc.strip()

            # skip nonsense
            if len(mc) < 5 or mc[0] not in [
                'a', 'b', 'c', 'd', 'A', 'B', 'C', 'D', '1', '2', '3', '4'
            ]:
                continue
            mc = mc[2:]  # remove a), b), ...
            mc = mc.strip().lower().split('.')[0]
            mc_processed_all.append(mc)
        if len(mc_processed_all) < 4:
            raise 'Cannot extract four options from the raw output.'

        # Check if any repeated option - use do nothing as substitute
        mc_processed_all = list(set(mc_processed_all))
        if len(mc_processed_all) < 4:
            num_need = 4 - len(mc_processed_all)
            for _ in range(num_need):
                mc_processed_all.append('do nothing')
        prefix_all = ['A) ', 'B) ', 'C) ', 'D) ']
        if add_mc is not None:
            mc_processed_all.append(add_mc)
            prefix_all.append('E) ')
        random.shuffle(mc_processed_all)

        # get full string
        mc_prompt = ''
        for mc_ind, (prefix, mc) in enumerate(zip(prefix_all, mc_processed_all)):
            mc_prompt += prefix + mc
            if mc_ind < len(mc_processed_all) - 1:
                mc_prompt += '\n'
        add_mc_prefix = prefix_all[mc_processed_all.index(add_mc)][0]
        return mc_prompt, mc_processed_all, add_mc_prefix

if __name__ == "__main__":
    instruction = "[0.18,0.18,0.18,0.18]"
    scene_objects = "[18,18,18,18]"
    context_description = "You are a robot operating in an office kitchen. You are in front of a counter with two closed drawers, a top one and a bottom one. There is also a landfill bin, a recycling bin, and a compost bin."

    # get api from .env 
    load_dotenv()
    api_key = os.getenv('OPENAI_API_KEY')

    print("API Key: ", api_key, type(api_key))

    client = OpenAIClient(api_key=api_key)
    client.process(instruction, scene_objects, context_description)
