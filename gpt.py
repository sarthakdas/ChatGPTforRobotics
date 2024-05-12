from openai import OpenAI

# TODO USE .ENV FILE

# client = OpenAI()

import os
import re
from openai import OpenAI


class OpenAIClient:
    def __init__(self, api_key_path="path/to/.env"):
        
        self.general_prompt = self.load_prompt("prompts/environment_description.txt")
        self.messages = [{"role": "system", "content": self.general_prompt}]
        self.conversation_history = []  # To store all responses from ChatGPT

    def load_prompt(self, path):
        with open(path, 'r') as file:
            return file.read()

    def get_user_input(self, prompt="Enter your question: "):
        return input(prompt)

    def generate_completion(self, user_input):
        self.messages.append({"role": "user", "content": user_input})
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=self.messages
        )
        
        chat_response = response.choices[0].message.content
        self.messages.append({"role": "assistant", "content": chat_response})
        self.conversation_history.append(chat_response)  # Store response
        return chat_response

    def extract_questions(self, response):
        questions = re.findall(r'<question>(.*?)</question>', response, re.DOTALL)
        return questions

    def write_code_snippet_to_file(self, code_snippet, file_path="prompts/response.py"):
        with open(file_path, 'w') as file:
            file.write(code_snippet)

    def process(self, objects=None):
        if objects:
            self.messages.append({"role": "system", "content": "you can currently see these objects in the sence:"+ str(objects)})
        user_input = self.get_user_input()
        response = self.generate_completion(user_input)
        print("ChatGPT:", response)  # Print the first response

        questions = self.extract_questions(response)
        for question in questions:
            answer = self.get_user_input("ChatGPT: " + question + "\nYou: ")
            response = self.generate_completion(answer)
            print("ChatGPT:", response)  # Print subsequent responses

        # Extract code snippet from the final response and save it
        code_snippet = self.extract_code_snippet(response)
        self.write_code_snippet_to_file(code_snippet)

    def extract_code_snippet(self, response):
        if "```python" in response:
            snippet = response.split("```python")[1]
            snippet = snippet.split("```")[0]
            return snippet.strip()
        return ""

    def print_conversation_history(self):
        for response in self.conversation_history:
            print(response)

if __name__ == "__main__":
    client = OpenAIClient("path/to/.env")
    client.process()
    client.print_conversation_history()  # Optionally print all ChatGPT responses at the end
