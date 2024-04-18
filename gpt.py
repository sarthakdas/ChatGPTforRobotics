from openai import OpenAI

# TODO USE .ENV FILE

# client = OpenAI()


completion = client.chat.completions.create(
  model="gpt-3.5-turbo",
  messages=[
{"role": "system", "content": "you are a geometry planner, output coordinates that center around the users requested shape. DO NOT OUTPUT ANYRHING OTHER THEN THE COORDINATES"},
    {"role": "user", "content": "a square that is 5cm by 5cm"}
  ]
)

print(completion.choices[0].message)