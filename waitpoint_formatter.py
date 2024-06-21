import json
import os

# Define the input and output directories
input_dir = 'prompts/demonstrations/'
output_dir = 'prompts/demonstrations/compressed/'

# Create the output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

# Function to process a single JSON file
def process_file(file_path, output_path):
    # Load the JSON file
    with open(file_path, 'r') as f:
        data = json.load(f)

    # Delete the demonstration key if the index is not divisible by 20
    length = len(data['demonstration']) 
    indices_to_delete = [i for i in range(length) if i % 20 != 0]

    # Delete indices in reverse order to avoid shifting issues
    for i in reversed(indices_to_delete):
        print(f"Deleting index {i}, length is {length}")
        del data['demonstration'][i]

    # Save the data to a new JSON file
    with open(output_path, 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)

# Process each JSON file in the input directory
for filename in os.listdir(input_dir):
    if filename.endswith('.json'):
        input_file_path = os.path.join(input_dir, filename)
        output_file_path = os.path.join(output_dir, filename)
        process_file(input_file_path, output_file_path)
        print(f"Processed {filename} and saved to {output_file_path}")
