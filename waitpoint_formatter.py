import json
import os

# Define the input and output directories
input_dir = 'data/demonstrations/'
output_dir = 'data/demonstrations/compressed/'
output_dir_txt = 'data/demonstrations/compressed_txt/'

# Create the output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

# Function to process a single JSON file
def process_file(file_path, output_path):
    # Load the JSON file
    with open(file_path, 'r') as f:
        data = json.load(f)

    # print out all keys
    print(file_path)
    print(data.keys())

    objects = data["objects"]
    # go through all the objects and remove the second element (rotation)
    for obj in objects:
        data["objects"][obj] = data["objects"][obj]["position"]

    # Delete the demonstration key if the index is not divisible by 20
    length = len(data['demonstration']) 
    indices_to_delete = [i for i in range(length) if i % 2 != 0]

    # Delete indices in reverse order to avoid shifting issues
    for i in reversed(indices_to_delete):
        print(f"Deleting index {i}, length is {length}")
        del data['demonstration'][i]

    # Save the data to a new JSON file
    with open(output_path, 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)

    
def process_file_to_txt(input_path, output_path):
    with open(input_path, 'r') as f:
        data = json.load(f)

    objects = data["objects"]
    # multiply by 100 and do to 0 dp and save as int
    for obj in objects:
        data["objects"][obj] = [int(x * 100) for x in data["objects"][obj]]

    waypoints = []
    for waypoint in data['demonstration']:
        xyz_position = waypoint['xyz_position']

        waypoints.append(xyz_position)

    with open(output_path, 'w') as txtfile:
        # write all the waypoints to the file in a single line
        txtfile.write(str(objects))
        # new line 
        txtfile.write('\n')

        txtfile.write(str(waypoints))

    

# Process each JSON file in the input directory
for filename in os.listdir(input_dir):
    if filename.endswith('.json'):
        input_file_path = os.path.join(input_dir, filename)
        output_file_path = os.path.join(output_dir, filename)
        output_file_path_txt = os.path.join(output_dir_txt, filename)
        # output_filepath to end with txt
        output_file_path_txt = output_file_path_txt[:-4] + 'txt'
        process_file(input_file_path, output_file_path)
        process_file_to_txt(output_file_path, output_file_path_txt)
        print(f"Processed {filename} and saved to {output_file_path}")
