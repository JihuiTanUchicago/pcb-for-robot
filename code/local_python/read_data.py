import os
import time
import json
import requests

def read_last_line(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        if len(lines) > 3:
            # Return the last line after the first three header lines
            return lines[-1].strip()
        else:
            return None

def upload_data(force_signal_value):
    url = f"http://ec2-35-94-176-147.us-west-2.compute.amazonaws.com:5050/input-data?force_signal={force_signal_value}"
    try:
        response = requests.get(url)
        response.raise_for_status()
        print(f"Data uploaded successfully: {force_signal_value}")
    except requests.RequestException as e:
        print(f"HTTP Request failed: {e}")

def main():
    #C:\\Users\\PCB-class-study\\Desktop\\toddtan_pcb_final_project\\data.log
    file_path = 'C:\\Users\\PCB-class-study\\Desktop\\toddtan_pcb_final_project\\data.log'
    last_line = None
    last_line_count = 0

    while True:
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                lines = file.readlines()
                line_count = len(lines)
                
                # Check if there are more than three lines and the line count has changed
                if line_count > 3 and line_count != last_line_count:
                    last_line_count = line_count
                    new_last_line = lines[-1].strip()
                    
                    if new_last_line != last_line:
                        last_line = new_last_line
                        try:
                            data_dict = json.loads(last_line[4:])
                            force_signal_value = data_dict.get("force_signal")
                            if force_signal_value:
                                upload_data(force_signal_value)
                        except json.JSONDecodeError as e:
                            print(f"Failed to parse JSON from line: {last_line}, error: {e}")
        else:
            print(f"{file_path} does not exist. Waiting for the file to be created.")
        
        time.sleep(1)

if __name__ == "__main__":
    main()
