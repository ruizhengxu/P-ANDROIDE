import json
from datetime import datetime

def read_json(file):
    # Read file
    try:
        f = open(file, "r")
        data = json.load(f)
        f.close()
        return data
    except Exception as e:
        print("Cannot read selected file :", e.__doc__)

def save_data_as_json(data, name):
    now = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    with open(name+now+".json", 'w') as f:
        json.dump(data, f)