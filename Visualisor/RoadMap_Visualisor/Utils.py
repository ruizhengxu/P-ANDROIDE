import json, os
from datetime import datetime
from pathlib import Path

def read_json(file):
    # Read file
    try:
        f = open(file, "r")
        data = json.load(f)
        f.close()
        return data
    except Exception as e:
        print("Cannot read selected file :", e.__doc__)

def save_history_as_json(data, name):
    now = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    path = os.path.abspath(__file__)
    path = str(Path(path).parent)
    path += "/histories/" + name.split(".")[0]
    file_name = "_"+now+".json"

    with open(path+file_name, 'w+') as f:
        json.dump(data, f)

def save_opt_as_json(data, name):
    now = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    path = os.path.abspath(__file__)
    path = str(Path(path).parent)
    path += "/optimals/" + name.split(".")[0]
    file_name = "_"+now+".json"

    with open(path+file_name, 'w+') as f:
        json.dump(data, f)