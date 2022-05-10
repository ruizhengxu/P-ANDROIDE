import json

def read_json(file):
    # Read file
    try:
        f = open(file, "r")
        data = json.load(f)
        f.close()
        return data
    except Exception as e:
        print("Cannot read selected file :", e.__doc__)