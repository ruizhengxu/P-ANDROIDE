import sys
sys.path.append( '../' )
from slicer import slice

# python3 test.py circuit.json files
slice(sys.argv[1],sys.argv[2],colorBG="white",color1="black",color2="black")
