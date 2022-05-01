echo -e "\033[34m  _   ___               ___             \033[0m\n\033[34m | | / (_)__ __ _____ _/ (_)__ ___  ____\033[0m\n\033[34m | |/ / (_-</ // / _ \`/ / (_-</ _ \/ __/\033[0m\n\033[34m |___/_/___/\_,_/\_,_/_/_/___/\___/_/   \033[0m\n"
if  [ $1 = "-d" -o $1 = "--debug" ]; then
    echo -e "\033[45m{•̃_•̃} DEBUG {•̃_•̃}\033[0m"
    python3 RoadMap_Visualisor/MainWindow.py -d
else
    python3 RoadMap_Visualisor/MainWindow.py
fi
./quit.sh
echo -e "\033[44mQuitting Visualisor (shutting down subprocess)\033[0m"
