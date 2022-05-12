pkill -f gzclient > /dev/null 2>&1 &
pkill -f gzserver > /dev/null 2>&1 &
pkill -f roslaunch > /dev/null 2>&1 &
pkill -f setup.sh > /dev/null 2>&1 &
pkill -f Pos.py > /dev/null 2>&1 &
rm -f data.json > /dev/null 2>&1 &
exit 2
