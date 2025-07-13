cd /home/pi/depthaiC++/depthai-core-example-main/src
#lxterminal -e "python3 /home/pi/depthaiC++/depthai-core-example-main/src/auto_st.py" &
lxterminal -e "python3 /home/pi/depthaiC++/depthai-core-example-main/src/play_sound.py" &

cd /home/pi/depthaiC++/depthai-core-example-main/build
lxterminal -e "sudo -E nice -n -20 ./myapp"
#./myapp

