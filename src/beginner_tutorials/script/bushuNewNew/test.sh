sleep 3s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd /home/unionsys/Desktop;python follower_all.py;exec bash"  
sleep 2s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd /home/unionsys/Desktop;python follower0_300-1.py;exec bash"  


sleep 3s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd /home/unionsys/Desktop;python follower_all.py;exec bash"  
sleep 2s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd /home/unionsys/Desktop;python follower1_300-3.py;exec bash"  


sleep 3s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd /home/unionsys/Desktop;python follower_all.py;exec bash"  
sleep 2s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd /home/unionsys/Desktop;python follower2_300-5.py;exec bash"  

# No module named rospy问题：
# 解决:source一下就可以