#! /bin/bash
cd $HOME  
git clone https://github.com/MapIV/RTKLIB.git
cd $HOME/RTKLIB     
git checkout rtklib_ros_bridge    

cd $HOME/RTKLIB/lib/iers/gcc/  
make   
cd $HOME/RTKLIB/app  
make   

cd $HOME/RTKLIB/app/rtkrcv/gcc  
sudo chmod 755 rtkstart.sh  
sudo chmod 755 rtkshut.sh  
