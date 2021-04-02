echo This script will set up the Antioch coater vision system to run at boot.
echo
echo system time: 
date
echo
echo removing incompatible build files.
rm -rf build
mkdir build
echo
echo regenerating build files
echo
cmake -B build
echo
echo recompiling program
echo
make -C build
echo
read -p "Enter module Number (1 is far from control panel, 9 is near control panel): "  n
echo module number $n
mod=s/nnn/$n/
echo $mod
sed $mod 'ACS.service' > 'ACS_new.service'
echo
read -p "Enter Camera serial number: " s
echo camera serial number: $s
ser=s/sss/$s/
sed -i $ser 'ACS_new.service'
echo
sudo mv ACS_new.service /lib/systemd/system/ACS_new.service
systemctl daemon-reload
sudo systemctl enable ACS_new.service
sudo systemctl start ACS_new.service
echo 'setup is complete'