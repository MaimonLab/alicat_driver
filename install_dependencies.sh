sudo apt-get update
sudo apt-get install -y python3-pip

pip3 install event_data_logging
pip3 install alicat
pip3 install pyserial
pip3 install ruamel.yaml

sudo adduser $USER dialout

echo ""
echo "--------------------------------------------------"
echo "Finished installing dependencies for alicat_driver"
echo "--------------------------------------------------"
echo ""