

sudo apt-get update
sudo apt-get install -y python3-pip

pip3 install git+ssh://git@github.com/MaimonLab/event_data_logging.git

pip3 install alicat
pip3 install pyserial
pip3 install ruamel.yaml
sudo adduser $USER dialout