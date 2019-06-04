trap 'python cleanup_gpio.py' INT
make && ./Shark
python cleanup_gpio.py
