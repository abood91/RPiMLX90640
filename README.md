# Melexis90640 Driver RPi3

This driver was made for Melexis90640 specifically for raspberry pi 3 in order to use it first install the BCM2835:

```c++
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.55.tar.gz
tar xvfz bcm2835-1.55.tar.gz
./configure
make
sudo make install
```


 In order to detect if you have connected the Mlx90640 correctly install `sudo apt-get install i2c-tools` then run the command `i2cdetect -y 0` if error then run `i2cdetect -y 1`
 it should be showing this output:
 ```   0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
 00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
 30: -- -- -- -- 33 -- -- -- -- -- -- -- -- -- -- --
 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
 70: -- -- -- -- -- -- -- --
 ```

## Build: ##
After you have verified that melexis90640 connected correctly, now you can compile the driver code:

```c++
cmake .
make
```

## Run: ##
```c++
sudo ./MLX90640 10
```
the 10 in this command represent the timespan between each image in seconds.
