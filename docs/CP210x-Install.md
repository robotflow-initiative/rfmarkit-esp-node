# Build and install CP210x driver on Ubuntu

Running the following command

```bash
sudo apt-get install bison
sudo apt install linux-source
sudo apt-get install linux-headers-$(sudo uname --kernel-release)
make
sudo insmod cp210x.ko
```
