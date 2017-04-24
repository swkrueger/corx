# Odroid installation guide

## Prepare SD card
Dowload and install image: http://odroid.com/dokuwiki/doku.php?id=en:odroid_flashing_tools, http://odroid.com/dokuwiki/doku.php?id=en:c2_release_linux_ubuntu

## First boot
Change hostname
```
echo "odroid2" > /etc/hostname
vi /etc/hosts
```

Add non-root user and add to sudoers:
```
adduser odroid
usermod -aG sudo odroid
sudo passwd -l root
```

Password-less sudo:
```
echo "odroid   ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/odroid
```

Configure IP: Edit `/etc/network/interfaces.d/eth0`:
```
auto eth0
iface eth0 inet static
    address 192.168.2.102
    netmask 255.255.255.0
    gateway 192.168.2.1
    dns-nameservers 192.168.2.1
```

Configure timezone and locales:
```
sudo dpkg-reconfigure locales
sudo dpkg-reconfigure tzdata
```

## Install software
Update software:
```
sudo apt update && sudo apt upgrade
```

Install tools
```
sudo apt install mc vim rsync screen tmux htop git buffer parallel
```

Install build tools and fastcard dependencies:
```
sudo apt-get install build-essential cmake libvolk1-dev pkg-config
```

Compile RTL-SDR:
```
sudo apt-get build-dep rtl-sdr

mkdir -p ~/build
cd ~/build
git clone https://github.com/rtlsdrblog/rtl-sdr.git
cd rtl-sdr
mkdir build
cd build
cmake -DINSTALL_UDEV_RULES=ON ..
make
sudo make install
```

Install kernel module for armv8cyclecounter:
```
sudo apt-get install linux-headers-`uname -r`
sudo ln -s /usr/src/linux-headers-`uname -r`/ /lib/modules/`uname -r`/build

cd ~/build
git clone https://github.com/rdolbeau/enable_arm_pmu.git
cd enable_arm_pmu
sudo make runtests
sudo ./load-module
```

Load kernel module on boot:
```
sudo mkdir /lib/modules/`uname -r`/extra
sudo cp ko/enable_arm_pmu.ko /lib/modules/`uname -r`/extra
sudo depmod -a
sudo modprobe enable_arm_pmu
sudo sh -c 'echo "enable_arm_pmu" > /etc/modules-load.d/arm_pmu.conf'
```

TODO: use DKMS (see https://wiki.centos.org/HowTos/BuildingKernelModules )

Compile FFTW:
```
sudo apt-get build-dep fftw3

cd ~/build
wget http://www.fftw.org/fftw-3.3.6-pl1.tar.gz
tar xzvf fftw-3.3.6-pl1.tar.gz
cd fftw-3.3.6-pl1
./configure --enable-shared --enable-neon --enable-single --enable-armv8cyclecounter
make
sudo make install
```

Compile fastcard and fastdet:
```
cd ~
git clone https://github.com/swkrueger/Thrifty.git thrifty
git checkout -b corx
cd ~/thrifty/fastcard
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install

cd ~/thrifty/fastdet
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

Install rest of corx dependencies:
```
sudo apt install libgflags2v5 libgflags-dev
```

Install correlator dependencies:
```
sudo apt install python-numpy python-matplotlib
```

Install linrad?

## Install Corx
Setup repository access:

 - Generate SSH key: run `ssh-keygen`
 - Copy public key: `cat ~/.ssh/id_rsa.pub`
 - Add key to deployment keys

Cd to home directory and clone repository:
```
git clone git@bitbucket.org:nwu-corx/corx.git corx
```

Build
```
cd ~/corx
mkdir build && cd build
cmake ../src
make
```
