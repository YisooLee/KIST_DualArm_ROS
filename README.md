
## Prerequisities
> This project was developed in Ubuntu 18.04.
> Realtime controller is developed on Xenomai 3.0.9, Linux 4.14.134.

## Requirements

#### SOEM

```sh
git clone https://github.com/OpenEtherCATsociety/SOEM
cd SOEM
mkdir build
cd build
cmake ..
make all
sudo make install
sudo ldconfig
```

#### Eigen3

#### RBDL 2.6.0
> RBDL 3.0 (ORB ver is not working for this project.)
```sh
git clone https://github.com/rbdl/rbdl
cd rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON ..
make all
make install
sudo ldconfig
```

#### qpOASES