#! /bin/bash
apt-get update
apt-get install -y git libuv1-dev libssl-dev gcc g++ cmake make vim
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
make install
cd ../..
ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
rm -r uWebSockets
