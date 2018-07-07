# MRPT-WEB

This repository contains server and client built on websocket transfer protocol and primarily used for remote procedure call.

<b>Dependencies:</b>
```bash
sudo apt-get install libargtable2-dev
```
Other than this MRPT is also required

<b>Build</b>
```build
mkdir build
cd build
cmake ..
make
```

<b>Use</b>
```bash
cd build/app/mrpt-ws-rpc
```
Example :
```bash
./mrpt-ws-rpc 127.0.0.1 5000
```

<b>Using the rawlog-viewer app</b>
```bash
cd build/app/rawlog-viewer
./rawlog-viewer 127.0.0.1 5000
```

The app is meant to be used as RPC server for [rawlog-web-ui](https://github.com/rachit173/rawlog-web-ui).
