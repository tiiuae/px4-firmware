#!/bin/sh
g++ -o mavproxy secure_mavlink_udp_proxy.cpp Crypto.cpp -L/home/kjyrinki/cryptopp -lcryptopp -lsodium
