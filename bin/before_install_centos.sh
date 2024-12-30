#! /bin/bash

# For legacy versions, use my patched repository
repo="https://github.com/frankaemika/libfranka.git"
if [[ "$LIBFRANKA_VER" == "0.7.1" || "$LIBFRANKA_VER" == "0.8.0" ]]; then
  repo="https://github.com/JeanElsner/libfranka.git"
fi

git clone --recursive $repo
cd libfranka
git checkout $LIBFRANKA_VER
git submodule update
cmake -B build . -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX ..
cmake --build build --target install

repo="https://github.com/pantor/ruckig"
git clone --recursive $repo
cd ruckig
git checkout beb713afb4d2fac7b43bb75ae2af21c897701c09
git submodule update
mkdir build && cd build
cmake -B build . -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX ..
cmake --build build --target install
