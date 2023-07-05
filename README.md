
kksdcmd
=======

"kksdcmd" is a free (opensource, GPLv3) industrial controller daemon with access to modbus, SPI via p44script

kksdcmd is based on a set of generic C++ utility classes called [**p44utils**](https://github.com/plan44/p44utils), which provides basic mechanisms for mainloop-based, nonblocking I/O driven automation daemons, including [libmodbus](https://www.libmodbus.org).
p44utils also support the p44script scripting language to use/configure many of p44utils' services and utilities directly from scripts, which makes prototyping and customisation easy.
p44utils is included as submodules into this project, a [heavily modified libmodbus](https://github.com/plan44/p44utils/tree/master/thirdparty/libmodbus) is included as part of p44utils.

## How to build kksdcmd on Linux

### Prerequisites

```bash
# build tools
apt install git automake libtool autoconf g++ make
```

### Libraries

```bash
apt install libjson-c-dev libsqlite3-dev libboost-dev libi2c-dev libssl-dev
```

### Checkout sources

```bash
# clone the main repo
cd my_projects_dir
git clone https://github.com/plan44/kksdcmd.git
# enter the project dir
cd kksdcmd

# optionally: change branch
git checkout my_special_branch

# clone and checkout submodules
git submodule init
git submodule update
```

### build

```bash
cd my_projects_dir/kksdcmd
autoreconf -i
./configure
make kksdcmd
```
	
### quick test

./kksdcmd --help
	
Should output the usage text explaining the command line options.

License
-------

kksdcmd is licensed under the GPLv3 License (see COPYING).

*(c) 2021-2023 by Lukas Zeller / [plan44.ch](http://www.plan44.ch/automation)*









