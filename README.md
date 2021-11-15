
kksdcmd
=======

"kksdcmd" is a free (opensource, GPLv3) industrial controller daemon with access to modbus, SPI via p44script

kksdcmd is based on a set of generic C++ utility classes called [**p44utils**](https://github.com/plan44/p44utils), which provides basic mechanisms for mainloop-based, nonblocking I/O driven automation daemons, including [libmodbus](https://www.libmodbus.org).
p44utils also support the p44script scripting language to use/configure many of p44utils' services and utilities directly from scripts, which makes prototyping and customisation easy.
p44utils is included as submodules into this project, a [heavily modified libmodbus](https://github.com/plan44/p44utils/tree/master/thirdparty/libmodbus) is included as part of p44utils.

License
-------

kksdcmd is licensed under the GPLv3 License (see COPYING).

*(c) 2021 by Lukas Zeller / [plan44.ch](http://www.plan44.ch/automation)*







