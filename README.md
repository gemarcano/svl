# Sparkfun Variable Loader

This is a meson-ified repository of the Sparkfun Variable Loader bootloader for
the Artemis module.

## Dependencies

 - https://github.com/gemarcano/AmbiqSuiteSDK

In order for the libraries to be found, `pkgconf` must know where they are. The
special meson cross-file property `sys_root` is used for this, and the
`artemis` cross-file already has a shortcut for it-- it just needs a
variable to be overriden. To override a cross-file constant, you only need to
provide a second cross-file with that variable overriden. For example:

Contents of `my_cross`:
```
[constants]
prefix = '/home/gabriel/.local/redboard'
```

# Compiling and installing

```
mkdir build
cd build
# The `artemis` cross-file is assumed to be installed per recommendations from
# the `asimple` repository
meson setup --prefix [prefix-where-sdk-installed] --cross-file artemis --cross-file ../my_cross --buildtype release
meson compile
```

FIXME instructions on how to flash using asb.py

```
python asb.py --load-address-blob 0x20000 --magic-num 0xCB --version 0x0 --load-address-wired 0xC000 -i 6 --options 0x1 -v -o ./temp -port /dev/ttyUSB0 --bin build/svl.bin
```

# License

See the license file for details. In summary, most of this project is licensed
BSD 3-clause licensed as it is copied from The AmbiqSuiteSDK repository. Other
files are licensed Apache-2.0.
