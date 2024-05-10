# Sparkfun Variable Loader

This is a meson-ified repository of the Sparkfun Variable Loader bootloader for
the Artemis module. It has also been refactored and cleaned up, increasing
transfer rates and reducing code size. Thanks to the use of a patched
AmbiqSuiteSDK-3.0.0, it also supports being built with LTO to further reduce
size.

## Dependencies

 - https://github.com/gemarcano/AmbiqSuiteSDK

In order for the libraries to be found, `pkgconf` must know where they are. The
special meson cross-file property `sys_root` is used for this, and the
`artemis` cross-file already has a shortcut for it-- it just needs a
variable to be overriden. To override a cross-file constant, you only need to
provide a second cross-file with that variable overriden. For example:

Contents of `crossfile`:
```
[constants]
prefix = '/home/gabriel/.local/redboard'
```

# Configuring

```
mkdir build
cd build
# The `artemis` cross-file is assumed to be installed per recommendations from
# the `AmbiqSuiteSDK` repository
meson setup --prefix [prefix-where-sdk-installed] --cross-file artemis --cross-file ../crossfile --buildtype release
```

One of two GPIOs can be used to determine whether or not to skip the bootloader
completely, which happens when the pin is held low. GPIO 47 is the default, and
it is the same pin used by the Ambiq Secondary Secure Bootloader (SBBL) to also
skip itself if the pin is low. However, this pin is not pinned out on the
Sparkfun Redboard Artemis ATP, so an alternative of GPIO 42 is also offered.
These GPIOs can be selected via `meson configure`. For example:

```
meson configure -Dboot_gpio=42
```

# Compiling

```
meson compile
```

# Installing

Make sure to set the `tty` meson configuration to the path of the serial device
the bootloader is connected to. For example:

```
meson configure -Dtty=/dev/ttyUSB0
```

And finally,

```
make compile flash
```

# License

See the license file for details. In summary, most of this project is licensed
BSD 3-clause licensed as it is copied from The AmbiqSuiteSDK repository. Other
files are licensed Apache-2.0.
