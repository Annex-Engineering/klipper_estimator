# Klipper print time estimator

`klipper_estimator` is a tool for determining the time a print will take using
the Klipper firmware. Currently it provides the following modes:

  * `estimate` mode outputs detailed statistics about a print job
  * `post-process` mode can be used as a Slicer post-processing script, updating
    the gcode output file with corrected time estimates.
  * `dump-moves` mode dumps planning data for every move in a file

The estimation is done using an implementation of Klippers kinematics, but may
in some cases be slightly off due to rounding modes. If the timing is far
off(e.g. more than a minute over a >12 hour print), this is considered a bug. 

Note that currently delta kinematic limits are _not_ implemented.

## Getting `klipper_estimator`

Pre-built binaries are available for the latest release on the GitHub Releases
page. If you wish to build the tool yourself or poke around the source, see the
[Building section](#Building).

Binaries are provided for Windows, Linux, Mac OS X, and Raspberry Pi targets.
On Linux and Mac OS X, ensure that you give the downloaded file executable
permissions. This can be done in the terminal as follows:
```
$ chmod +x klipper_estimator
```
Change the filename (last parameter) to match the downloaded file.

For Arch Linux, an AUR package
[`klipper_estimator`](https://aur.archlinux.org/packages/klipper-estimator) is
available, courtesy of Wilhelm Schuster. Thanks!

## Usage

Basic usage info can be found by running `klipper_estimator` with no arguments.

### Configuration

In order to provide accurate times, `klipper_estimator` needs printer settings
including maximum velocity, acceleration, etc. It can take these either from a
config file(`--config_file` option) or grab them directly from Moonraker(using
the `--config_moonraker_url` option). Note that the Klipper configuration files
cannot be used directly.

To experiment with settings, one can use the `dump-config` command together with
`--config_moonraker_url` to generate a config file based on the current printer
settings. The config file can then be modified and used as input for the other
commands.

To dump a config, use e.g.:
```
$ ./klipper_estimator --config_moonraker_url http://192.168.0.21 dump-config > config.json
```

The config file format is Json5 and thus allows normal JSON with some
extensions(see https://json5.org/).

After generating a config, one can use this in other commands like so:
```
$ ./klipper_estimator --config_file config.json estimate ...
```

### `estimate` mode

Estimation mode is useful for determining statistics about a print, in order to
optimize print times. It gives a high level summary.

Basic usage:
```
$ ./klipper_estimator [config options] estimate ~/3DBenchy.data
Sequences:
 Run 0:
  Total moves: 42876
  Total distance: 73313.01640025008
  Total extrude distance: 3407.877500000097
  Minimal time: 1h29m9.948s (5349.947936969622)
  Average flow: 1.5321468696371368 mm3/s
  Phases:
    Acceleration: 27m4.291s
    Cruise:       35m1.116s
    Deceleration: 27m4.291s
  Moves:
  Layer times:
         0 => 2.536s
         ... [some lines omitted for brevity]
        48 => 4.834s
  Kind times:
   4m23.463s            => FILL
   2.639s               => Other
   18m0.185s            => SOLID-FILL
   28m29.706s           => WALL-INNER
   38m13.706s           => WALL-OUTER
```

The calculations are done based only on the commands found in the file, with no
regards for macro expansions. This means that `print_start` type macros will
count as zero seconds, as well heat up times, homing, etc. Therefore the time
output should be considered a "minimal time", assuming these extra factors take
no time.

### `post-process` mode

In `post-process` mode `klipper_estimator` directly modifies the filename passed
in in-place, updating time estimations in the file.

When using `klipper_estimator` in `post-process` mode, simply add a
post-processing script in your slicer like so:
```
/path/to/klipper_estimator --config_moonraker_url http://192.168.0.21 post-process
```
Change the path and config options to fit your situation.

Currently the following slicers are supported:

  * PrusaSlicer
  * SuperSlicer
  * ideaMaker
  * Cura

In PrusaSlicer and SuperSlicer `Post-processing scripts` are set in `Output
Options` under `Print Settings`:

![PrusaSlicer and SuperSlicer Post-processing scripts option](/doc/post_processing_psss.png)

Note that ideaMaker does not have support for post-processing scripts, and thus
cannot automatically run `klipper_estimator` on export.

For Cura, use the script in `compat/CuraPostProcessing/`.

### `dump-moves` mode

The `dump-moves` mode is used like `estimate` mode, but instead of providing a
summary, move planning data is dumped for every move.

### Accurately estimating `PRINT_START`/`PRINT_END` macros

Klipper macros can perform arbitrarily complex operations. `klipper_estimator`
has no hope of estimating how long these will take, as the Jinja templates can
access any state of the read printer. However it is often the case that the
amount of print time actually spent within the macro is constant. A prime
example of this is print start macros. The macro may execute homing and heating
commands, but the print timer does not start until the first material is
extruded. This generally happens when the prime line is started.

This gives rise to an offset in print time that we cannot estimate, but the user
can easily measure it after a print is over.

To compensate for this, `klipper_estimator` understands the following gcode
comment(generally syntax followed by some examples):

```
; ESTIMATOR_ADD_TIME <duration> [description]
; E.g.:
; ESTIMATOR_ADD_TIME 21
; ESTIMATOR_ADD_TIME 21 Print start
```

When `klipper_estimator` encounters a comment with this format, it will add the
requested duration to the total print time. The time will also be tracked as a
"move kind", if the description field is given.

Note that only the upper-case string `ESTIMATOR_ADD_TIME`, on a separate comment
line, will trigger this behaviour. Any whitespace between the `;` and `E`
characters will however be ignored.

The intended usage of this functionality is for print start macros, when
executed by the slicer. E.g. in PrusaSlicer or SuperSlicer, one might set their
print start gcode like this:

```
; ESTIMATOR_ADD_TIME 20 Prime line
print_start extruder=[first_layer_temperature] bed=[first_layer_bed_temperature]
```

## Building

`klipper_estimator` is written in Rust. Version 1.58 or newer is required to
compile the tool. Assuming a Rust toolchain is installed, along with git, one
can build `klipper_estimator` by running:

```
$ git clone https://github.com/dalegaard/klipper_estimator.git
$ cd klipper_estimator
$ cargo build --release
// Resulting binary will be at `target/release/klipper_estimator`(.exe on Windows)
```

## Acknowledgements

This project is in no way endorsed by the Klipper project. Please do not direct
any support requests to the Klipper project.

  * [Klipper](https://www.klipper3d.org/) by [Kevin O'Connor](https://www.patreon.com/koconnor)
  * [Moonraker](https://github.com/Arksine/moonraker) by Arksine
