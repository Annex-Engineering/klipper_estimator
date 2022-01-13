# Klipper print time estimator

`klipper_estimator` is a tool for determining the time a print will take using
the Klipper firmware. Currently it provides two modes:

  * `estimate` mode outputs detailed statistics about a print job
  * `post-process` mode can be used as a Slicer post-processing script, updating
    the gcode output file with corrected time estimates.

The estimation is done using an implementation of Klippers kinematics, but may
in some cases be slightly off due to rounding modes. If the timing is far off,
this is considered a bug. 

Note that currently delta kinematic limits are _not_ implemented.

## Usage

Basic usage info can be found by running `klipper_estimator` with no arguments.

### Configuration

In order to provide accurate times, `klipper_estimator` needs printer settings
including maximum velocity, acceleration, etc. It can take these either from a
config file(`--config_file` option) or grab them directly from Moonraker(using
the `--config_moonraker_url` option).

To experiment with settings, one can use the `dump-config` command together with
`--config_moonraker_url` to generate a config file based on the current printer
settings. The config file can then be modified and used as input for the other
commands.

To dump a config, use e.g.:
```
$ ./klipper_estimator --config_moonraker_url http://192.168.0.21 dump-config
{
  ...
}
```

The config file format is Hjson and thus allows normal JSON with some
extensions(see https://hjson.github.io/).

After generating a config, one can use this in other commands like so:
```
$ ./klipper_estimator --config_file config.json estimate ...
```

### `estimate` mode

Estimation mode is useful for determining statistics about a print, in order to
optimize print times. It gives a high level summary, and can optionally also
give a detailed output of every single move when using the `--dump_moves`
option.

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

These calculations are done without regard for prime lines, printer heat up,
etc. and therefore is the "minimal time" the print could take assuming
everything else took no time.

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

## Building

`klipper_estimator` is written in Rust. Assuming a Rust toolchain is installed,
along with git, one can build `klipper_estimator` by running:

```
$ git clone https://github.com/dalegaard/klipper_estimator.git
$ cd klipper_estimator
$ cargo build --release
// Resulting binary will be at `target/release/klipper_estimator`(.exe on Windows)
```

For Linux and Windows binaries are provided in each release.
