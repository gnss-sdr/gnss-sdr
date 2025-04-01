<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: GPL-3.0-or-later
)

[comment]: # (
SPDX-FileCopyrightText: 2020 Carles Fernandez-Prades <carles.fernandez@cttc.es>
)
<!-- prettier-ignore-end -->

## Benchmarks

This is a collection of implementation benchmarks based on
[Benchmark](https://github.com/google/benchmark). It is useful to developers for
assessing which is the fastest implementation of a given snippet of code in
their machine, and to keep a track about them over different machines and
compilers. The results may vary over different system architectures, compiler
versions, building modes, C++ standard version used by the compiler, etc.

If Benchmark is not found in your system, CMake will download, build and
statically link against that library for you at building time.

This collection is only built if the option `ENABLE_BENCHMARKS` is enabled when
configuring `gnss-sdr` project's building:

```
$ cmake -DENABLE_BENCHMARKS=ON ..
```

## Basic usage

Just execute the binaries generated in your `install` folder.

Example:

```
$ cd ../install
$ ./benchmark_copy
```

### Output formats

The benchmarks support multiple output formats. Use the
`--benchmark_format=<console|json|csv>` flag (or set the
`BENCHMARK_FORMAT=<console|json|csv>` environment variable) to set the format
type. `console` is the default format.

Write benchmark results to a file with the `--benchmark_out=<filename>` option.

Example:

```
$ ./benchmark_copy --benchmark_format=json --benchmark_out=benchmark_copy.json
```

### Statistics

The number of runs of each benchmark is specified globally by the
`--benchmark_repetitions` flag. When a benchmark is run more than once the mean,
median and standard deviation of the runs will be reported.

Example:

```
$ ./benchmark_copy --benchmark_repetitions=10
```
