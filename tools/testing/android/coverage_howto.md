HOW TO COLLECT KERNEL CODE COVERAGE FROM A TRADEFED TEST RUN
============================================================


## Build and use a kernel with GCOV profile enabled
Build your kernel with the [`--gcov`](https://android.googlesource.com/kernel/build/+/refs/heads/main/kleaf/docs/gcov.md) option to enable
GCOV profiling from the kernel. This will also trigger the build to save the required *.gcno files needed to viewing the collected count data.

For example to build a Cuttlefish (CF) kernel with GCOV profiling enabled run:
```
$ bazel run --gcov //common-modules/virtual-device:virtual_device_x86_64_dist
```

## Run your test(s) using tradefed.sh with kernel coverage collection enabled
'tradefed.sh' can be used to run a number of different types of tests. Adding the appropriate coverage flags
to the tradefed call will trigger tradefed to take care of mounting debugfs, reseting the gcov counts prior
to test run, and the collection of gcov data files from debugfs after test completion.

These coverage arguments are:
```
--coverage --coverage-toolchain GCOV_KERNEL --auto-collect GCOV_KERNEL_COVERAGE
```

The following is a fulll example call running the selftests test suite that exists under the
'bazel-bin/common/testcases' directory. The artifact output has been redirected to 'tf-logs'
for easier referebce needed in the next step.
```
$ prebuilts/tradefed/filegroups/tradefed/tradefed.sh run commandAndExit \
    template/local_min --template:map test=suite/test_mapping_suite     \
    --include-filter selftests --tests-dir=bazel-bin/common/testcases/  \
    --primary-abi-only --log-file-path tf-logs                          \
    --coverage --coverage-toolchain GCOV_KERNEL                         \
    --auto-collect GCOV_KERNEL_COVERAGE
```

## Create an lcov tracefile out of the gcov tar artifact from test run
The previously mentioned tradefed run will produce a tar file artifact with a
name similar to <test>_kernel_coverage_*.tar.gz. This tar file is an archive of
all the gcov data files collected into debugfs/ from the profiled device. In
order to make it easier to work with this data, it needs to be converted to a
single lcov tracefile.

The script 'create-tracefile.py' facilitates this generation by handling the
required unpacking, file path corrections and ultimate 'lcov' call.

An example:
```
$ python3 common/tools/testing/android/bin/create-tracefile.py -t tf-logs/
```

This will create a local tracefile named 'cov.info'.


## Visualizing Results
With the created tracefile there a number of different ways to view coverage data from it.
### 1. Text Options
#### 1.1 Summary
```
$ lcov --summary --rc lcov_branch_coverage=1 cov.info
Reading tracefile cov.info_fix
Summary coverage rate:
  lines......: 6.0% (81646 of 1370811 lines)
  functions..: 9.6% (10285 of 107304 functions)
  branches...: 3.7% (28639 of 765538 branches)
```
#### 1.2 List
```
$ lcov --summary --rc lcov_branch_coverage=1 cov.info
Reading tracefile cov.info_fix
                                               |Lines      |Functions|Branches
Filename                                       |Rate    Num|Rate  Num|Rate   Num
================================================================================
[/usr/local/google/home/joefradley/dev/common-android-mainline-2/common/]
arch/x86/crypto/aesni-intel_glue.c             |23.9%   623|22.2%  36|15.0%  240
arch/x86/crypto/blake2s-glue.c                 |50.0%    28|50.0%   2|16.7%   30
arch/x86/crypto/chacha_glue.c                  | 0.0%   157| 0.0%  10| 0.0%   80
<truncated>
virt/lib/irqbypass.c                           | 0.0%   137| 0.0%   6| 0.0%   88
================================================================================
                                         Total:| 6.0% 1369k| 9.6%  0M| 3.7% 764k
```
### 2. HTML
The `lcov` tool `genhtml` is used to generate html. To create html with the default settings:

```
$ genhtml --branch-coverage -o html cov.info
```

The page can be viewed at `html\index.html`.

Options of interest:
 * `--frame`: Creates a left hand macro view in a source file view.
 * `--missed`: Helpful if you want to sort by what source is missing the most as opposed to the default coverage percentages.



