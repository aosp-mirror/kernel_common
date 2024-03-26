#!/usr/bin/python3
# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2024 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This utility generates a single lcov tracefile from a gcov tar file."""

import argparse
import fnmatch
import glob
import json
import logging
import os
import pathlib
import shutil
import sys
import tarfile


LCOV = "lcov"

EXCLUDED_FILES = [
    "*/security/selinux/av_permissions.h",
    "*/security/selinux/flask.h",
]

# TODO(b/330365583) These gcno files are not currently preserved.
OMITTED_GCNO_FILES = [
    "common-modules/virtual-device/v4l2loopback/v4l2loopback.gcno",
    "common-modules/virtual-device/goldfish_drivers/goldfish_sync.gcno",
    "common-modules/virtual-device/goldfish_drivers/goldfish_pipe.gcno",
    "common-modules/virtual-device/goldfish_drivers/goldfish_address_space.gcno",
]


def generate_lcov_tracefile(
    gcov_dir: str,
    kernel_source: str,
    llvm_gcov_sh_filename: str,
    llvm_gcov_sh: str,
    tracefile_filename: str,
    included_files: [],
) -> None:
  """Call lcov to create tracefile based on gcov data files.

  A temporay helper script, 'llvm_gcov.sh', is created that points to a
  prebuilt copy of llvm-cov.

  Args:
    gcov_dir: Directory that contains the extracted gcov data files as retreived
      from debugfs.
    kernel_source: Directory containing the kernel source same as what was used
      to build system under test.
    llvm_gcov_sh_filename: The full filename of temp llvm_gcov.sh helper script.
    llvm_gcov_sh: The contents of what should be put into llvm_gcov.sh.
    tracefile_filename: The name of tracefile to create.
    included_files: List of source file pattern to include in tracefile. Can be
      empty in which case include allo source.
  """

  exclude_args = " ".join([f'--exclude "{f}"' for f in EXCLUDED_FILES])
  include_args = (
      " ".join([f'--include "{f[0]}"' for f in included_files])
      if included_files is not None
      else ""
  )

  # Create helper script to be used during the `lcov` call
  file_path = pathlib.Path(llvm_gcov_sh_filename)
  file_path.parent.mkdir(parents=True, exist_ok=True)
  file_path.write_text(llvm_gcov_sh)
  os.chmod(llvm_gcov_sh_filename, 0o755)

  logging.info("Running lcov on %s", gcov_dir)
  lcov_cmd = (
      f"{LCOV} -q "
      "--ignore-errors=source "
      "--rc branch_coverage=1 "
      f"-b {kernel_source} "
      f"-d {gcov_dir} "
      f"--gcov-tool {llvm_gcov_sh_filename} "
      f"{exclude_args} "
      f"{include_args} "
      "--ignore-errors gcov,gcov,unused,unused "
      "--capture "
      f"-o {tracefile_filename} "
  )
  os.system(lcov_cmd)


def update_symlink_from_mapping(filepath: str, prefix_mappings: {}) -> bool:
  """Update symbolic link based on prefix mappings.

  It will attempt to update the given symbolic link based on the prefix
  mappings. For every "from" prefix that matches replace with the new "to"
  value. If the resulting path doesn't exist, try the next.

  Args:
    filepath: Path of symbolic link to update.
    prefix_mappings: A multimap where the key is the "from" prefix to match, and
      the value is an array of "to" values to attempt to replace with.

  Returns:
    True or false depending on the whether symbolic link was successfully
      updated to a new path that exists.
  """

  link_target = os.readlink(filepath)
  for old_prefix, new_prefix_list in prefix_mappings.items():
    for new_prefix in new_prefix_list:
      if link_target.startswith(old_prefix):
        new_target = os.path.abspath(
            link_target.replace(old_prefix, new_prefix)
        )
        if not os.path.exists(new_target):
          continue
        os.unlink(filepath)  # Remove the old symbolic link
        os.symlink(new_target, filepath)  # Create the updated link
        return True

  return False


def correct_symlinks_in_directory(directory: str, prefix_mappings: {}) -> None:
  """Recursively traverses a directory, updating symbolic links.

  Replaces 'old_prefix' in the link destination with 'new_prefix'.

  Args:
    directory: The root directory to traverse.
    prefix_mappings: Dictionary where the keys are the old prefixes and the
      values are the new prefixes
  """

  logging.info("Fixing up symbolic links in %s", directory)

  for root, _, files in os.walk(directory):
    for filename in files:
      filepath = os.path.join(root, filename)

      # TODO(b/330365583) These gcno files aren't preserved and can't be used.
      if any(
          ommitted_filename in filepath
          for ommitted_filename in OMITTED_GCNO_FILES
      ):
        os.remove(filepath)
        continue

      if os.path.islink(filepath):
        if not update_symlink_from_mapping(filepath, prefix_mappings):
          logging.error(
              "Unable to update link at %s with any prefix mappings: %s",
              filepath,
              prefix_mappings,
          )
          sys.exit(-1)


def find_most_recent_tarfile(path: str, pattern: str = "*.tar.gz") -> str:
  """Attempts to find a valid tar file given the location.

  If location is a directory finds the most recent tarfile or if location is a
  a valid tar file returns, if neither of these return None.

  Args:
    path (str): The path to either a tarfile or a directory.
    pattern (str, optional): Glob pattern for matching tarfiles. Defaults to
      "*.tar.gz".

  Returns:
      str: The path to the most recent tarfile found, or the original path
           if it was a valid tarfile. None if no matching tarfiles are found.
  """

  if os.path.isfile(path):
    if tarfile.is_tarfile(path):
      return path  # Path is a valid tarfile
    return None  # Path is a file but not a tar file

  if os.path.isdir(path):
    results = []
    for root, _, files in os.walk(path):
      for file in files:
        if fnmatch.fnmatch(file, pattern):
          full_path = os.path.join(root, file)
          results.append((full_path, os.path.getmtime(full_path)))

    if results:
      return max(results, key=lambda item: item[1])[
          0
      ]  # Return path of the most recent one
    else:
      return None  # No tarfiles found in the directory

  return None  # Path is neither a tarfile nor a directory


def make_absolute(path: str, base_dir: str) -> str:
  if os.path.isabs(path):
    return path

  return os.path.join(base_dir, path)


def append_slash(path: str) -> str:
  if path is not None and path[-1] != "/":
    path += "/"
  return path


def update_multimap_from_json(
    json_file: str, base_dir: str, result_multimap: {}
) -> None:
  """Reads 'to' and 'from' fields from a JSON file and updates a multimap.

  The multimap is implemented as a dictionary of lists allowing multiple 'to'
  values for each 'from' key.

  Args:
    json_file: The path to the JSON file.
    base_dir: Used if either of the 'to' or 'from' paths are relative to make
      them absolute by prepending this base_dir value.
    result_multimap: A multimap that is updated with every 'to' and 'from'
      found.

  Returns:
    The updated dictionary.
  """

  with open(json_file, "r") as file:
    data = json.load(file)

  for item in data:
    to_value = append_slash(item.get("to"))
    from_value = append_slash(item.get("from"))
    if to_value and from_value:
      to_value = make_absolute(to_value, base_dir)
      from_value = make_absolute(from_value, base_dir)
      if from_value not in result_multimap:
        result_multimap[from_value] = []
      result_multimap[from_value].append(to_value)


def read_gcno_mapping_files(search_dir: str, base_dir: str) -> {}:
  pattern = os.path.join(search_dir, "**", "dist", "gcno_mapping.*")
  result_multimap = {}
  for filepath in glob.iglob(pattern, recursive=False):
    logging.info("Reading %s", filepath)
    update_multimap_from_json(filepath, base_dir, result_multimap)
  return result_multimap


def get_testname_from_filename(file_path: str) -> str:
  filename = os.path.basename(file_path)
  if "_kernel_coverage" in filename:
    tmp = filename[: filename.find("_kernel_coverage")]
    testname = tmp[: tmp.rfind("_")]
  else:
    testname = filename[: filename.rfind("_")]
  return testname


def unpack_gcov_tar(file_path: str, output_dir: str) -> str:
  """Unpack the tar file into the specified directory.

  Args:
    file_path: The path of the tar file to be unpacked.
    output_dir: The root directory where the unpacked folder will reside.

  Returns:
    The path of extracted data.
  """

  testname = get_testname_from_filename(file_path)
  logging.info(
      "Unpacking %s for test %s...", os.path.basename(file_path), testname
  )

  test_dest_dir = os.path.join(output_dir, testname)
  os.makedirs(test_dest_dir, exist_ok=True)
  shutil.unpack_archive(file_path, test_dest_dir, "tar")
  return test_dest_dir


def get_parent_path(path: str, levels_up: int) -> str:
  """Goes up a specified number of levels from a given path.

  Args:
    path: The path to find desired ancestor.
    levels_up: The number of levels up to go.

  Returns:
    The desired ancestor of the given path.
  """
  p = pathlib.Path(path)
  for _ in range(levels_up):
    p = p.parent
  return str(p)


def get_kernel_repo_dir() -> str:
  this_script_full_path = os.path.abspath(__file__)

  # Assume this script is placed this many places from
  # the base kernel repo directory, e.g.:
  # kernel_repo/common/tools/testing/android/bin/<this_script>
  return get_parent_path(this_script_full_path, 6)


def build_config() -> {}:
  """Build configuration.

  Returns:
    A dictionary containing the configuration.
  """
  config = {}
  config["repo_dir"] = get_kernel_repo_dir()
  config["output_dir"] = f'{config["repo_dir"]}/out'
  config["output_cov_dir"] = f'{config["output_dir"]}/coverage'
  config["llvm_cov_prebuilt"] = (
      f'{config["repo_dir"]}/prebuilts/clang/host/linux-x86/'
      "llvm-binutils-stable/llvm-cov"
  )
  config["llvm_gcov_sh"] = (
      f'#!/bin/bash\nexec {config["llvm_cov_prebuilt"]} gcov "$@"'
  )
  config["llvm_gcov_sh_filename"] = (
      f'{config["output_cov_dir"]}/tmp/llvm-gcov.sh'
  )
  return config


def main() -> None:
  arg_parser = argparse.ArgumentParser(
      description="Generate lcov tracefiles from gcov file dumps"
  )

  arg_parser.add_argument(
      "-t",
      dest="tar_location",
      required=True,
      help=(
          "Either a path to a gcov tar file or a directory that contains gcov"
          " tar file(s). The gcov tar file is expected to be created from"
          " Tradefed. If a directory is used, will search the entire directory"
          " for files matching *_kernel_coverage*.tar.gz and select the most"
          " recent one."
      ),
  )
  arg_parser.add_argument(
      "-o",
      dest="out_file",
      required=False,
      help="Name of output tracefile generated. Default: cov.info",
      default="cov.info",
  )
  arg_parser.add_argument(
      "--include",
      action="append",
      nargs=1,
      required=False,
      help=(
          "File pattern of source file(s) to include in generated tracefile."
          " Multiple patterns can be specified by using multiple --include"
          " command line switches. If no includes are specified all source is"
          " included."
      ),
  )
  arg_parser.add_argument(
      "--verbose",
      action="store_true",
      default=False,
      help="Enable verbose logging",
  )

  args = arg_parser.parse_args()

  if args.verbose:
    logging.basicConfig(level=logging.DEBUG)
  else:
    logging.basicConfig(level=logging.WARNING)

  if shutil.which(LCOV) is None:
    logging.error(
        "%s is not found and is required for this script. Please install from:",
        LCOV,
    )
    logging.critical("       https://github.com/linux-test-project/lcov")
    sys.exit(-1)

  config = build_config()

  gcno_mappings = read_gcno_mapping_files(
      config["output_dir"], config["repo_dir"]
  )

  if not gcno_mappings:
    logging.error(
        "Either 'to' or 'from' fields not found in the %s.",
        config["output_dir"],
    )
    sys.exit(-1)

  tar_file = find_most_recent_tarfile(
      args.tar_location, pattern="*kernel_coverage_*.tar.gz"
  )
  if tar_file is None:
    logging.error("Unable to find a gcov tar under %s", args.tar_location)
    sys.exit(-1)

  gcov_dir = unpack_gcov_tar(tar_file, config["output_cov_dir"])
  correct_symlinks_in_directory(gcov_dir, gcno_mappings)
  generate_lcov_tracefile(
      gcov_dir,
      config["repo_dir"],
      config["llvm_gcov_sh_filename"],
      config["llvm_gcov_sh"],
      args.out_file,
      args.include,
  )


if __name__ == "__main__":
  main()
