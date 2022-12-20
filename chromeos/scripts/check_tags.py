#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Run presubmit checks for subject lines

This script is intended to be used by `repo`. For more information, please see
https://chromium.googlesource.com/chromiumos/repohooks/+/refs/heads/main/README.md.

Enforces the following rules:
  - Each uploaded patch must have valid subject line tag
    - ChromeOS only commits must not have FROMLIST:, FROMGIT:,
      or UPSTREAM: tags
    - Patches with FROMLIST:, FROMGIT:, BACKPORT:, or UPSTREAM: tags
      must not include ChromeOS only files
"""

import os
import subprocess
import sys


TAG_MISSING_MSG = 'The subject of this patch does not include a valid tag'
BAD_TAG_MSG = 'Bad subject tag for patch touching ChromeOS specific files'

UPSTREAM_TAGS = ('FROMLIST:', 'UPSTREAM:', 'FROMGIT:', 'BACKPORT:')
OTHER_TAGS = ('CHROMIUM:', 'WIP:', 'TEST:', 'TEST-ONLY:', 'Revert', 'Reland',
              'FIXUP:', 'DO-NOT-SUBMIT')

CHROMEOS_FILES = ('chromeos/', 'OWNERS', 'PRESUBMIT.cfg', 'unblocked_terms.txt')


def repo_path():
    """Return path to repository"""
    return subprocess.check_output(
        ['git', 'rev-parse', '--show-toplevel'], encoding='utf-8').strip()


def includes_chromium(files):
    """Check if the CL modifies files that belong to ChromiumOS"""

    gitroot=repo_path()

    return any(os.path.relpath(f, gitroot).startswith(CHROMEOS_FILES)
               for f in files)


def check_commit(commit, files):
    """Inspect commit messages to validate subject line."""
    commit_message = subprocess.check_output(
        ['git', 'log', '-n', '1', '--format=%B', commit],
        encoding='utf-8',
    )

    if not commit_message.startswith(UPSTREAM_TAGS + OTHER_TAGS):
        print(TAG_MISSING_MSG, file=sys.stderr)
        return False

    if commit_message.startswith(UPSTREAM_TAGS) and includes_chromium(files):
        print(BAD_TAG_MSG, file=sys.stderr)
        return False

    return True


def main():
    """Main function."""

    presubmit_files = os.environ.get('PRESUBMIT_FILES')
    if presubmit_files is None:
        sys.exit('Need a value for PRESUBMIT_FILES')

    presubmit_commit = os.environ.get('PRESUBMIT_COMMIT')
    if presubmit_commit is None:
        sys.exit('Need a value for PRESUBMIT_COMMIT')

    if not check_commit(presubmit_commit, presubmit_files.splitlines()):
        sys.exit(1)


if __name__ == '__main__':
    main()
