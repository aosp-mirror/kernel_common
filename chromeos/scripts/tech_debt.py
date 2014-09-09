#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0

"""Does presubmit checks for kernel technical debt.

This is intended to be used by `repo`. For more information, please see
https://chromium.googlesource.com/chromiumos/repohooks/+/refs/heads/main/README.md.

Enforces the following rules:
  - If a patch is not from upstream, a UPSTREAM-TASK tag must be present
"""

import os
import re
import subprocess
import sys


TECH_DEBT_MSG = (
    "This patch is not fully upstream. Please open a tracking bug here: "
    "go/cros-kernel-technical-debt-bug and add a label UPSTREAM-TASK=b:XXXX "
    "referencing it. A member of the review committee will review the CL. "
    "Thank you."
)
TECH_DEBT_DUP_BUG_MSG = (
    "UPSTREAM-TASK bugs cannot be the same as those present in the BUG tag."
)


def is_chromium_only(files):
    """Check if the CL modifies files that only belong to ChromiumOS"""
    repo_path = subprocess.check_output(
        ["git", "rev-parse", "--show-toplevel"], encoding="utf-8"
    ).strip()

    for file in files:
        file = os.path.relpath(file, repo_path)
        if not file.startswith(
            ("chromeos/", "OWNERS", "PRESUBMIT.cfg", "unblocked_terms.txt")
        ):
            return False

    return True


def check_tech_debt(commit):
    """Inspect commit messages to validate tech_debt tags."""
    commit_message = subprocess.check_output(
        ["git", "log", "-n", "1", "--format=%B", commit],
        encoding="utf-8",
    )
    if commit_message.startswith(
        (
            "UPSTREAM:",
            "FROMGIT:",
            "WIP:",
            "TEST:",
            "TEST-ONLY:",
            "BACKPORT:",
            "Revert",
            "Reland",
        )
    ) and not commit_message.startswith("BACKPORT: FROMLIST"):
        return True

    bug_id = r"b:[0-9]{7,}"
    bugs_ids = rf"({bug_id}(?:[, ]+{bug_id})*)"

    bug_tag = rf"\nBUG={bugs_ids}+"
    upstream_task_tag = rf"\nUPSTREAM-TASK={bugs_ids}+"

    bug_line = re.findall(bug_tag, commit_message)
    upstream_task_line = re.findall(upstream_task_tag, commit_message)
    if upstream_task_line == []:
        print(TECH_DEBT_MSG, file=sys.stderr)
        return False

    bugs = set()
    for b in bug_line:
        bugs |= set(re.findall(bug_id, b))

    upstream_tasks = set()
    for b in upstream_task_line:
        upstream_tasks |= set(re.findall(bug_id, b))

    if set(bugs) & set(upstream_tasks):
        print(TECH_DEBT_DUP_BUG_MSG, file=sys.stderr)
        return False

    return True


def main():
    """Main function."""

    presubmit_files = os.environ.get("PRESUBMIT_FILES")
    if presubmit_files is None:
        sys.exit("Need a value for PRESUBMIT_FILES")

    presubmit_commit = os.environ.get("PRESUBMIT_COMMIT")
    if presubmit_commit is None:
        sys.exit("Need a value for PRESUBMIT_COMMIT")

    if is_chromium_only(presubmit_files.splitlines()):
        sys.exit(0)

    if not check_tech_debt(presubmit_commit):
        sys.exit(1)


if __name__ == "__main__":
    main()
