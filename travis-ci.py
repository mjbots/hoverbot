#!/usr/bin/python3

import subprocess
import sys

def run_all(cmds):
    for cmd in [x for x in cmds.split('\n') if x.strip() != '']:
        print(">>>", cmd)
        subprocess.run(cmd, shell=True)


ALWAYS = """
sudo apt-get update
./install-packages --yes
"""


CMDS = [
    "./tools/bazel test //base/...",
    "./tools/bazel build @ffmpeg//...",
    "./tools/bazel build @opencv//...",
    "./tools/bazel test //...",
]

def main():
    run_all(ALWAYS)

    if len(sys.argv) < 2:
        [run_all(x) for x in CMDS]
    else:
        run_all(CMDS[int(sys.argv[1])])


if __name__ == '__main__':
    main()