#!/usr/bin/env python
import os
import sys
def main(args):

    # Change dash to bash
    print >> sys.stderr, "Change Dash to bash"
    status = os.system("sudo echo 'dash dash/sh boolean false' | sudo debconf-set-selections")
    status = os.system("sudo DEBIAN_FRONTEND=noninteractive dpkg-reconfigure dash")

if __name__ == '__main__':
    main(sys.argv)

