#!/usr/bin/python3

import os
import re

path = 'src'
all_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
c_files = [f for f in all_files if re.match('.*\.c', f)]
c_files.sort()

for i, fname in enumerate(c_files):
    fname = fname[:len(fname) - 2] + '_fileid.h'
    with open(os.path.join(path, fname), 'w') as f:
        f.write('#define FILE_ID {}\n'.format(i))
