# psmove_dk2_coreg
A simple tool to coregister the PSMove and DK2

# Warnings

1. There is no error checking.
2. Mac support does not work yet.
3. This is not intended for general use. It is intended only for a few specific programmers. I do not intend to make this user-friendly or to support it beyond getting basic functionality down.

# Instructions

1. Clone the repo.
2. mkdir build
3. cd build
4. cmake-gui ..
    * Only VS2013 supported right now.
5. Build
    * Open the VS2013 project
    * Change the target to Release W64
    * Build solution
6. Copy the DLLs from 3rdparty/psmoveapi/lib/win64 to wherever your .exe was made (probably build/Release)
7. Make sure your PSMove and DK2 are both in view of their cameras.
8. Run psmdk2.exe
    * Alternatively, if you want to output the data to a file, try `psmdk2 > output.csv`
9. Press & Hold the PSMove Move button to print data (to screen or file)
10. Press PSMove select button to quit.