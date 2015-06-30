# psmove_dk2_coreg
A simple tool to coregister the PSMove and DK2. Currently, all it does is get the position and orientation data straight from the APIs.

# Warnings

1. There is no error checking.
1. This is not intended for general use. It is intended only for a few specific programmers. I do not intend to make this user-friendly or to support it beyond getting basic functionality down.

# Instructions

1. Clone the repo.
1. mkdir build
1. cd build
1. cmake .. -G "Visual Studio 12 2013 Win64"
1. Build
    * Windows
        * Open the VS2013 project
        * Change the target to Release W64
        * Build solution
    * Mac
        * `make`
1. Copy the shared objects from 3rdparty/psmoveapi/lib to wherever your executable was made
1 Make sure your PSMove and DK2 are both in view of their cameras.
1. Run psmdk2 executable
    * Alternatively, if you want to output the data to a file, try `psmdk2 > output.txt`
1. Attach the PSMove to the DK2.
1. Press & Hold the PSMove Move button to print data (to screen or file) while you move the objects in the camera frustums.
1. Press PSMove select button to quit.

# Testing the result in Matlab

1. Edit the output.txt file.
    * Copy the camera position 7-values to `matlab\output_camerapose.txt`
    * Cleanup the output.txt file by deleting any text (except the column headers) and save in the matlab folder.
2. Run `matlab/test.m` to visualize the result of the coregistration.