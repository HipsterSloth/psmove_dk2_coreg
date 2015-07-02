# psmove_dk2_coreg
A simple tool to coregister the PSMove and DK2. Currently, all it does is get the position and orientation data straight from the APIs.

# Warnings

1. There is no error checking.
1. This is not intended for general use. It is intended only for a few specific programmers. I do not intend to make this user-friendly or to support it beyond getting basic functionality down.

# Instructions

1. Clone the repo.
1. mkdir build
1. cd build
1. Configure
    * Windows: `cmake .. -G "Visual Studio 12 2013 Win64"`
    * Mac: `cmake ..`
1. Build
    * Windows
        * Open the VS2013 project
        * Change the target to Release W64
        * Build solution
    * Mac
        * `make`
1. Copy the shared objects from 3rdparty/psmoveapi/lib to wherever your executable was made (e.g., Windows `psmove_dk2_coreg\build\Release`
1 Make sure your PSMove and DK2 are both in view of their cameras.
1. Run psmdk2 executable
1. Attach the PSMove to the DK2.
1. Press & Hold the PSMove Move button to sample locations while moving the attached devices through their workspace.
1. After 300 samples it will write the calculate pseye -> DK2_camera transform to a csv file in the home/.psmoveapi directory.

# Testing the result in Matlab

1. Copy `<home>/.psmoveapi/output_camerapose.csv` and `<home>/.psmoveapi/output.txt` to `psmove_dk2_coreg\matlab`
2. Run `matlab/test.m` to visualize the result of the coregistration.
