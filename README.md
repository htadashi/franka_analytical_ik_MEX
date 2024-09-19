# franka_analytical_ik_MEX

Unofficial MEX functions wrapping [franka_analytical_ik](https://github.com/ffall007/franka_analytical_ik), an analytical inverse kinematics solver for Franka Emika Panda originally developed by Yanhao He and Steven Liu.

In case of usage of this file, please cite Yanhao He and Steven Liu work according to the .bib file provided in the [README.md](https://github.com/ffall007/franka_analytical_ik/blob/main/README.md) of their repository.

To compile the MEX files, run one of the following commands in MATLAB:

- `mex franka_IK_EE_mex.cpp`
- `mex franka_IK_EE_CC_mex.cpp`

You can find an example of how to use the compiled functions in `demo.m`.

Tested under MATLAB R2022a (Windows version).
