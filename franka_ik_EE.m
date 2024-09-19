% franka_ik_EE
% IK = franka_ik_EE(O_T_EE_array, q7, q_actual_array);
% Attempts to obtain 4 different joint configurations for a given pose of the end effector.
% 
% The input O_T_EE_array is the desired Cartesian pose, represented as a transformation matrix relative to the robot base frame, 
% stored in an array in column-major format. 
% q7 is the last joint angle as the redundant parameter, specified by the user in radians. 
% q_actual_array is the actual joint configuration. At singularity, the actual angle of the first joint will be assigned to the final output.
% 
% Original C++ code by Yanhao He, February 2020 (https://github.com/ffall007/franka_analytical_ik)
% Wrapper by Hugo Tadashi and Martin Schonger, November 2023
function IK = franka_ik_EE(O_T_EE_array, q7, q_actual_array)
    arguments
        O_T_EE_array (1,16) {mustBeNumeric}
        q7 (1,1) {mustBeNumeric}
        q_actual_array (1,7) {mustBeNumeric}
    end

    IK = franka_ik_EE_mex(O_T_EE_array, q7, q_actual_array);
end