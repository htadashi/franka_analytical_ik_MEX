% clear all; clc;
robot = loadrobot("frankaEmikaPanda");

% Fix rigid body tree
body_hand = getBody(robot, "panda_hand");
body_hand.Joint.JointToParentTransform  % note the missing z offset (d in DH table)

panda_hand_joint = rigidBodyJoint("panda_hand_joint", "fixed");
setFixedTransform(panda_hand_joint, [0, 0, 0.1034, -pi/4], "mdh");  % note the swapped sign for pi/4 compared to He's paper (https://github.com/ffall007/franka_analytical_ik/blob/main/paper_preprint.pdf)
replaceJoint(robot, "panda_hand", panda_hand_joint);

body_hand = getBody(robot, "panda_hand");
body_hand.Joint.JointToParentTransform  % now correct


rnd_config_invalid = true;
while rnd_config_invalid
    q = randomConfiguration(robot);
    JointPosition = [q.JointPosition];
    if rad2deg(JointPosition(4)) < -27  % Check if q_a is below -27 degrees
        rnd_config_invalid = false;
    end
end

% Optionally plot robot pose
% figure;
% show(robot, q);

transform = getTransform(robot, q, "panda_link0", "panda_hand");
transform_inv = inv(transform)

target_q = JointPosition(1:7)

computed_q = franka_ik_EE_CC_mex(transform_inv(:), JointPosition(7), JointPosition(1:7))
all_qs = franka_ik_EE_mex(transform_inv(:), JointPosition(7), JointPosition(1:7))
