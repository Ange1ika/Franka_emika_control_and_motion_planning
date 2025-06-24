# Franka_emika_control_and_motion_planning
Here is a project from the UGRIP MBZUAI, project dedicated by robot motion and planning, avoiding obstacles. Here implemented impedance control with using libfranka [https://github.com/frankarobotics/libfranka/tree/main] and ruckig for planning trajectory [https://github.com/pantor/ruckig], also slerp was done for addind orientation.

simple slerp:
quat slerp(quat q1, quat q2, float t) {
    float angle = acos(dotProduct(q1, q2));
    float denom = sin(angle);
    //check if denom is zero
    return (q1*sin((1-t)*angle)+q2*sin(t*angle))/denom;
}
