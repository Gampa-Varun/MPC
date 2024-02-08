import casadi



def conjugate(q):

    q_conj = q
    q_conj[1] = -q[1]
    q_conj[2] = -q[2]
    q_conj[3] = -q[3]
    return q_conj

def quat_mult(q1, q2):
    # q = casadi.DM.zeros(4,1)
    # q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    # q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    # q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    # q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]

    q = casadi.vertcat(q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
                        q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
                        q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
                        q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0])
    return q


def quat_vector_rotate(q1, v1):

    v1_extend = casadi.vertcat(0, v1[0], v1[1], v1[2])

    q1_conj = conjugate(q1)

    v1_rot_extend = quat_mult(quat_mult(q1,v1_extend),q1_conj)

    v1_rot = casadi.vertcat(v1_rot_extend[1], v1_rot_extend[2], v1_rot_extend[3])

    return v1_rot
    