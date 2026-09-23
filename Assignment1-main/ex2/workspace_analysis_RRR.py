import numpy as np


# Example usage
# joints = [3.14, 2.5, 1.5]
# linkLengths = [0.375, 0.310, 0.1]
def endEffector_RRR(joints, linkLengths):
    # TODO
    t1, t2, t3 = joints
    l1, l2, l3 = linkLengths
    s1, c1 = np.sin(t1), np.cos(t1)
    s12, c12 = np.sin(t1 + t2), np.cos(t1 + t2)
    s123, c123 = np.sin(t1 + t2 + t3), np.cos(t1 + t2 + t3)
    
    x = l1 * c1 + l2 * c12 + l3 * c123
    y = l1 * s1 + l2 * s12 + l3 * s123
    return x, y


def workspace_analysis_RRR(
        nSamples = 50, minJoint=[0, 0, 0], 
        maxJoint=[2*np.pi, 2*np.pi, 2*np.pi], 
        linkLengths=[0.3, 0.2, 0.1]):
    # TODO
    theta1 = np.linspace(minJoint[0], maxJoint[0], nSamples)
    theta2 = np.linspace(minJoint[1], maxJoint[1], nSamples)
    theta3 = np.linspace(minJoint[2], maxJoint[2], nSamples)
    xs = np.zeros(nSamples**3)
    ys = np.zeros(nSamples**3)
    
    for i in range(nSamples):
        for j in range(nSamples):
            for k in range(nSamples):
                joints = [theta1[i], theta2[j], theta3[k]]
                index = i * (nSamples**2) + j * nSamples + k
                xs[index], ys[index] = endEffector_RRR(joints, linkLengths)
    return xs, ys

