import pymel.core as pm


def getPoleVector(start, mid, end):
    """
    Returns a unit pole vector for `start`, `mid`, and `end` transforms.
    The pole vector is (parallel to) the vector orthogonal to the vector
    between `start` and `end` that passes through `mid`.
    (Note that `start` and `end` are interchangeable.)

    Parameters
    ----------
    start : pm.nodetypes.Transform

    mid : pm.nodetypes.Transform

    end : pm.nodetypes.Transform


    Returns
    -------
    pm.datatypes.Vector

    """

    start, mid, end = (pm.ls(x)[0] for x in (start, mid, end))

    locs = [xform.getTranslation(space='world') for xform in [start, mid, end]]
    vec_basen = (locs[2] - locs[0]).normal()
    vec_mid = (locs[1] - locs[0])
    pole_vec = (vec_mid - vec_mid.dot(vec_basen)*vec_basen)

    return pole_vec


def alignToVector(xform, aim_x=(1, 0, 0), aim_y=(0, 1, 0), freeze=False):
    """
    Rotates transform so that axes align with specified world-space directions.

    Parameters
    ----------
    xform : pm.nt.Transform
        Transform to rotate.
    aim_x : tuple, optional
        World-space direction for the x-axis of `xform`.
    aim_y : tuple, optional
        World-space direction for the y-axis of `xform`.  If not orthogonal to `aim_x`,
        the y-axis will attempt to be as close as possible to this vector.
    freeze : bool, optional
        Freeze transformation if True. Default is False.

    """

    xform = pm.ls(xform)[0]

    xf_node = pm.createNode('transform')
    pm.move(xf_node, xform.getTranslation(ws=True))

    aim_node = pm.createNode('transform')
    pm.move(aim_node, xform.getTranslation(space='world') + aim_x)

    pm.delete(pm.aimConstraint(aim_node, xf_node,
                               worldUpVector=aim_y), aim_node)

    xform.setRotation(xf_node.getRotation(ws=True), ws=True)
    pm.delete(xf_node)

    if freeze:
        pm.makeIdentity(xform, apply=True)


def orientJoint(joint, aim_loc, aim_axis=(1, 0, 0), up_axis=(0, 1, 0), world_up=(0, 1, 0)):

    joint = pm.ls(joint)[0]

    children = joint.listRelatives()

    if children:
        for child in children:
            child.setParent(None)

    target = pm.createNode('transform')
    target.setTranslation(aim_loc, ws=True)

    pm.delete(pm.aimConstraint(target, joint, aimVector=aim_axis,
                               upVector=up_axis, worldUpVector=world_up))

    pm.delete(target)

    if children:
        for child in children:
            child.setParent(joint)

    pm.makeIdentity(joint, apply=True)


def alignToWorldVector(xform, aim_x=(1, 0, 0), aim_y=(0, 1, 0), freeze=False):
    """
    Rotates transform so that axes align with specified world-space directions.

    Parameters
    ----------
    xform : pm.nt.Transform
        Transform to rotate.
    aim_x : tuple, optional
        World-space direction for the x-axis of `xform`.
    aim_y : tuple, optional
        World-space direction for the y-axis of `xform`.  If not orthogonal to `aim_x`,
        the y-axis will attempt to be as close as possible to this vector.
    freeze : bool, optional
        Freeze transformation if True. Default is False.

    """

    xform = pm.ls(xform)[0]

    xf_node = pm.createNode('transform')
    pm.move(xf_node, xform.getTranslation(ws=True))

    aim_node = pm.createNode('transform')
    pm.move(aim_node, xform.getTranslation(space='world') + aim_x)

    pm.delete(pm.aimConstraint(aim_node, xf_node,
                               worldUpVector=aim_y), aim_node)

    xform.setRotation(xf_node.getRotation(ws=True), ws=True)
    pm.delete(xf_node)

    if freeze:
        pm.makeIdentity(xform, apply=True)
