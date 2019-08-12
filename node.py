from functools import partial

import pymel.core as pm


CTRL_SHAPES = {
    'ik': {
        'point': [[0.0, 1.0, 1.0],
                  [0.0, 1.0, -1.0],
                  [0.0, -1.0, -1.0],
                  [0.0, -1.0, 1.0],
                  [0.0, 1.0, 1.0]],
        'degree': 1},

    'fk': {
        'point': [[0.0, 0.7836, -0.7836],
                  [0.0, 1.1082, -0.0],
                  [-0.0, 0.7836, 0.7836],
                  [-0.0, 0.0, 1.1082],
                  [-0.0, -0.7836, 0.7836],
                  [-0.0, -1.1082, 0.0],
                  [0.0, -0.7836, -0.7836],
                  [0.0, -0.0, -1.1082],
                  [0.0, 0.7836, -0.7836],
                  [0.0, 1.1082, -0.0],
                  [-0.0, 0.7836, 0.7836]],
        'degree': 3,
        'knot': [-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]},

    'pole': {'point': [[0.0, 1.0, 0.0],
                       [0.0, -1.0, 0.0],
                       [0.0, 0.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, 0.0, -1.0]],
             'degree': 1},

    'other': {'point': [[0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [0.0, -1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0]],
              'degree': 1}
}

CTRL_COLORS = {
    'Left': (1.0, 0.0, 0.0),
    'Right': (0.0, 0.0, 1.0),
    'Center': (0.15, 1.0, 1.0),
    'Other': (1.0, 1.0, 0.15)
}


def createControlCurve(name=None, ctrl_type='FK', size=1.0, color=(1.0, 1.0, 0.15)):
    """
    Creates a curve using predefined parameters.

    Parameters
    ----------
    name : str
        Desired curve name in Maya
    ctrlType : str
        Shape type, as defined in rig.ControlShapes (the default is 'FK')
    size : float
        Curve radius, in Maya scene units (the default is 1.0)
    color : tuple
        Tuple of RGB values in 0.0 to 1.0 range.  Set to None to use Maya's default color.

    """

    shape_args = CTRL_SHAPES.get(ctrl_type.lower(), CTRL_SHAPES['other'])

    if name:
        shape_args.update({'name': name})
    crv = pm.curve(**shape_args)

    if color:
        crv.setAttr('overrideColorRGB', color)
        crv.setAttr('overrideEnabled', True)
        crv.setAttr('overrideRGBColors', True)

    if size:
        crv.setScale([size, size, size])
        pm.makeIdentity(crv, apply=True)

    return(crv)


def createNodeChain(input_xforms, node_func=partial(pm.createNode, 'transform'), name_list=None, prefix='_'):
    if not isinstance(input_xforms, list):
        input_xforms = [input_xforms]

    if not name_list:
        name_list = [prefix + node.nodeName() for node in input_xforms]

    node_list = list()
    for node, node_name in zip(input_xforms, name_list):
        new_node = node_func(name=node_name)
        pm.delete(pm.parentConstraint(node, new_node))
        if node_list:
            new_node.setParent(node_list[-1])
        node_list.append(new_node)

    return node_list


def lockAndHideAttrs(node_list, attr_list=('scaleX', 'scaleY', 'scaleZ')):
    for node in node_list:
        for attr_name in attr_list:
            node.setAttr(attr_name, lock=True, keyable=False)


def makeLockXYZ(xf_name, unlock=False):

    def lockFunc(xforms):
        xforms = pm.ls(xforms)
        xf_attrs = [xf_name+axis for axis in ('X', 'Y', 'Z')]

        for xform in xforms:
            for xf_attr in xf_attrs:
                xform.attr(xf_attr).set(k=unlock, lock=not unlock)

    return lockFunc


lockTranslate = makeLockXYZ('translate')
unlockTranslate = makeLockXYZ('translate', unlock=True)
lockRotate = makeLockXYZ('rotate')
unlockRotate = makeLockXYZ('rotate', unlock=True)
lockScale = makeLockXYZ('scale')
unlockScale = makeLockXYZ('scale', unlock=True)
