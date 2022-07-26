import gtsam
import gtsam_quadrics


def ps_and_qs_from_values(values: gtsam.Values):
    # TODO there's got to be a better way to access the typed values...
    return ({
        k: values.atPose3(k)
        for k in values.keys()
        if gtsam.Symbol(k).string()[0] == 'x'
    }, {
        k: gtsam_quadrics.ConstrainedDualQuadric.getFromValues(values, k)
        for k in values.keys()
        if gtsam.Symbol(k).string()[0] == 'q'
    })
