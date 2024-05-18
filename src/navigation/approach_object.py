from navigation.approach_target_base import ApproachTargetBaseState


class ApproachObjectState(ApproachTargetBaseState):
    """
    State for when we see an object (mallet or water bottle).
    We are only using the ZED camera.
    Transitions:
    -If arrived at target: DoneState
    -Did not arrive at target: ApproachObjectState
    """

    pass
