from navigation.approach_target_base import ApproachTargetBaseState


class ApproachPostState(ApproachTargetBaseState):
    """
    State for when the tag is seen in the ZED camera.
    Transitions:
    -If arrived at target: DoneState
    -Did not arrive at target: ApproachPostState
    """

    pass
