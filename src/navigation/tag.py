from common import BaseState, Context


class SingleTagState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['single_tag', 'single_tag_done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        return 'single_tag'
