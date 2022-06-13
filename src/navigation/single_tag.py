from common import Context

from navigation.waypoint import WaypointState


class SingleTagState(WaypointState):
    def __init__(self, context: Context, *args, **kwargs):
        kwargs['input_keys'].extend(['fiducial_id'])
        super().__init__(context, *args, **kwargs)

    def has_fiducial_with_id(self, fid_id: int):
        fiducial_id_set = set(fid.fiducial_id for fid in self.context.fiducial_transforms.fiducials)
        return fid_id in fiducial_id_set

    def evaluate(self, ud):
        is_on_last_waypoint = ud.waypoint_index == len(ud.waypoints) - 1
        if is_on_last_waypoint and self.has_fiducial_with_id(ud.fiducial_id):
            pass
        else:
            super().evaluate(ud)
