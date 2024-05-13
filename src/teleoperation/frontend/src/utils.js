// Function to convert coordinates between different odom formats

const convertDMS = function(coord_in, odom_format) {
  const DEG_DECIMALS = 8
  const MIN_DECIMALS = 6
  const SEC_DECIMALS = 4
  const coord_total = coord_in.d + coord_in.m / 60 + coord_in.s / 3600
  let coord_out = {}
  if (odom_format === 'DMS') {
    coord_out.d = Math.trunc(coord_total.toFixed(DEG_DECIMALS))
    coord_out.m = Math.trunc(((coord_total - coord_out.d) * 60).toFixed(MIN_DECIMALS))
    coord_out.s = +(((coord_total - coord_out.d) * 60 - coord_out.m) * 60).toFixed(SEC_DECIMALS)
  } else if (odom_format === 'DM') {
    coord_out.d = Math.trunc(coord_total.toFixed(DEG_DECIMALS))
    coord_out.m = +((coord_total - coord_out.d) * 60).toFixed(MIN_DECIMALS)
    coord_out.s = 0
  } else {
    coord_out.d = +coord_total.toFixed(DEG_DECIMALS)
    coord_out.m = 0
    coord_out.s = 0
  }
  return coord_out
}

const quaternionToMapAngle = function(quaternion) {
  /*
    Convert a quaternion into euler display angles
  */
  const q0 = quaternion[0]
  const q1 = quaternion[1]
  const q2 = quaternion[2]
  const q3 = quaternion[3]

  const Rx = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
  const Ry = Math.asin(2 * (q0 * q2 - q3 * q1))
  const Rz = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

  const euler = [Rx, Ry, Rz]
  return (Math.PI / 2 - euler[0]) * (180 / Math.PI)
}

export { convertDMS, quaternionToMapAngle }
