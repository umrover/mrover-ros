const convertDMS = function(coordinate, format: string) {
  const DEG_DECIMALS = 8
  const MIN_DECIMALS = 6
  const SEC_DECIMALS = 4
  const coordinate_total: number = coordinate.d + coordinate.m / 60 + coordinate.s / 3600
  const result = {
    d: 0,
    m: 0,
    s: 0
  }
  switch (format) {
    case 'DMS':
      result.d = Math.trunc(Number(coordinate_total.toFixed(DEG_DECIMALS)))
      result.m = Math.trunc(Number(((coordinate_total - result.d) * 60).toFixed(MIN_DECIMALS)))
      result.s = +(((coordinate_total - result.d) * 60 - result.m) * 60).toFixed(SEC_DECIMALS)
      break
    case 'DM':
      result.d = Math.trunc(Number(coordinate_total.toFixed(DEG_DECIMALS)))
      result.m = +((coordinate_total - result.d) * 60).toFixed(MIN_DECIMALS)
      result.s = 0
      break
    case 'D':
      result.d = +coordinate_total.toFixed(DEG_DECIMALS)
      result.m = 0
      result.s = 0
      break
    default:
      throw new Error('Invalid coordinate format. Supported: DMS, DM, D')
  }
  return result
}

const quaternionToMapAngle = function(quaternion: number[]): number {
  const [qx, qy, qz, qw] = quaternion
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  return (Math.PI / 2 - yaw) * (180 / Math.PI)
}

export { convertDMS, quaternionToMapAngle }
