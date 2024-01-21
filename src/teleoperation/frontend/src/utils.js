// Function to convert coordinates between different odom formats
<<<<<<< HEAD
import * as qte from "quaternion-to-euler";
import { inject } from "vue";
const convertDMS = function (coord_in, odom_format) {
  const DEG_DECIMALS = 8;
  const MIN_DECIMALS = 6;
  const SEC_DECIMALS = 4;
  const coord_total = coord_in.d + coord_in.m / 60 + coord_in.s / 3600;
  let coord_out = {};
  if (odom_format === "DMS") {
    coord_out.d = Math.trunc(coord_total.toFixed(DEG_DECIMALS));
    coord_out.m = Math.trunc(
      ((coord_total - coord_out.d) * 60).toFixed(MIN_DECIMALS)
    );
    coord_out.s = +(
      ((coord_total - coord_out.d) * 60 - coord_out.m) *
      60
    ).toFixed(SEC_DECIMALS);
  } else if (odom_format === "DM") {
    coord_out.d = Math.trunc(coord_total.toFixed(DEG_DECIMALS));
    coord_out.m = +((coord_total - coord_out.d) * 60).toFixed(MIN_DECIMALS);
    coord_out.s = 0;
  } else {
    coord_out.d = +coord_total.toFixed(DEG_DECIMALS);
    coord_out.m = 0;
    coord_out.s = 0;
  }
  return coord_out;
};
=======
import * as qte from 'quaternion-to-euler'
import { inject } from 'vue'
const convertDMS = function (coord_in, odom_format) {
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
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68

const quaternionToMapAngle = function (quaternion) {
  /*
    Convert a quaternion into euler display angles
  */
<<<<<<< HEAD
  quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z];
  let euler = qte(quaternion);
  // euler[2] == euler z component
  return (Math.PI / 2.0 - euler[2]) * (180 / Math.PI);
};

const disableAutonLED = function () {
    // const ws = inject<WebSocket>("webSocketService");
    const ws =  new WebSocket('ws://localhost:8000/ws/gui');
    ws.send(JSON.stringify({type:"disable_auton_led"}))
};

export { convertDMS, quaternionToMapAngle, disableAutonLED };
=======
  quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
  let euler = qte(quaternion)
  // euler[2] == euler z component
  return (Math.PI / 2.0 - euler[2]) * (180 / Math.PI)
}

const disableAutonLED = function () {
  // const ws = inject<WebSocket>("webSocketService");
  const ws = new WebSocket('ws://localhost:8000/ws/gui')
  ws.send(JSON.stringify({ type: 'disable_auton_led' }))
}

export { convertDMS, quaternionToMapAngle, disableAutonLED }
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
