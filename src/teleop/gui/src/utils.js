// Function to convert coordinates between different odom formats
import * as qte from "quaternion-to-euler";
import ROSLIB from "roslib";
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

const quaternionToMapAngle = function (quaternion) {
  /*
    Convert a quaternion into euler display angles
  */
  quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z];
  let euler = qte(quaternion);
  // euler[2] == euler z component
  return (Math.PI / 2.0 - euler[2]) * (180 / Math.PI);
};

const disableAutonLED = function (ros) {
  let auton_led_client = new ROSLIB.Service({
    ros: ros,
    name: "change_auton_led_state",
    serviceType: "mrover/ChangeAutonLEDState",
  });

  let request = new ROSLIB.ServiceRequest({
    color: "off",
  });

  auton_led_client.callService(request, (result) => {
    // Wait 1 second then try again if fail
    if (!result.success) {
      alert("Failed to disable auton LED. Retrying in 1 second.");
      setTimeout(() => {
        disableAutonLED(ros);
      }, 1000);
    }
  });
};

export { convertDMS, quaternionToMapAngle, disableAutonLED };
