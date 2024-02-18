import * as THREE from 'three';
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader'
import { TrackballControls } from 'three/addons/controls/TrackballControls.js';

export function threeSetup(containerId) {
  // Get the container where the scene should be placed
  const container = document.getElementById(containerId);

  const canvas = {
    width: container.clientWidth,
    height: container.clientHeight
  }

  // THREE.Object3D.DefaultUp = new THREE.Vector3(0, 0, 1);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(75, canvas.width / canvas.height, 0.1, 1000);
  camera.up.set(0, 0, 1);

  const light = new THREE.AmbientLight(0x404040); // soft white light
  scene.add(light);

  const renderer = new THREE.WebGLRenderer();
  renderer.setSize(canvas.width, canvas.height);
  container.appendChild(renderer.domElement); // append it to your specific HTML element

  scene.background = new THREE.Color(0xcccccc);

  var controls = new TrackballControls(camera, renderer.domElement);
  controls.rotateSpeed = 1.0;
  controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;

  const joints = [
    {
      name: "chassis",
      file: "rover_chassis.fbx",
      translation: [0.164882, 0.200235, 0.051497],
      rotation: [0, 0, 0],
    },
    {
      name: "a",
      file: "arm_a.fbx",
      translation: [0.034346, 0, 0.049024],
      rotation: [0, 0, 0],
    },
    {
      name: "b",
      file: "arm_b.fbx",
      translation: [0.534365, 0, 0.009056],
      rotation: [0, 0, 0],
    },
    {
      name: "c",
      file: "arm_c.fbx",
      translation: [19.5129, 0.264652, 0.03498],
      rotation: [0, 0, 0],
    },
    {
      name: "d",
      file: "arm_d.fbx",
      translation: [0.546033, 0, 0.088594],
      rotation: [0, 0, 0],
    },
    {
      name: "e",
      file: "arm_e.fbx",
      translation: [0.044886, 0, 0],
      rotation: [0, 0, 0],
    }
  ]

  const loader = new FBXLoader();
  for (let i = 0; i < joints.length; ++i) {
    loader.load('/meshes/' + joints[i].file, (fbx) => {
      // Traverse the loaded scene to find meshes or objects
      fbx.traverse((child)=>  {
        if (child.isMesh) {
          scene.add(child);
          child.scale.set(1, 1, 1);
          child.position.set(0, 0, 0);
        }
      });
    },
    (xhr) => {
        console.log((xhr.loaded / xhr.total) * 100 + '% loaded')
    },
    (error) => {
        console.log(error)
    });
  }

  camera.position.x = 2;
  camera.position.y = 2;
  camera.position.z = 2;
  camera.lookAt(new THREE.Vector3(0, 0, 0));

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }

  animate();

  function fk(positions) {

    let cumulativeMatrix = new THREE.Matrix4();

    console.log(positions);

    positions.forEach((newAngle, i) => {
      let mesh = scene.getObjectByName(joints[i].name);

      let localMatrix = new THREE.Matrix4();

      // Assume rotation around Y-axis, create rotation matrix from the angle
      let rotationAngle = newAngle; // Angle for this joint
      if (joints[i].name == "d") {
        localMatrix.makeRotationX(rotationAngle);
      }
      else {
        localMatrix.makeRotationY(rotationAngle);
      }

      let offset = new THREE.Vector3().fromArray(joints[i].translation);

      localMatrix.setPosition(offset.x, offset.y, offset.z);

      mesh.matrixAutoUpdate = false;
      mesh.matrix = cumulativeMatrix.clone();

      cumulativeMatrix.multiply(localMatrix);
    });
  }

  function updateJointAngles(newJointAngles) {
    fk(newJointAngles);
  }

  // Return an object with the updateJointAngles() function
  return {
    updateJointAngles
  }
};
