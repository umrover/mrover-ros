import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { TrackballControls } from 'three/addons/controls/TrackballControls.js';

export function threeSetup(containerId) {
    // Get the container where the scene should be placed
    const container = document.getElementById(containerId);
    
    const canvas = {
        width: container.clientWidth,
        height: container.clientHeight
    }

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, canvas.width / canvas.height, 0.1, 1000);
    
    const light = new THREE.AmbientLight( 0x404040 ); // soft white light
    scene.add( light );

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


    const loader = new ColladaLoader();
    loader.load( '/meshes/rover.dae', function ( collada ) {
        // collada.scene.traverse(node => {
        //     if (node.isMesh) {
        //         node.material = new THREE.MeshPhongMaterial({
        //             color: node.material.color,
        //             map: node.material.map
        //         });
        //     }
        // });
        scene.add( collada.scene );

    } );

    camera.position.z = 1.5;
  
    function animate() {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }
  
    animate();

    function fk(positions) {
      const LINK_AB = 22.8544;
      const LINK_BC = 19.5129;
      const LINK_CD = 5.59352;
      let p1 = {
        x: 0,
        y: 0
      }
      let p2 = {
        x: p1.x + Math.cos(positions[0]),
        y: p1.y + Math.sin(positions[0])
      }
      let p3 = {
        x: p2.x + Math.cos(positions[0]+positions[1]),
        y: p2.y + Math.sin(positions[0]+positions[1])
      }
    }

    function updateJointAngles(newJointAngles) {
      //...code to update the scene based on the newJointAngles parameter...
    }

    // Return an object with the updateJointAngles() function
    return {
        updateJointAngles
    }
  };
  