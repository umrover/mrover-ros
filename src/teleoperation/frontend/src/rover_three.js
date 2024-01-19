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



    // const geometry = new THREE.BoxGeometry(1, 1, 1);
    // const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    // const cube = new THREE.Mesh(geometry, material);
    // scene.add(cube);

    camera.position.z = 1.5;
  
    function animate() {
      requestAnimationFrame(animate);
      controls.update();
    //   cube.rotation.x += 0.01;
    //   cube.rotation.y += 0.01;
      renderer.render(scene, camera);
    }
  
    animate();

    // function onWindowResize() {
    //     camera.aspect = container.clientWidth / container.clientHeight; // Update the camera's aspect ratio
    //     camera.updateProjectionMatrix(); // Update the camera's projection matrix
    //     renderer.setSize(container.clientWidth, container.clientHeight); // Resizer the renderer to the container's size
    // }
    // window.addEventListener('resize', onWindowResize, false);
  };
  