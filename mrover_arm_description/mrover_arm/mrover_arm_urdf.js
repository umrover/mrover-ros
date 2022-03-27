//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

var scale_factor = 0.0254;
var scale_factor_neg = -1*scale_factor;

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "mrover_arm";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0.1,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "chassis-a";  

// specify and create data objects for the links of the robot
// units in inches
robot.links = {
    "chassis-a": {
        visual : { 
            origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "chassis-a.stl" } },
            material : { color : { rgba : [1, 0, 0, 1] } }
            //Red
        }
    },
    "a-b": {
        visual : { 
            origin : { xyz: [0.0,0.0,0.2721*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "a-b.stl" } },
            material : { color : { rgba : [1, .43, 0, 1] } }
            //Orange
        }
    },
    "b-c": {
        visual : { 
            origin : { xyz: [0.000,-1.4234*scale_factor_neg,1.8571*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "b-c.stl" } },
            material : { color : { rgba : [1, 1, 0, 1] } }
            //Yellow
        }
    },
    "c-d": {
        visual : { 
            origin : { xyz: [0.000,-0.5393*scale_factor_neg,12.9821*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "c-d.stl" } },
            material : { color : { rgba : [0.43, 1, 0, 1] } }
            //Green
        }
    },
    "d-e": {
        visual : { 
            origin : { xyz: [0.000,1.7249*scale_factor_neg,22.1071*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "d-e.stl" } },
            material : { color : { rgba : [0, 0, 1, 1] } }
            //Blue
        }
    },
    "e-f": {
        visual : { 
            origin : { xyz: [-1.002*scale_factor_neg,2.7240*scale_factor_neg,22.6051*scale_factor_neg], rpy:[0,0,0] },
            origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "e-f.stl" } },
            material : { color : { rgba : [.5, 0, 1, 1] } }
            //Violet
        }
    },
    "hand": {
        visual: { 
            origin : { xyz: [-2.5166*scale_factor_neg,2.7226*scale_factor_neg,34.3566*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "hand.stl" } },
            material : { color : { rgba : [1, 0, 1, 1] } }
            //Pink
        }
    }
};

robot.target_material = { color : {rgba : [1, 1, 0.5, 1] } };

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

// units converted from inches to meters

robot.joints.joint_a = {parent:"chassis-a", child:"a-b"};
robot.joints.joint_a.origin = {xyz: [0.0,0.0,0.2721*scale_factor], rpy:[0,0,0]};
robot.joints.joint_a.axis = [0.0,0.0,1.0];   
robot.joints.joint_a.type = "continuous";

robot.joints.joint_b = {parent:"a-b", child:"b-c"};
robot.joints.joint_b.origin = {xyz: [0.0,-1.4234*scale_factor,1.585*scale_factor], rpy:[0,0,0]};
robot.joints.joint_b.axis = [1.0,0.0,0];   
robot.joints.joint_b.type = "revolute";
robot.joints.joint_b.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_c = {parent:"b-c", child:"c-d"};
robot.joints.joint_c.origin = {xyz: [0.0,0.8841*scale_factor,11.125*scale_factor], rpy:[0,0,0]};
robot.joints.joint_c.axis = [1.0,0.0,0];   
robot.joints.joint_c.type = "revolute";
robot.joints.joint_c.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_d = {parent:"c-d", child:"d-e"};
robot.joints.joint_d.origin = {xyz: [0.0,2.2642*scale_factor,9.125*scale_factor], rpy:[0,0,0]};
robot.joints.joint_d.axis = [0.0,1.0,0];   
robot.joints.joint_d.type = "revolute";
robot.joints.joint_d.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_e = {parent:"d-e", child:"e-f"};
robot.joints.joint_e.origin = {xyz: [-1.002*scale_factor,0.9991*scale_factor,1.498*scale_factor], rpy:[0,0,0]};
robot.joints.joint_e.axis = [1.0,0.0,0];   
robot.joints.joint_e.type = "revolute";
robot.joints.joint_e.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_f = {parent:"e-f", child:"hand"};
robot.joints.joint_f.origin = {xyz: [-1.5146*scale_factor,-0.0014*scale_factor,10.7515*scale_factor], rpy:[0,0,0]};
robot.joints.joint_f.axis = [0.0,0.0,1.0];   
robot.joints.joint_f.type = "revolute";
robot.joints.joint_f.limit = {lower:-2.0, upper:2.0};

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "joint_f";
robot.endeffector.position = [[0],[0],[0],[1]]
robot.endeffector.name = "hand";

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

robot.links_geom_imported = true;

links_geom = {};

// define threejs geometries and associate with robot links 
progressLinkLoading = 0;
i = 0;
imax = Object.keys(robot.links).length;

for (linkname in robot.links) {
    filename_split = robot.links[linkname].visual.geometry.mesh.filename.split('.');
    geom_index = filename_split[0];
    geom_extension = filename_split[filename_split.length-1];
    console.log(geom_index + "  " + geom_extension);

    // assignFetchModelSTL('./robots/mrover_arm/'+robot.links[linkname].visual.geometry.mesh.filename,
    //                     robot.links[linkname].visual.material,
    //                     linkname);
    assignMRoverModelSTL('./robots/mrover_arm/'+robot.links[linkname].visual.geometry.mesh.filename,
                        robot.links[linkname].visual.material,
                        linkname,
                        scale_factor);
    i++;
  
    progressLinkLoading = i/imax; 
    console.log("Robot geometry: progressLinkLoading " + progressLinkLoading*100);
}

// Add "target" to links_geom with yellow color of b-c
assignMRoverModelSTL("./robots/mrover_arm/"+robot.links["hand"].visual.geometry.mesh.filename,
                     robot.target_material,
                     "target",
                     scale_factor * 0.6);

function assignMRoverModelSTL(filename, material_urdf, linkname, scale) {
    console.log("assignMRoverModel : "+filename+" - "+linkname); 
    var stl_loader = new THREE.STLLoader();
    var val = stl_loader.load(filename, 
       function ( geometry ) {
            // TODO: add transparency
            var material_color = new THREE.Color(material_urdf.color.rgba[0], material_urdf.color.rgba[1], material_urdf.color.rgba[2]);
            var material = new THREE.MeshLambertMaterial( {color: material_color, side: THREE.DoubleSide} );
            var m = new THREE.Matrix4().makeScale(scale, scale, scale);

            geometry.applyMatrix(m);
            links_geom[linkname] = new THREE.Mesh( geometry, material );
        }
    );
}
