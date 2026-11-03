import * as THREE from 'three';

// Photo-informed silhouette, not a measured A2/Piper CAD model. Local +X is
// forward, +Y is up and -Z is left. The user's old panorama sector 90° is front;
// the simulation already uses robot-forward as sector zero. Only the physical
// 360 housing rotates 90°: its two lenses face robot left/right, not front/back.
export function createRobotModel({cameraHeight=1.6}={}){
  const root=new THREE.Group();root.name='A2 with forward arm and camera rig';
  root.userData.reference='2026-10-30 /dev/video91; user-confirmed forward view';
  root.userData.dimensionsEstimated=true;
  const material=(color,metalness=0,roughness=.65)=>new THREE.MeshStandardMaterial({color,metalness,roughness});
  const dark=material('#24292e',.35),shell=material('#353c43',.4),metal=material('#899299',.65,.4);
  const rubber=material('#14181c',0,.9),red=material('#a6383d',.25),glass=material('#142c3c',.5,.2);
  const unitBox=new THREE.BoxGeometry(1,1,1),unitBall=new THREE.SphereGeometry(1,12,8);
  const unitCylinder=new THREE.CylinderGeometry(1,1,1,12);
  const up=new THREE.Vector3(0,1,0);
  function mesh(geometry,mat,position,parent=root){
    const part=new THREE.Mesh(geometry,mat);part.position.set(...position);
    part.castShadow=true;part.receiveShadow=true;part.userData.semantic='robot';parent.add(part);return part;
  }
  function box(size,position,mat=dark,parent=root){const part=mesh(unitBox,mat,position,parent);part.scale.set(...size);return part;}
  function ball(radius,position,mat=shell){const part=mesh(unitBall,mat,position);part.scale.setScalar(radius);return part;}
  function rod(start,end,radius,mat=dark,parent=root){
    const a=new THREE.Vector3(...start),b=new THREE.Vector3(...end),delta=b.clone().sub(a);
    const part=mesh(unitCylinder,mat,a.clone().add(b).multiplyScalar(.5).toArray(),parent);
    part.scale.set(radius,delta.length(),radius);part.quaternion.setFromUnitVectors(up,delta.normalize());return part;
  }
  function cable(points,radius=.009){
    const curve=new THREE.CatmullRomCurve3(points.map(p=>new THREE.Vector3(...p)));
    return mesh(new THREE.TubeGeometry(curve,28,radius,6,false),rubber,[0,0,0]);
  }

  // Compact quadruped, silver side plates and articulated legs. No contact dynamics.
  box([1.04,.25,.46],[0,.64,0],dark);
  box([.85,.055,.49],[0,.79,0],shell);
  for(const side of [-1,1]){
    box([.66,.16,.025],[-.04,.65,side*.242],metal);
    for(const x of [-.39,.39]){
      const hip=[x,.61,side*.285],knee=[x-.10,.32,side*.34],ankle=[x+.035,.10,side*.36];
      rod([x,.61,side*.22],hip,.085,shell);ball(.082,hip);
      rod(hip,knee,.052,shell);ball(.065,knee,dark);rod(knee,ankle,.034,metal);
      const foot=ball(.065,ankle,rubber);foot.scale.y=.042;
      box([.14,.045,.105],[ankle[0],.055,ankle[2]],rubber);
    }
  }
  box([.075,.13,.34],[.535,.66,0],shell);
  for(const z of [-.12,.12])rod([.574,.69,z],[.584,.69,z],.026,glass);

  // Equipment tray and protective rails visible around the panorama's lower rim.
  box([.87,.035,.57],[-.015,.835,0],dark);
  box([.32,.12,.34],[-.27,.91,0],shell);
  box([.27,.018,.32],[-.27,.977,0],red);
  for(const side of [-1,1]){
    rod([-.47,.89,side*.29],[.48,.89,side*.29],.017,shell);
    for(const x of [-.43,.41])rod([x,.81,side*.26],[x,.94,side*.29],.014,metal);
    rod([-.43,.94,side*.29],[-.21,1.015,side*.29],.016,shell);
    box([.22,.027,.025],[-.32,1.01,side*.29],metal).rotation.z=.18;
  }
  // Flat black cable runs, rear connectors and straps on the equipment deck.
  for(const x of [-.35,-.20])box([.035,.135,.355],[x,.915,0],rubber);
  cable([[-.4,.99,.15],[-.16,.91,.24],[.12,.9,.23],[.25,.92,.10]],.012);
  cable([[-.36,.9,-.15],[-.44,.97,-.23],[-.1,.92,-.26],[.18,.91,-.13]],.011);

  // Forward-mounted arm with simplified wiring omitted from its silhouette.
  // Reach stays inside the old .65 m footprint.
  const base=[.16,.91,0],shoulder=[.20,1.02,0],elbow=[.32,1.22,0],wrist=[.50,1.40,0];
  rod([.16,.85,0],base,.105,shell);rod(base,shoulder,.075,dark);
  for(const p of [shoulder,elbow,wrist]){
    ball(.064,p,shell);
    rod([p[0],p[1],-.064],[p[0],p[1],.064],.052,metal);
    rod([p[0],p[1],-.071],[p[0],p[1],-.065],.023,dark);
    rod([p[0],p[1],.065],[p[0],p[1],.071],.023,dark);
  }
  rod(shoulder,elbow,.045,dark);rod(elbow,wrist,.038,shell);
  box([.11,.07,.08],[.545,1.455,0],dark).rotation.z=.55;
  for(const side of [-1,1]){
    rod([.56,1.47,side*.025],[.61,1.49,side*.05],.012,metal);
    rod([.61,1.49,side*.05],[.64,1.48,side*.035],.009,dark);
  }

  // The lens origin remains at the configured camera height. Only the camera
  // housings are excluded from their own sensor views; the arm/deck stay visible.
  rod([0,.88,0],[0,cameraHeight-.10,0],.019,dark);
  // The stitched Insta360 nadir shows the deck, not a polished near-lens cap.
  rod([0,cameraHeight-.11,0],[0,cameraHeight-.06,0],.034,metal).userData.sensorHidden=true;
  const surround=new THREE.Group(),front=new THREE.Group();
  surround.name='360 camera housing';front.name='front camera housing';
  for(const housing of [surround,front]){housing.userData.sensorHidden=true;root.add(housing);}
  box([.048,.15,.064],[0,cameraHeight-.025,0],dark,surround);
  for(const side of [-1,1])rod([side*.024,cameraHeight,0],[side*.035,cameraHeight,0],.029,glass,surround);
  surround.rotation.y=Math.PI/2; // Local lens ±X becomes robot ∓Z (left/right).
  surround.userData.lensDirections='robot-left-and-right';
  box([.075,.08,.19],[0,cameraHeight,0],shell,front);
  for(const side of [-1,1])rod([.0375,cameraHeight,side*.064],[.049,cameraHeight,side*.064],.023,glass,front);
  front.visible=false;
  return {root,surroundHousing:[surround],frontHousing:[front]};
}
