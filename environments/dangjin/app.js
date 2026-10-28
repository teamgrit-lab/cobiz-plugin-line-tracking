import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { WorldGeometry, barnSilos } from './world-geometry.js';
import { enrichScene, roadTexture } from './scene-details.js';
import { RobotSimulation } from './robot-sim.js';

const $ = (id) => document.getElementById(id);
const offline = window.DANGJIN_OFFLINE;
const asset = (path) => offline?.assets[path.replace('./evidence/../','./')] || path;
const V = (x, y, z = 0) => new THREE.Vector3(x, z, -y);
const fmtTime = (seconds) => `${Math.floor(seconds / 60).toString().padStart(2, '0')}:${Math.floor(seconds % 60).toString().padStart(2, '0')}`;
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#dfe5d9');
scene.fog = new THREE.Fog('#dfe5d9', 1000, 1800);
const camera = new THREE.PerspectiveCamera(42, 1, 1, 2200);
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
} catch (error) {
  $('loading').textContent = '브라우저에서 WebGL을 사용할 수 없습니다. 하드웨어 가속을 켠 Chrome 또는 Safari에서 열어 주세요.';
  throw error;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
$('scene').append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.075;
controls.minDistance = 3;
controls.maxDistance = 950;
controls.maxPolarAngle = Math.PI / 2.02;
controls.target.copy(V(60, 90));
const model = new THREE.Group();
const roofs = new THREE.Group();
const vegetation = new THREE.Group();
const cloudGroup = new THREE.Group();
scene.add(model, cloudGroup);
model.add(roofs, vegetation);
cloudGroup.visible = false;
scene.add(new THREE.HemisphereLight('#f8fbf2', '#7d8973', 1.6));
const sun = new THREE.DirectionalLight('#fff2d7', 2.3);
sun.position.set(-200, 370, -200);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -380, right: 380, top: 370, bottom: -370, near: 1, far: 1000 });
sun.shadow.normalBias = 0.45;
sun.shadow.bias = -0.00001;
scene.add(sun);

const mat = (color, options = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.87, ...options });
const M = {
  road: mat('#858b83'), curb: mat('#cbd0bd'), concrete: mat('#c7c6b8'),
  walls: mat('#d9dbcd'), steel: mat('#8e9c96', { metalness: 0.35, roughness: 0.5 }),
  roof: mat('#d9d9cc', { metalness: 0.3, roughness: 0.7, side: THREE.DoubleSide }),
  trim: mat('#855646'), dark: mat('#4b574b'), grass: mat('#9ba987'),
  soil: mat('#9e9b79'), lane: mat('#d0b576'), glass: mat('#718e8c', { metalness: 0.35 }),
  trunk: mat('#787864'), silo: mat('#d6d9cc', { metalness: 0.5, roughness: 0.45 }),
};
M.road.map=roadTexture();
const boxGeometry = new THREE.BoxGeometry(1, 1, 1);
const dummy = new THREE.Object3D();
const batches = new Map();
function box(x, y, z, length, width, height, material, parent = model) {
  let byMaterial = batches.get(parent);
  if (!byMaterial) { byMaterial = new Map(); batches.set(parent, byMaterial); }
  let entries = byMaterial.get(material);
  if (!entries) { entries = []; byMaterial.set(material, entries); }
  dummy.position.copy(V(x, y, z));
  dummy.rotation.set(0, 0, 0);
  dummy.scale.set(length, height, width);
  dummy.updateMatrix();
  entries.push(dummy.matrix.clone());
}
function flushBoxes() {
  for (const [parent, byMaterial] of batches) {
    for (const [material, matrices] of byMaterial) {
      const mesh = new THREE.InstancedMesh(boxGeometry, material, matrices.length);
      matrices.forEach((matrix, index) => mesh.setMatrixAt(index, matrix));
      mesh.castShadow = true; mesh.receiveShadow = true;
      parent.add(mesh);
    }
  }
  batches.clear();
}
function cylinder(x, y, z, radiusTop, radiusBottom, height, material, parent = model, segments = 12) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radiusTop, radiusBottom, height, segments), material);
  mesh.position.copy(V(x, y, z)); mesh.castShadow = true; mesh.receiveShadow = true;
  parent.add(mesh); return mesh;
}
function surface(points, indices, material, parent = model) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(points.flatMap(p => V(...p).toArray()), 3));
  geometry.setIndex(indices); geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry, material); mesh.castShadow = true; mesh.receiveShadow = true;
  parent.add(mesh); return mesh;
}
function line(points, color, parent = model) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => V(...p)));
  const mesh = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color }));
  parent.add(mesh); return mesh;
}
function road(points, width, material = M.road, elevation = 0.08, sharedCurve=null, sensorHidden=false) {
  const curve = sharedCurve||new THREE.CatmullRomCurve3(points.map(p => V(p[0], p[1], elevation)), false, 'centripetal');
  const samples = curve.getPoints(Math.max(30, points.length * 18));
  for(const p of samples)p.y=elevation;
  const positions = [], indices = [], uvs=[];let distance=0;
  samples.forEach((p, i) => {
    const tangent = (samples[Math.min(i+1, samples.length-1)].clone().sub(samples[Math.max(0, i-1)])).normalize();
    const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).multiplyScalar(width / 2);
    positions.push(...p.clone().add(normal).toArray(), ...p.clone().sub(normal).toArray());
    if(i)distance+=p.distanceTo(samples[i-1]);uvs.push(0,distance/8,1,distance/8);
    if (i > 0) { const n = i*2; indices.push(n-2, n, n-1, n-1, n, n+1); }
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs,2));
  geometry.setIndex(indices); geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry, material); mesh.receiveShadow = true; model.add(mesh);
  if(material===M.road)mesh.userData.semantic='road';
  mesh.userData.sensorHidden=sensorHidden;
  return curve;
}
function pavedArea(area){
  const contour=area.points.map(([x,y])=>new THREE.Vector2(x,y));
  const indices=THREE.ShapeUtils.triangulateShape(contour,[]).flat();
  const mesh=surface(area.points.map(([x,y])=>[x,y,.085]),indices,M.road);
  mesh.geometry.setAttribute('uv',new THREE.Float32BufferAttribute(area.points.flatMap(([x,y])=>[x/8,y/8]),2));
  mesh.userData.semantic='road';mesh.userData.pavedArea=area.id;
}
let randomSeed = 11618;
function random() { randomSeed = (randomSeed * 1664525 + 1013904223) >>> 0; return randomSeed / 4294967296; }
function terrainHeight(x, y) {
  const east = Math.max(0, x - 320) / 110;
  const north = Math.max(0, y - 212) / 90;
  const south = Math.max(0, -y - 94) / 80;
  const west = Math.max(0, -x - 325) / 90;
  return (east + north + south + west) * (8 + 3 * Math.sin(x / 73) * Math.cos(y / 67));
}
function terrain() {
  const g = new THREE.PlaneGeometry(1050, 820, 80, 60);
  g.rotateX(-Math.PI / 2); g.translate(-25, -0.15, -75);
  const positions = g.attributes.position, colors = [];
  for (let i=0; i<positions.count; i++) {
    const x=positions.getX(i), y=-positions.getZ(i);
    positions.setY(i, terrainHeight(x, y) - 0.15);
    const c = new THREE.Color('#a9b595').lerp(new THREE.Color('#8c9f78'), random() * 0.35);
    colors.push(c.r, c.g, c.b);
  }
  g.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3)); g.computeVertexNormals();
  const mesh = new THREE.Mesh(g, mat('#ffffff', { vertexColors: true, flatShading: true }));
  mesh.receiveShadow = true; model.add(mesh);
  // Quiet agricultural parcels outside the site. Their shapes are illustrative.
  for (const [x,y,l,w] of [[-170,-62,145,65],[-250,25,120,55],[-175,268,95,45],[80,270,130,48],[-200,318,185,50]]) {
    box(x,y,terrainHeight(x,y),l,w,0.1,mat(random()>.5?'#b7bb91':'#a5ad84'));
    for (let offset=-w/2;offset<w/2;offset+=5) {
      line([[x-l/2,y+offset,terrainHeight(x-l/2,y+offset)+.1],[x+l/2,y+offset,terrainHeight(x+l/2,y+offset)+.1]], '#919f7c');
    }
  }
}
function trees() {
  const positions = [];
  for (let i=0; i<620; i++) {
    const x=-420+random()*890, y=-160+random()*550;
    const alongEdge = (x>320 && x<425) || (y>215 && y<275 && x>-70) || (y< -100 && x>-60) || (x< -305 && y<210);
    if (alongEdge) positions.push([x,y,5+random()*6]);
  }
  const crownGeometry = new THREE.IcosahedronGeometry(1, 1);
  const crowns = new THREE.InstancedMesh(crownGeometry, mat('#708267', { flatShading:true }), positions.length*2);
  const trunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(.28,.46,1,5), M.trunk, positions.length);
  positions.forEach(([x,y,h], i) => {
    const z=terrainHeight(x,y);
    dummy.position.copy(V(x,y,z+h*.35)); dummy.scale.set(1,h*.65,1); dummy.updateMatrix();trunks.setMatrixAt(i,dummy.matrix);
    for(let j=0;j<2;j++){
      dummy.position.copy(V(x+j*1.3,y-j,z+h*.7+j*1.1));dummy.scale.set(h*.47,h*.48,h*.44);dummy.rotation.set(random(),random(),random());dummy.updateMatrix();
      crowns.setMatrixAt(i*2+j,dummy.matrix);crowns.setColorAt(i*2+j,new THREE.Color().setHSL(.22+random()*.035,.16+random()*.1,.32+random()*.1));
    }
  });
  trunks.castShadow=true; crowns.castShadow=true;vegetation.add(crowns,trunks);
}
const labelEntries = [], pickables = [];
function addLabel(item, z) {
  const element=document.createElement('div');element.className='map-label';element.textContent=item.name;
  $('labels').append(element);
  labelEntries.push({element,position:V(item.x+item.length/2,item.y+item.width/2,z),id:item.id,diagramOnly:item.diagramOnly});
}
function pickVolume(item, height) {
  const mesh=new THREE.Mesh(new THREE.BoxGeometry(item.length,height,item.width),new THREE.MeshBasicMaterial({visible:false}));
  mesh.position.copy(V(item.x+item.length/2,item.y+item.width/2,height/2));mesh.userData.facility=item;model.add(mesh);pickables.push(mesh);
}
function barn(b, a) {
  const {x,y,length:l,width:w}=b, c=y+w/2, e=a.eave_height, r=a.ridge_height;
  box(x+l/2,c,.15,l+2,w+2,.3,M.concrete);
  box(x+l/2,c,.32,l,a.aisle_width,.05,M.road);
  // Low side walls and railings, with open steel structure above.
  for(const side of [y,y+w]){
    box(x+l/2,side,a.wall_height/2,l,.25,a.wall_height,M.concrete);
    for(const h of [2.6,3.5,4.4])box(x+l/2,side,h,l,.1,.1,M.steel);
    for(let dx=0;dx<=l;dx+=l/22)box(x+dx,side,e/2,.25,.25,e,M.steel);
  }
  // Inner pen rails and periodic partitions leave an eight-metre feed aisle.
  for(const side of [-1,1]){
    for(const h of [.9,1.5])box(x+l/2,c+side*5,h,l,.12,.12,M.steel);
    for(let dx=3;dx<l;dx+=8){
      for(const h of [.7,1.5])box(x+dx,c+side*(w/4+2.5),h,.12,w/2-5,.12,M.steel);
      box(x+dx,c+side*5,1.05,.14,.14,2.1,M.steel);
    }
    box(x+l/2,c+side*5.8,.45,l,1.2,.6,M.concrete);
  }
  // Gable planes follow the exterior observed in the recordings; heights are assumed.
  for(const side of [-1,1]){
    const outerY=c+side*(w/2+.8);
    surface([[x-.8,c,r],[x+l+.8,c,r],[x+l+.8,outerY,e],[x-.8,outerY,e]],
      side===1?[0,1,2,0,2,3]:[2,1,0,3,2,0],M.roof,roofs);
    box(x+l/2,outerY,e,l+1.6,.3,.28,M.trim,roofs);
    // Corrugations and roof purlins are economical line geometry.
    for(let dx=0;dx<=l;dx+=3)line([[x+dx,c,r+.03],[x+dx,outerY,e+.03]],'#b6bcb0',roofs);
  }
  box(x+l/2,c,r+.3,l*.9,1.9,.6,M.roof,roofs);
  box(x+l/2,c,r+.65,l*.92,2.5,.12,M.trim,roofs);
  for(const end of [x,x+l]){
    surface([[end,y,e],[end,y+w,e],[end,c,r]], [0,1,2],M.roof,roofs);
    for(const side of [-1,1]){
      const frontWidth=(w-a.aisle_width)/2;
      box(end,c+side*(a.aisle_width/2+frontWidth/2),3.3,.25,frontWidth,6.6,M.walls);
      for(let s=9;s<w/2-3;s+=6)box(end+(end===x?-.16:.16),c+side*s,3.1,.06,4,1.7,M.dark);
    }
    box(end,c,5.8,.3,a.aisle_width,1.4,M.walls);
    // White facade with the same building identifier as the plan.
    const canvas=document.createElement('canvas');canvas.width=256;canvas.height=128;
    const ctx=canvas.getContext('2d');ctx.fillStyle='#d9dbcd';ctx.fillRect(0,0,256,128);ctx.fillStyle='#586559';ctx.font='56px sans-serif';ctx.textAlign='center';ctx.fillText(b.name.replace('축사 ',''),128,84);
    const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
    const sign=new THREE.Mesh(new THREE.PlaneGeometry(8,4),new THREE.MeshStandardMaterial({map:texture}));
    sign.position.copy(V(end+(end===x?-.2:.2),c,8.1));sign.rotation.y=end===x?-Math.PI/2:Math.PI/2;roofs.add(sign);
  }
  // Silo form observed on camera; placement/count is illustrative.
  for(const {x:end,y:sy} of barnSilos(b)){
    cylinder(end,sy,6.5,1.65,1.65,5,M.silo);
    cylinder(end,sy,3.25,1.65,.3,1.5,M.silo);
    cylinder(end,sy,9.25,.25,1.65,.5,M.silo);
    for(const dx of [-1,1])for(const dy of [-1,1])box(end+dx,sy+dy,1.6,.13,.13,3.2,M.steel);
  }
  addLabel(b,r+3);pickVolume(b,r);
}
function facility(f) {
  const {x,y,length:l,width:w,height:h}=f;
  box(x+l/2,y+w/2,h/2,l,w,h,M.walls);
  box(x+l/2,y+w/2,h+.18,l+1,w+1,.35,M.roof,roofs);
  box(x+l/2,y-.04,1.8,4,.1,3.6,M.dark);
  for(let dx=8;dx<l;dx+=9)box(x+dx,y-.07,2.4,3,.12,1.2,M.glass);
  if(f.id==='milking')cylinder(x+9,y+4,2.3,14,14,4.6,M.walls);
  addLabel(f,h+4);pickVolume(f,h);
}
function lamp(x,y){
  box(x,y,4.5,.15,.15,9,M.steel);
  box(x+1.1,y,8.9,2.2,.15,.15,M.steel);
  box(x+2,y,8.8,.8,.4,.18,M.dark);
}
function buildSite(site){
  terrain();trees();
  box(145.5,94,-.02,320,225,.12,M.grass);
  for(const r of world.roads){road(r.points,r.width+.4,M.curb,.055,r.curve);road(r.points,r.width,M.road,.08,r.curve);}
  world.pavedAreas.forEach(pavedArea);
  // Policy guides belong to the overview, not to the robot's RGB observation.
  road([[145.65,5],[145.65,178]],.18,M.lane,.115,null,true);
  for(const p of world.lamps)lamp(p.x,p.y);
  site.barns.forEach(b=>barn(b,site.assumptions));site.facilities.forEach(f=>facility(f));
  const layout=site.facility_layout_frame;
  for(const [name,point] of [['3동 앞쪽 입구',layout.front_entrance],['3동 뒤쪽 · 중앙도로 쪽',layout.rear_entrance],['도로',[layout.rear_entrance[0],layout.facility_lane_y]]]){
    addLabel({id:`diagram-${name}`,name,x:point[0],y:point[1],length:0,width:0,diagramOnly:true},3);
  }
  enrichScene({model,world,box,cylinder,line,M,addLabel});
  flushBoxes();
}

let data, manifest, recordings, world, sim, selected=null, mode='model', walking=false, tour=false, fly=null, cloudIndex=-1;
let yaw=0, pitch=0, dragging=false, lastPointer=null, downPointer=null, tourT=0;
const keys=new Set();
const outline=new THREE.Box3Helper(new THREE.Box3(), '#d6a463');outline.visible=false;scene.add(outline);
function moveCamera(position,target,instant=false){
  if(instant||window.matchMedia('(prefers-reduced-motion: reduce)').matches){camera.position.copy(position);controls.target.copy(target);camera.lookAt(target);fly=null;return;}
  fly={from:camera.position.clone(),fromTarget:controls.target.clone(),position,target,start:performance.now()};
}
function setWalking(value){
  if(mode!=='model')return;
  if(value)sim?.close();
  tour=false;updateTour();fly=null;walking=value;controls.enabled=!value;
  $('walk').textContent=value?'보행 종료 / Esc':'보행 시점 ↗';
  camera.near=value?.1:1;camera.updateProjectionMatrix();
  $('walk').setAttribute('aria-pressed',String(value));
  $('walk-pad').hidden=!value;$('photo-card').hidden=value;$('selection').hidden=true;
  $('help').textContent=value?'WASD 이동 · 드래그 시선 · Shift 빠르게 · Esc 종료 · 충돌 미적용':'드래그 회전 · 스크롤 확대 · 우클릭 이동';
  if(value){
    let pos=V(145.65,-10,1.7),target=V(145.65,80,1.7);
    if(selected){pos=V(selected.x-9,selected.y+selected.width/2,1.7);target=V(selected.x+20,selected.y+selected.width/2,1.7);}
    camera.position.copy(pos);camera.lookAt(target);
    const direction=target.clone().sub(pos).normalize();yaw=Math.atan2(-direction.x,-direction.z);pitch=0;
    $('scene').focus();
  }else{keys.clear();controls.target.copy(camera.position.clone().add(new THREE.Vector3(-Math.sin(yaw)*25,0,-Math.cos(yaw)*25)));camera.position.y=Math.max(camera.position.y,8);}
}
function updateTour(){ $('tour').textContent=tour?'Ⅱ 투어 멈춤':'▷ 부지 투어';$('tour').setAttribute('aria-pressed',String(tour)); }
function overview(){
  sim?.close();$('photo-card').hidden=false;
  if(mode==='cloud')setMode('model');
  if(walking)setWalking(false);tour=false;updateTour();selected=null;outline.visible=false;$('selection').hidden=true;
  document.querySelectorAll('.location-button').forEach(b=>b.classList.toggle('active',b.id==='overview'));
  labelEntries.forEach(l=>l.element.classList.remove('selected'));
  $('facility-select').value='overview';
  moveCamera(V(405,-430,460),V(65,88,0));
}
function selectFacility(item){
  sim?.close();$('photo-card').hidden=false;
  if(mode==='cloud')setMode('model');
  if(walking)setWalking(false);tour=false;updateTour();selected=item;
  $('selection').hidden=false;$('selection-title').textContent=item.name;
  $('selection-info').textContent=`${item.length.toFixed(1)} × ${item.width.toFixed(1)} m\n${item.id.startsWith('barn')?'도면 기반 평면 · 높이·내부 구조 추정':'개략 배치 · 높이·외관 추정'}`;
  document.querySelectorAll('.location-button').forEach(b=>b.classList.toggle('active',b.dataset.id===item.id));
  $('facility-select').value=item.id;
  labelEntries.forEach(l=>l.element.classList.toggle('selected',l.id===item.id));
  outline.box.set(V(item.x-.8,item.y+item.width+.8,0.3),V(item.x+item.length+.8,item.y-.8,item.height||12.8));outline.visible=true;
  const center=V(item.x+item.length/2,item.y+item.width/2,2);
  moveCamera(center.clone().add(new THREE.Vector3(100,100,120)),center);
}
const raycaster=new THREE.Raycaster();
renderer.domElement.addEventListener('pointerdown',event=>{
  downPointer={x:event.clientX,y:event.clientY};
  if(walking){dragging=true;lastPointer=downPointer;renderer.domElement.setPointerCapture(event.pointerId);}
});
renderer.domElement.addEventListener('pointermove',event=>{
  if(walking&&dragging&&lastPointer){yaw-=(event.clientX-lastPointer.x)*.004;pitch=Math.max(-1.25,Math.min(1.25,pitch-(event.clientY-lastPointer.y)*.004));lastPointer={x:event.clientX,y:event.clientY};}
});
renderer.domElement.addEventListener('pointerup',event=>{
  dragging=false;
  if(!walking&&!sim?.active&&mode==='model'&&downPointer&&Math.hypot(event.clientX-downPointer.x,event.clientY-downPointer.y)<5){
    const rect=renderer.domElement.getBoundingClientRect();raycaster.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),camera);
    const hit=raycaster.intersectObjects(pickables)[0];if(hit)selectFacility(hit.object.userData.facility);
  }
  downPointer=null;
});
renderer.domElement.addEventListener('pointercancel',()=>{dragging=false;downPointer=null;});
controls.addEventListener('start',()=>{fly=null;tour=false;updateTour();if(sim?.active)$('sim-follow').checked=false;});
window.addEventListener('keydown',event=>{
  if(event.key==='Escape'&&walking)setWalking(false);
  if(!walking||document.querySelector('dialog[open]'))return;
  if(['w','a','s','d','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Shift'].includes(event.key)){event.preventDefault();keys.add(event.key);}
});
window.addEventListener('keyup',event=>keys.delete(event.key));window.addEventListener('blur',()=>keys.clear());
document.querySelectorAll('[data-move]').forEach(button=>{
  const key={forward:'w',back:'s',left:'a',right:'d'}[button.dataset.move];
  button.addEventListener('pointerdown',e=>{keys.add(key);button.setPointerCapture(e.pointerId);});
  for(const type of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(type,()=>keys.delete(key));
});

let pointsObject=null, cloudRequest=0;
async function showCloud(index){
  const request=++cloudRequest,item=manifest.clouds[index];
  $('cloud-info').textContent='스캔을 불러오고 있습니다…';
  try{
    const points=await loadJson(`./evidence/${item.points}`);if(request!==cloudRequest)return;
    if(pointsObject){cloudGroup.remove(pointsObject);pointsObject.geometry.dispose();pointsObject.material.dispose();}
    const geometry=new THREE.BufferGeometry(),xyz=[],colors=[];
    points.forEach(([x,y,z])=>{xyz.push(x,z,-y);const color=new THREE.Color().setHSL(.57-Math.max(0,Math.min(1,(z+3)/12))*.48,.57,.62);colors.push(color.r,color.g,color.b);});
    geometry.setAttribute('position',new THREE.Float32BufferAttribute(xyz,3));geometry.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));
    pointsObject=new THREE.Points(geometry,new THREE.PointsMaterial({size:.13,vertexColors:true,sizeAttenuation:true}));cloudGroup.add(pointsObject);cloudIndex=index;
    $('cloud-info').textContent=`${fmtTime(item.elapsed_s)} · ${item.frame_id}\n${item.displayed_points.toLocaleString()} / ${item.original_points.toLocaleString()} points`;
  }catch(error){if(request===cloudRequest)$('cloud-info').textContent=`스캔을 읽지 못했습니다: ${error.message}`;}
}
function setMode(value){
  sim?.close();$('photo-card').hidden=false;
  if(walking)setWalking(false);tour=false;updateTour();mode=value;model.visible=value==='model';cloudGroup.visible=value==='cloud';
  viewport.classList.toggle('is-cloud',value==='cloud');
  outline.visible=value==='model'&&!!selected;$('selection').hidden=true;$('cloud-panel').hidden=value!=='cloud';
  $('model-mode').setAttribute('aria-pressed',String(value==='model'));$('cloud-mode').setAttribute('aria-pressed',String(value==='cloud'));
  $('labels').hidden=value==='cloud'||!$('label-toggle').checked;
  $('walk').disabled=value==='cloud';$('tour').disabled=value==='cloud';
  for(const id of ['roof-toggle','label-toggle','green-toggle'])$(id).disabled=value==='cloud';
  $('scene-caption').querySelector('strong').textContent=value==='model'?'당진 축산 단지':'LiDAR · 현장 스캔';
  $('scene-caption').querySelector('span:last-child').textContent=value==='model'?'도면의 평면 배치 + 영상 참고 외관':'개별 센서 프레임 · 건물 모델과 좌표 미정합';
  document.querySelector('.scale').hidden=value==='cloud';document.querySelector('.compass').hidden=value==='cloud';
  scene.background.set(value==='model'?'#dfe5d9':'#182d32');scene.fog=value==='model'?new THREE.Fog('#dfe5d9',1000,1800):null;
  if(value==='cloud'){if(cloudIndex<0)showCloud(0);moveCamera(new THREE.Vector3(25,20,30),new THREE.Vector3(0,0,0));}
  else moveCamera(V(405,-430,460),V(65,88,0));
}
const cloudGrid=new THREE.GridHelper(100,20,'#4c7075','#304c52');cloudGroup.add(cloudGrid);
cloudGroup.add(new THREE.AxesHelper(3));

let frameIndex=0;
function setFrame(index){
  frameIndex=Math.max(0,Math.min(manifest.frames.length-1,index));const frame=manifest.frames[frameIndex];
  $('evidence-photo').src=asset(`./evidence/${frame.image}`);$('evidence-photo').alt=`실제 카메라 장면 ${frameIndex+1}, 기록 시작 ${fmtTime(frame.elapsed_s)}`;
  $('frame-time').textContent=`${String(frameIndex+1).padStart(2,'0')} / ${manifest.frames.length} · ${fmtTime(frame.elapsed_s)}`;
  $('frame-slider').value=frameIndex;$('frame-source').textContent=`${frame.file} · ${frame.frame_id} · ${frame.width}×${frame.height}`;
  $('previous-frame').disabled=frameIndex===0;$('next-frame').disabled=frameIndex===manifest.frames.length-1;
  document.querySelectorAll('.thumbnail').forEach((button,i)=>{button.classList.toggle('active',i===frameIndex);button.setAttribute('aria-pressed',String(i===frameIndex));});
  $('preview-photo').src=asset(`./evidence/${frame.image}`);
}
function openDialog(id){keys.clear();tour=false;updateTour();if(sim?.running)sim.pause();$(id).showModal();}
function connectUI(){
  for(const item of [...data.barns,...data.facilities]){
    const option=document.createElement('option');option.value=item.id;option.textContent=item.name;$('facility-select').append(option);
  }
  $('facility-select').onchange=event=>{if(event.target.value==='rear-road'){$('rear-road').click();return;}const item=[...data.barns,...data.facilities].find(f=>f.id===event.target.value);if(item)selectFacility(item);else overview();};
  for(const b of data.barns){
    const button=document.createElement('button');button.className='location-button';button.dataset.id=b.id;
    button.innerHTML=`<span class="location-icon">${b.id.split('-')[1].padStart(2,'0')}</span><span>${b.name}</span><span class="arrow">↗</span>`;
    button.addEventListener('click',()=>selectFacility(b));$('locations').append(button);
  }
  $('overview').onclick=overview;
  $('entrance').onclick=()=>{if(mode==='cloud')setMode('model');if(walking)setWalking(false);selected=null;outline.visible=false;$('selection').hidden=true;tour=false;updateTour();moveCamera(V(-70,60,190),V(-175,160,0));};
  $('rear-road').onclick=()=>{if(mode==='cloud')setMode('model');if(walking)setWalking(false);selected=null;outline.visible=false;$('selection').hidden=true;tour=false;updateTour();$('facility-select').value='rear-road';document.querySelectorAll('.location-button').forEach(b=>b.classList.toggle('active',b.id==='rear-road'));moveCamera(V(345,255,155),V(224,154,0));};
  $('top-view').onclick=()=>{
    document.body.classList.remove('facility-layout');
    if(walking)setWalking(false);tour=false;updateTour();
    // A small southward offset avoids OrbitControls' zenith singularity and
    // keeps site +Y at the top regardless of the robot's preceding heading.
    if(sim?.active){$('sim-follow').checked=false;moveCamera(V(70,-49,800),V(70,-48,0));}
    else moveCamera(mode==='model'?V(30,84,780):new THREE.Vector3(0,65,.01),mode==='model'?V(30,85,0):new THREE.Vector3());
  };
  $('facility-layout-view').onclick=()=>{
    document.body.classList.add('facility-layout');
    if(mode==='cloud')setMode('model');if(walking)setWalking(false);tour=false;updateTour();
    if(sim?.active)$('sim-follow').checked=false;
    // User's diagram looks from the central-road end toward barn 3's front:
    // site -X is screen up, site -Y is screen left. Keep this frame explicit.
    moveCamera(V(111,-18,470),V(110,-18,0));
  };
  $('fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await viewport.requestFullscreen();}catch{$('help').textContent='이 브라우저에서는 전체 화면을 사용할 수 없습니다.';}};
  $('roof-toggle').onchange=e=>roofs.visible=e.target.checked;
  $('label-toggle').onchange=e=>$('labels').hidden=!e.target.checked;
  $('green-toggle').onchange=e=>vegetation.visible=e.target.checked;
  $('close-selection').onclick=()=>{$('selection').hidden=true;};$('visit').onclick=()=>setWalking(true);$('walk').onclick=()=>setWalking(!walking);
  $('tour').onclick=()=>{if(walking)setWalking(false);fly=null;selected=null;outline.visible=false;$('selection').hidden=true;tour=!tour;tourT=0;updateTour();};
  $('model-mode').onclick=()=>setMode('model');$('cloud-mode').onclick=()=>setMode('cloud');
  manifest.clouds.forEach((item,i)=>{const option=document.createElement('option');option.value=i;option.textContent=`구간 ${i+1} · ${fmtTime(item.elapsed_s)}`;$('cloud-select').append(option);});
  $('cloud-select').onchange=e=>showCloud(Number(e.target.value));
  for(const id of ['open-evidence','evidence-button'])$(id).onclick=()=>{setFrame(frameIndex);openDialog('evidence-dialog');};
  $('source-button').onclick=()=>openDialog('source-dialog');
  $('gps-button').onclick=()=>{drawGPS();openDialog('gps-dialog');};
  $('recording-select').onchange=e=>setRecording(e.target.value);
  document.querySelectorAll('[data-close]').forEach(button=>button.onclick=()=>button.closest('dialog').close());
  document.querySelectorAll('dialog').forEach(dialog=>dialog.addEventListener('click',event=>{if(event.target===dialog){const rect=dialog.getBoundingClientRect();if(event.clientX<rect.left||event.clientX>rect.right||event.clientY<rect.top||event.clientY>rect.bottom)dialog.close();}}));
  $('previous-frame').onclick=()=>setFrame(frameIndex-1);$('next-frame').onclick=()=>setFrame(frameIndex+1);$('frame-slider').max=manifest.frames.length-1;$('frame-slider').oninput=e=>setFrame(Number(e.target.value));
  setRecording('september');
  for(const note of data.source_notes){const row=document.createElement('div');row.className='source-row';row.innerHTML=`<div><strong>${note.label}</strong><small>${note.basis}</small></div><p>${note.detail}</p>`;$('source-notes').append(row);}
  for(const feature of data.observed_features||[]){const row=document.createElement('a');row.href=asset(`./${feature.evidence}`);if(offline)row.download=feature.evidence.split('/').pop();else{row.target='_blank';row.rel='noopener';}row.className='feature-reference';row.innerHTML=`<img loading="lazy" src="${asset(`./${feature.evidence}`)}" alt="${feature.name}"><div><strong>${feature.name}</strong><p>${feature.detail}</p></div>`;$('feature-references').append(row);}
}
function setRecording(key){
  manifest=recordings[key];frameIndex=0;cloudIndex=-1;cloudRequest++;$('thumbnails').replaceChildren();$('cloud-select').replaceChildren();
  manifest.frames.forEach((frame,i)=>{const button=document.createElement('button');button.className='thumbnail';button.setAttribute('aria-label',`장면 ${i+1}, ${fmtTime(frame.elapsed_s)}`);button.innerHTML=`<img loading="lazy" src="${asset(`./evidence/${frame.image}`)}" alt=""><span>${fmtTime(frame.elapsed_s)}</span>`;button.onclick=()=>setFrame(i);$('thumbnails').append(button);});
  manifest.clouds.forEach((item,i)=>{const option=document.createElement('option');option.value=i;option.textContent=`${manifest.recorded_date} · 구간 ${i+1} · ${fmtTime(item.elapsed_s)}`;$('cloud-select').append(option);});
  $('frame-slider').max=manifest.frames.length-1;$('evidence-heading').textContent=`${manifest.files.length}개의 기록, ${manifest.frames.length}개의 시선`;
  $('evidence-count').textContent=`${manifest.frames.length}개 장면 보기 →`;$('open-evidence').setAttribute('aria-label',`현장 기록 ${manifest.frames.length}개 장면 보기`);
  $('recording-caption').textContent=manifest.recorded_date;
  $('evidence-stats').innerHTML=`선택 구간 합계 <strong>${fmtTime(manifest.duration_s||manifest.coverage_s)}</strong><br>${manifest.frames.length}개 사진 · ${manifest.clouds.length}개 센서 스캔<br>${key==='september'?'GPS 63개 표본 · 중간 미제공 구간 있음':'GPS·전역 궤적 미기록'}`;
  $('evidence-description').textContent=key==='september'?'26개 MCAP에서 78개 장면을 추출했습니다. 연석, 철망 울타리, 맨홀, 포장 표면, 작업 장비와 녹색 통과 구조물을 참고했습니다.':'8개 MCAP에서 32개 장면을 추출했습니다. 박공지붕, 붉은 처마, 철골과 사일로를 참고했습니다.';
  setFrame(0);if(mode==='cloud')showCloud(0);
}
function drawGPS(){
  const m=recordings.september,points=m.gps_samples.filter(p=>p.enu_m),c=$('gps-canvas'),ctx=c.getContext('2d');ctx.fillStyle='#eef1e7';ctx.fillRect(0,0,c.width,c.height);
  const xs=points.map(p=>p.enu_m[0]),ys=points.map(p=>p.enu_m[1]),xmin=Math.min(...xs)-20,xmax=Math.max(...xs)+20,ymin=Math.min(...ys)-20,ymax=Math.max(...ys)+20;
  const scale=Math.min((c.width-100)/(xmax-xmin),(c.height-80)/(ymax-ymin));const project=p=>[60+(p[0]-xmin)*scale,c.height-45-(p[1]-ymin)*scale];
  ctx.font='12px sans-serif';ctx.lineWidth=1;ctx.strokeStyle='#d1d9c8';ctx.fillStyle='#75836b';
  for(let x=Math.ceil(xmin/50)*50;x<=xmax;x+=50){const a=project([x,ymin]),b=project([x,ymax]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.fillText(`${x}`,a[0]-8,c.height-20);}
  for(let y=Math.ceil(ymin/50)*50;y<=ymax;y+=50){const a=project([xmin,y]),b=project([xmax,y]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.fillText(`${y}`,20,a[1]+4);}
  ctx.lineWidth=2;ctx.strokeStyle='#416d58';
  points.forEach((p,i)=>{const xy=project(p.enu_m);if(i&&p.elapsed_s-points[i-1].elapsed_s<40){ctx.beginPath();ctx.moveTo(...project(points[i-1].enu_m));ctx.lineTo(...xy);ctx.stroke();}ctx.fillStyle=i===0?'#c17c43':'#416d58';ctx.beginPath();ctx.arc(...xy,i===0?5:3,0,Math.PI*2);ctx.fill();});
  ctx.fillStyle='#445942';ctx.fillText('East (m) →',c.width-120,c.height-20);ctx.fillText('North (m)',20,20);
  const sigmas=points.map(p=>Math.sqrt(Math.max(p.covariance[0],p.covariance[4])));
  $('gps-summary').textContent=`유효 표본 ${points.length}개 · 일반 GPS fix · 수평 σ ${Math.min(...sigmas).toFixed(2)}–${Math.max(...sigmas).toFixed(2)} m (기록 공분산 기준). 도면과 미정합이며 측량 정확도를 보장하지 않습니다.`;
}
function resize(){const width=viewport.clientWidth,height=viewport.clientHeight;camera.aspect=width/height;camera.updateProjectionMatrix();renderer.setSize(width,height);}
new ResizeObserver(resize).observe(viewport);
const tourCurve=new THREE.CatmullRomCurve3([V(-265,235,17),V(-150,142,22),V(-32,27,22),V(72,-27,28),V(145,10,22),V(145,100,24),V(145,197,28),V(310,130,38),V(310,-25,40),V(20,-40,100)],false,'centripetal');
let previous=performance.now();
function animate(now){
  requestAnimationFrame(animate);const dt=Math.min((now-previous)/1000,.05);previous=now;
  if(fly){
    const progress=Math.min(1,(now-fly.start)/1100),ease=progress*progress*(3-2*progress);
    camera.position.lerpVectors(fly.from,fly.position,ease);controls.target.lerpVectors(fly.fromTarget,fly.target,ease);
    if(progress===1)fly=null;
  }
  if(walking&&!document.querySelector('dialog[open]')){
    let forward=Number(keys.has('w')||keys.has('ArrowUp'))-Number(keys.has('s')||keys.has('ArrowDown'));
    let right=Number(keys.has('d')||keys.has('ArrowRight'))-Number(keys.has('a')||keys.has('ArrowLeft'));
    const length=Math.hypot(forward,right)||1,speed=dt*(keys.has('Shift')?18:5);
    forward/=length;right/=length;
    camera.position.x+=(-Math.sin(yaw)*forward+Math.cos(yaw)*right)*speed;
    camera.position.z+=(-Math.cos(yaw)*forward-Math.sin(yaw)*right)*speed;
    camera.position.x=THREE.MathUtils.clamp(camera.position.x,-470,440);camera.position.z=THREE.MathUtils.clamp(camera.position.z,-330,170);
    camera.position.y=terrainHeight(camera.position.x,-camera.position.z)+1.7;
    camera.rotation.order='YXZ';camera.rotation.set(pitch,yaw,0);
  }else if(tour){
    tourT=Math.min(1,tourT+dt/100);camera.position.copy(tourCurve.getPointAt(tourT));
    controls.target.copy(tourCurve.getPointAt(Math.min(.999,tourT+.025)));controls.target.y-=2;
    if(tourT===1){tour=false;updateTour();}
  }
  if(!walking)controls.update();
  sim?.tick(now);
  camera.updateMatrixWorld();
  if(mode==='model'&&!$('labels').hidden){
    for(const entry of labelEntries){
      const projected=entry.position.clone().project(camera),distance=camera.position.distanceTo(entry.position);
      const visible=(!entry.diagramOnly||document.body.classList.contains('facility-layout'))&&projected.z<1&&projected.z>-1&&Math.abs(projected.x)<.96&&Math.abs(projected.y)<.96&&(!walking||distance<100);
      entry.element.hidden=!visible;
      if(visible){entry.element.style.left=`${(projected.x*.5+.5)*viewport.clientWidth}px`;entry.element.style.top=`${(-projected.y*.5+.5)*viewport.clientHeight}px`;}
    }
    const arrow=document.querySelector('.compass-arrow');const direction=new THREE.Vector3();camera.getWorldDirection(direction);arrow.style.transform=`rotate(${Math.atan2(-direction.x,-direction.z)}rad)`;
    // Scale is a projected 50 m segment at the current orbit target depth.
    if(!walking){const a=controls.target.clone().project(camera),b=controls.target.clone().add(new THREE.Vector3(50,0,0)).project(camera);const pixels=Math.hypot((b.x-a.x)*viewport.clientWidth/2,(b.y-a.y)*viewport.clientHeight/2);document.querySelector('.scale>span').style.width=`${pixels}px`;}
  }
  document.querySelector('.scale').hidden=mode==='cloud'||walking;
  renderer.render(scene,camera);
}
async function loadJson(path){const canonical=path.replace('./evidence/../','./');if(offline?.json[canonical])return offline.json[canonical];const response=await fetch(path);if(!response.ok)throw new Error(`${path}: HTTP ${response.status}`);return response.json();}
try{
  let october,september;
  [data,october,september]=await Promise.all([loadJson('./site.json'),loadJson('./evidence/manifest.json'),loadJson('./evidence-september/manifest.json')]);
  // Retain original source directories while sharing the gallery implementation.
  for(const entry of september.frames)entry.image=`../evidence-september/${entry.image}`;
  for(const entry of september.clouds)entry.points=`../evidence-september/${entry.points}`;
  recordings={october,september};manifest=october;world=new WorldGeometry(data);
  buildSite(data);connectUI();resize();camera.position.copy(V(405,-430,460));controls.target.copy(V(65,88,0));controls.update();
  sim=new RobotSimulation({scene,renderer,camera,controls,model,roofs,vegetation,world,onOpen:()=>{if(walking)setWalking(false);setMode('model');fly=null;tour=false;updateTour();$('selection').hidden=true;outline.visible=false;}});
  $('loading').hidden=true;requestAnimationFrame(animate);
}catch(error){$('loading').textContent=`환경을 불러오지 못했습니다: ${error.message}. README의 로컬 서버 실행 방법을 확인해 주세요.`;console.error(error);}
