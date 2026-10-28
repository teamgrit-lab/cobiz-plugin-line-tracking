import * as THREE from 'three';
import {distanceToSegment,polygonClearance} from './world-geometry.js';

export function roadTexture(){
  const c=document.createElement('canvas');c.width=c.height=512;const ctx=c.getContext('2d');
  ctx.fillStyle='#d0cec5';ctx.fillRect(0,0,512,512);
  let seed=5401;const rnd=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
  for(let i=0;i<17000;i++){const shade=Math.floor(90+rnd()*105);ctx.fillStyle=`rgba(${shade},${shade},${shade-5},${.08+rnd()*.18})`;const r=.4+rnd()*2;ctx.fillRect(rnd()*512,rnd()*512,r,r);}
  ctx.strokeStyle='#777b7040';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,4);ctx.lineTo(512,4);ctx.stroke();
  const texture=new THREE.CanvasTexture(c);texture.colorSpace=THREE.SRGBColorSpace;texture.wrapS=texture.wrapT=THREE.RepeatWrapping;texture.anisotropy=4;return texture;
}

export function enrichScene({model,world,box,cylinder,line,M,addLabel}){
  const details=new THREE.Group();model.add(details);
  const excluded=[];
  const curbMaterial=new THREE.MeshStandardMaterial({color:'#c2c0ad',roughness:1});
  const segments=[];
  for(const road of world.roads.filter(r=>['access','west-north','south-east','compost-outer','south-barn-west-dead-end','south-barn-east-dead-end'].includes(r.id))){
    for(const side of [-1,1])for(let i=1;i<road.samples.length;i++){
      const a=road.samples[i-1],b=road.samples[i],dx=b[0]-a[0],dy=b[1]-a[1],l=Math.hypot(dx,dy);
      if(l<.01)continue;
      const offset=side*(road.width/2+.15);
      const x=(a[0]+b[0])/2-dy/l*offset,y=(a[1]+b[1])/2+dx/l*offset;
      // A road union has no curb across its junction. Individual ribbons
      // otherwise leave a raised diagonal barrier through the shared asphalt.
      const junction=world.pavedAreas.some(area=>polygonClearance(x,y,area.points)>-.35)||world.roads.some(other=>other!==road&&other.samples.some((p,j)=>j>0&&distanceToSegment(x,y,other.samples[j-1],p)<other.width/2+.35));
      if(!junction)segments.push({x,y,angle:Math.atan2(dy,dx),length:l+.03});
    }
  }
  const curbs=new THREE.InstancedMesh(new THREE.BoxGeometry(1,1,1),curbMaterial,segments.length);const d=new THREE.Object3D();
  segments.forEach((s,i)=>{d.position.set(s.x,.16,-s.y);d.rotation.set(0,s.angle,0);d.scale.set(s.length,.18,.25);d.updateMatrix();curbs.setMatrixAt(i,d.matrix);});
  curbs.castShadow=true;curbs.receiveShadow=true;details.add(curbs);
  // Wire fence on the outer side of the northern bend, observed in Sept imagery.
  const fenceRoad=world.roads.find(r=>r.id==='compost-outer');const fence=[];
  fenceRoad.samples.forEach((p,i)=>{
    const a=fenceRoad.samples[Math.max(0,i-1)],b=fenceRoad.samples[Math.min(i+1,fenceRoad.samples.length-1)];const dx=b[0]-a[0],dy=b[1]-a[1],l=Math.hypot(dx,dy)||1;
    const offset=fenceRoad.width/2+2.5;
    fence.push([p[0]-dy/l*offset,p[1]+dx/l*offset]);
  });
  for(const h of [.45,.95,1.55,1.85])line(fence.map(p=>[...p,h]),'#777f6b',details);
  let since=10;
  fence.forEach((p,i)=>{if(i)since+=Math.hypot(p[0]-fence[i-1][0],p[1]-fence[i-1][1]);if(since>=4){box(p[0],p[1],1,.09,.09,2,M.steel,details);since=0;}});
  // Thin wire diagonals make a mesh at close viewing distance, with no opaque wall.
  for(let i=0;i<fence.length-4;i+=4){const a=fence[i],b=fence[i+4];line([[...a,.35],[...b,1.8]],'#89917d',details);line([[...a,1.8],[...b,.35]],'#89917d',details);}
  for(const [x,y] of [[-6,29],[-6,94],[-6,160],[65,194],[200,187],[270,141],[297,94],[297,28],[66,-10]]){
    // Flush lids are traversable surface, unlike curbs and upright equipment.
    cylinder(x,y,.11,.42,.42,.035,M.dark,details,24).userData.semantic='road';
    for(const offset of [-.16,0,.16])line([[x-.28,y+offset,.135],[x+.28,y+offset,.135]],'#899180',details);
  }
  // A green framed passage seen in later clips; its exact position is unregistered.
  for(const x of [66,74]){
    box(x,-10,4.2,.18,9,.22,M.steel,details);
    for(const y of [-14.4,-5.6])box(x,y,2.1,.25,.25,4.2,M.steel,details);
  }
  const glass=new THREE.MeshStandardMaterial({color:'#63a799',transparent:true,opacity:.35,side:THREE.DoubleSide,roughness:.7});
  for(const y of [-14.4,-5.6]){
    const mesh=new THREE.Mesh(new THREE.PlaneGeometry(8,3.8),glass);mesh.position.set(70,2.1,-y);details.add(mesh);
  }
  for(const zone of world.zones){
    const g=new THREE.PlaneGeometry(zone.length,zone.width);g.rotateX(-Math.PI/2);
    const mesh=new THREE.Mesh(g,new THREE.MeshBasicMaterial({color:'#bb684a',transparent:true,opacity:.48,depthWrite:false}));
    mesh.position.set(zone.x+zone.length/2,.21,-zone.y-zone.width/2);mesh.userData.semantic='hazard';mesh.userData.sensorHidden=true;model.add(mesh);excluded.push(mesh);
    for(let y=zone.y+5;y<zone.y+zone.width;y+=12){
      line([[zone.x+.5,y,.24],[zone.x+zone.length-.5,y+8,.24]],'#f7cf92',details).userData.sensorHidden=true;
    }
    addLabel(zone,4);
  }
  // Invisible in RGB, solid hazard footprints in the semantic render. Interior
  // roads do not become an accidental shortcut through the open barn doorways.
  for(const b of world.obstacles){
    const mesh=new THREE.Mesh(new THREE.PlaneGeometry(b.length,b.width),new THREE.MeshBasicMaterial({color:'#ff0000',side:THREE.DoubleSide}));
    mesh.rotation.x=-Math.PI/2;mesh.position.set(b.x+b.length/2,.2,-b.y-b.width/2);mesh.visible=false;mesh.userData.semantic='hazard';mesh.userData.semanticOnly=true;model.add(mesh);excluded.push(mesh);
  }
  return {details,excluded};
}
