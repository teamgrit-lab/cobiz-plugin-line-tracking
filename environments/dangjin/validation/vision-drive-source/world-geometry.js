import * as THREE from './vendor/three.module.js';

class CurveSection extends THREE.Curve {
  constructor(parent,start,end){super();this.parent=parent;this.start=start;this.end=end;}
  getPoint(t,target=new THREE.Vector3()){return this.parent.getPoint(this.start+(this.end-this.start)*t,target);}
}

// One shared road definition for rendering, collision checks and map export.
export function roadDefinitions(site) {
  // Use a single closed curve at the three perimeter joins so curbs, road
  // surfaces and robot clearance all follow a continuous tangent.
  const west=site.outer_road,compost=site.compost_outer_road,south=[...site.south_east_road].reverse();
  const ring=[...west,...compost.slice(1),...south.slice(1,-1)];
  const loop=new THREE.CatmullRomCurve3(ring.map(([x,y])=>new THREE.Vector3(x,0,-y)),true,'centripetal');
  const westEnd=(west.length-1)/ring.length,compostEnd=(west.length+compost.length-2)/ring.length;
  const perimeter={'west-north':new CurveSection(loop,0,westEnd),'compost-outer':new CurveSection(loop,westEnd,compostEnd),'south-east':new CurveSection(loop,1,compostEnd)};
  const roads = [
    ['access',site.access_road,8],['west-north',site.outer_road,8],
    ['south-east',site.south_east_road,8],['central',site.central_road,12],
    ['compost-outer',site.compost_outer_road,7],
    ['south-barn-west-dead-end',site.south_barn_west_dead_end,8],
    ['south-barn-east-dead-end',site.south_barn_east_dead_end,8],
  ];
  for(const y of [27.75,93.75,159.75])roads.push([`west-entry-${y}`,[[-7,y],[134.3,y],[157,y]],8]);
  for(const y of [27.75,93.75])roads.push([`east-entry-${y}`,[[157,y],[297,y]],8]);
  for(const y of [60.75,126.75])roads.push([`west-cross-${y}`,[[-8,y],[138,y]],8]);
  roads.push(['east-cross',[[154,60.75],[297,60.75]],8]);
  return roads.map(([id,points,width])=>{
    const curve=perimeter[id]||new THREE.CatmullRomCurve3(points.map(([x,y])=>new THREE.Vector3(x,0,-y)),false,'centripetal');
    const samples=curve.getPoints(Math.max(30,points.length*18)).map(p=>[p.x,-p.z]);
    return {id,points,width,samples,curve};
  });
}

export function distanceToSegment(x,y,a,b){
  const dx=b[0]-a[0],dy=b[1]-a[1],t=Math.max(0,Math.min(1,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy||1)));
  return Math.hypot(x-a[0]-t*dx,y-a[1]-t*dy);
}
export function rectangleDistance(x,y,r){
  if(r.heading){const cx=r.x+r.length/2,cy=r.y+r.width/2,dx=x-cx,dy=y-cy,c=Math.cos(r.heading),s=Math.sin(r.heading);x=cx+c*dx+s*dy;y=cy-s*dx+c*dy;}
  return Math.hypot(Math.max(r.x-x,0,x-r.x-r.length),Math.max(r.y-y,0,y-r.y-r.width));}
export function barnSilos(b){return [b.x-3.8,b.x+b.length+3.8].map(x=>({x,y:b.y+b.width/2+17}));}
export function lampPositions(roads){
  const roadside=(id,x,y,side)=>{
    const road=roads.find(r=>r.id===id);let best=null;
    for(let i=1;i<road.samples.length;i++){
      const a=road.samples[i-1],b=road.samples[i],dx=b[0]-a[0],dy=b[1]-a[1],length=Math.hypot(dx,dy);
      const t=Math.max(0,Math.min(1,((x-a[0])*dx+(y-a[1])*dy)/(length*length)));
      const px=a[0]+t*dx,py=a[1]+t*dy,distance=Math.hypot(x-px,y-py);
      if(!best||distance<best.distance){const offset=side*(road.width/2+1.5);best={distance,x:px-dy/length*offset,y:py+dx/length*offset,tx:dx/length,ty:dy/length};}
    }
    // Move along the verge if another road joins here, keeping the junction open.
    for(const shift of [0,3,-3,6,-6,9,-9,12,-12]){
      const p={x:best.x+best.tx*shift,y:best.y+best.ty*shift};
      const intersects=roads.some(r=>r.samples.some((b,i)=>i>0&&distanceToSegment(p.x,p.y,r.samples[i-1],b)<r.width/2+1));
      if(!intersects)return p;
    }
    throw new Error(`No clear roadside lamp position near ${x},${y}`);
  };
  // Illustrative poles follow the actual curved verge, not a nominal x value.
  return [...[2,65,130,190].map(y=>roadside('west-north',-6,y,1)),
    ...[8,65,120].map(y=>roadside('south-east',297,y,-1)),
    ...[2,65,130,190].map(y=>({x:137.5,y}))];
}
export function localToSite(point,pose){return [pose.x+point[0]*Math.cos(pose.heading)-point[1]*Math.sin(pose.heading),pose.y+point[0]*Math.sin(pose.heading)+point[1]*Math.cos(pose.heading)];}
export function stepPose(pose,command,dt){
  const mid=pose.heading+command.yaw_rate*dt/2;
  return {x:pose.x+command.vx*Math.cos(mid)*dt,y:pose.y+command.vx*Math.sin(mid)*dt,heading:pose.heading+command.yaw_rate*dt};
}

export class WorldGeometry {
  constructor(site){
    this.site=site;this.roads=roadDefinitions(site);
    const supports=site.barns.flatMap(b=>barnSilos(b).flatMap((s,i)=>[-1,1].flatMap(dx=>[-1,1].map(dy=>({id:`${b.id}-silo-${i}-${dx}-${dy}`,x:s.x+dx-.065,y:s.y+dy-.065,length:.13,width:.13})))));
    this.lamps=lampPositions(this.roads);
    const poles=this.lamps.map((p,i)=>({id:`lamp-${i}`,x:p.x-.075,y:p.y-.075,length:.15,width:.15}));
    const milking=site.facilities.find(f=>f.id==='milking');
    const rotary={id:'milking-rotary',x:milking.x-5,y:milking.y-10,length:28,width:28};
    this.obstacles=[...site.barns,...site.facilities,rotary,...(site.static_obstacles||[]),...supports,...poles];this.zones=site.no_go_zones||[];
    this.extraObstacles=[];
  }
  perimeterRoute(){
    const samples=[];
    for(const [id,reverse] of [['west-north',false],['compost-outer',false],['south-east',true]]){
      const road=this.roads.find(r=>r.id===id),points=road.curve.getSpacedPoints(Math.ceil(road.curve.getLength()/.2)).map(p=>[p.x,-p.z]);
      if(reverse)points.reverse();samples.push(...(samples.length?points.slice(1):points));
    }
    return samples;
  }
  nearestRoad(x,y){
    let best={distance:Infinity,clearance:-Infinity,id:null};
    for(const road of this.roads){
      let distance=Infinity;
      for(let i=1;i<road.samples.length;i++)distance=Math.min(distance,distanceToSegment(x,y,road.samples[i-1],road.samples[i]));
      const clearance=road.width/2-distance;
      if(clearance>best.clearance)best={distance,clearance,id:road.id};
    }
    return best;
  }
  blocked(x,y,radius=0){
    if(!Number.isFinite(x)||!Number.isFinite(y))return 'invalid_pose';
    if(this.zones.some(r=>rectangleDistance(x,y,r)<=radius))return 'no_go_zone';
    if([...this.obstacles,...this.extraObstacles].some(r=>rectangleDistance(x,y,r)<=radius))return 'obstacle';
    if(this.nearestRoad(x,y).clearance<radius)return 'road_edge';
    return null;
  }
  swept(a,b,radius=.65){
    const n=Math.max(1,Math.ceil(Math.hypot(b.x-a.x,b.y-a.y)/.1));
    for(let i=0;i<=n;i++){const t=i/n;const reason=this.blocked(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,radius);if(reason)return reason;}
    return null;
  }
  export(){return {schema:'dangjin.world.v1',layout_revision:this.site.layout_revision,units:'metres',frame:'site_xy',roads:this.roads.map(({id,width,samples})=>({id,width,samples})),perimeter_route:this.perimeterRoute(),obstacles:this.obstacles,no_go_zones:this.zones,robot_radius_m:this.site.simulation.robot_radius_m,provenance:'plan dimensions + visual interpretation; not a registered survey or SLAM map'};}
}
