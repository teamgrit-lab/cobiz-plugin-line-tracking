import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {WorldGeometry,localToSite,stepPose} from '../environments/dangjin/world-geometry.js';

const site=JSON.parse(await readFile(new URL('../environments/dangjin/site.json',import.meta.url)));
const world=new WorldGeometry(site);
test('central road is excluded for every barn doorway and along its length',()=>{
  for(let y=5;y<=178;y+=1)assert.equal(world.blocked(145.65,y,.65),'no_go_zone');
  for(const y of [27.75,93.75,159.75])assert.notEqual(world.swept({x:133,y},{x:158,y},.65),null);
});
test('swept collision catches crossing the forbidden zone with safe endpoints',()=>{
  assert.equal(world.blocked(145.65,1,.65),null);
  assert.equal(world.blocked(145.65,184,.65),null);
  assert.equal(world.swept({x:145.65,y:1},{x:145.65,y:184},.65),'no_go_zone');
});
test('compost outer road has usable footprint clearance and the nonexistent road is removed',()=>{
  assert.ok(!world.roads.some(r=>r.id==='barn4-rear'));
  assert.equal(world.blocked(205,127,.65),'road_edge');
  for(const name of ['compost-outer']){
    const road=world.roads.find(r=>r.id===name);
    for(let i=10;i<=100;i++){
      const p=road.curve.getPointAt(i/100);
      assert.equal(world.blocked(p.x,-p.z,.65),null,`${name} at ${i}%`);
    }
  }
});
test('closed perimeter permits an entire lap with a 0.65 m footprint',()=>{
  const route=world.perimeterRoute();assert.ok(route.length>1000);
  assert.ok(Math.hypot(route[0][0]-route.at(-1)[0],route[0][1]-route.at(-1)[1])<1e-6);
  for(let i=1;i<route.length;i++){
    const [x,y]=route[i],a=route[i-1];
    assert.equal(world.swept({x:a[0],y:a[1]},{x,y},.65),null,`lap at ${x.toFixed(2)},${y.toFixed(2)}`);
  }
});
test('facilities follow the barn 3 rear frame, not the front entrance',()=>{
  const f=Object.fromEntries(site.facilities.map(f=>[f.id,f]));
  const barn=site.barns.find(b=>b.id==='barn-3'),rear=barn.x+barn.length;
  const center=b=>[b.x+b.length/2,b.y+b.width/2];
  const [mx,my]=center(f.milking),[px,py]=center(f.practice),[sx,sy]=center(f.storage);
  assert.ok(Math.abs(mx-rear)<1e-8&&Math.abs(px-rear)<1e-8);
  assert.ok(my<barn.y&&py<my);
  assert.ok(sx>px);assert.equal(sy,py);
  assert.ok(f.practice.y+f.practice.width<-71);
  assert.ok(f.milking.y-10>-63);
  assert.equal(world.swept({x:100,y:-67},{x:175,y:-67},.65),null);
  assert.deepEqual(site.facility_layout_frame.front_entrance,[0,27.75]);
  assert.deepEqual(site.facility_layout_frame.rear_entrance,[134.3,27.75]);
});
test('facility road joins the external loop and both old barn roads stop at the centre',()=>{
  assert.equal(world.blocked(145.65,-13,.65),'road_edge');
  assert.equal(world.blocked(120,-12,.65),null);
  assert.equal(world.blocked(180,-13,.65),null);
  assert.equal(world.blocked(145.65,-67,.65),null);
  assert.equal(world.blocked(-5,-40,.65),null);
});
test('buildings, off-road movement and injected obstacles are rejected',()=>{
  assert.equal(world.blocked(20,20,.65),'obstacle');
  assert.equal(world.blocked(-100,0,.65),'road_edge');
  world.extraObstacles=[{x:200,y:185,length:4,width:4}];
  assert.equal(world.blocked(202,187,.65),'obstacle');
  world.extraObstacles=[];
});
test('curved-road poles sit outside the carriageway and rendered supports are solid',()=>{
  for(const p of world.lamps.slice(0,7)){
    assert.ok(world.nearestRoad(p.x,p.y).clearance<-.8);
    assert.equal(world.blocked(p.x,p.y,.65),'obstacle');
  }
  assert.equal(world.blocked(-4.8,109.75,.65),'obstacle');
});
test('camera forward/left and positive yaw have the same site handedness',()=>{
  const p={x:10,y:20,heading:Math.PI/2};
  assert.deepEqual(localToSite([4,2],p),[8,24]);
  const next=stepPose(p,{vx:.5,yaw_rate:.1},.2);
  assert.ok(next.x<10&&next.y>20&&next.heading>p.heading);
});
