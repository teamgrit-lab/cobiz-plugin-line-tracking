import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../environments/dangjin/vendor/three.module.js';
import {WalkingCameraSway,applyCameraSway} from '../environments/dangjin/camera-sway.js';

const walking={vx:.4,vy:0,yaw_rate:0},stopped={vx:0,vy:0,yaw_rate:0};
test('walking reaches both signs of the requested 5 degree amplitude, including 10 Hz capture',()=>{
  const sway=new WalkingCameraSway(),frames=[];
  for(let i=0;i<500;i++){
    sway.advance(.05,walking);
    assert.ok(Math.abs(sway.snapshot().pitch_deg)<=5);
    if(i%2===1&&i>20)frames.push(sway.snapshot().pitch_deg);
  }
  assert.ok(Math.min(...frames)<-4.99&&Math.max(...frames)>4.99);
});
test('stationary camera stays level; stopping settles without continued stepping',()=>{
  const sway=new WalkingCameraSway();sway.advance(10,stopped);
  assert.equal(sway.snapshot().pitch_deg,0);
  for(let i=0;i<13;i++)sway.advance(.05,walking);
  const phase=sway.snapshot().walking_s;let previous=Math.abs(sway.snapshot().pitch_deg);
  assert.ok(previous>1);
  for(let i=0;i<60;i++){
    sway.advance(.05,stopped);const current=Math.abs(sway.snapshot().pitch_deg);
    assert.ok(current<=previous);previous=current;
  }
  assert.equal(sway.snapshot().pitch_deg,0);assert.equal(sway.snapshot().walking_s,phase);
});
test('replay depends only on integrated motion, not frame rate, and disabling removes all tilt',()=>{
  const a=new WalkingCameraSway(),b=new WalkingCameraSway();
  for(const command of [walking,walking,stopped,walking]){
    a.advance(.1,command);b.advance(.05,command);b.advance(.05,command);
    assert.ok(Math.abs(a.snapshot().pitch_deg-b.snapshot().pitch_deg)<1e-10);
  }
  a.setEnabled(false);a.advance(20,walking);assert.equal(a.snapshot().pitch_deg,0);
  a.setEnabled(true);assert.equal(a.snapshot().walking_s,0);
  a.reset();assert.equal(a.snapshot().pitch_deg,0);
});
test('pitch rotates the actual rendering camera, preserves its position and changes road projection',()=>{
  const camera=new THREE.PerspectiveCamera(90,480/270,.05,650);
  camera.position.set(12,1.6,-5);camera.lookAt(22,1.6,-5);camera.updateMatrixWorld();
  const point=new THREE.Vector3(20,.08,-5),level=point.clone().project(camera);
  const pose=camera.position.clone();
  for(const pitch of [-5,5]){
    camera.lookAt(22,1.6,-5);applyCameraSway(camera,{enabled:true,pitch_deg:pitch});
    const direction=camera.getWorldDirection(new THREE.Vector3());
    assert.ok(Math.abs(Math.asin(direction.y)*180/Math.PI-pitch)<1e-8);
    assert.ok(Math.abs(direction.z)<1e-8);assert.deepEqual(camera.position,pose);
    const tilted=point.clone().project(camera);
    assert.ok(Math.abs(tilted.y-level.y)>0.075,'input road pixels must actually move');
  }
  camera.lookAt(22,1.6,-5);applyCameraSway(camera,{enabled:false,pitch_deg:10});
  assert.ok(point.clone().project(camera).distanceTo(level)<1e-12);
});
test('invalid motion/configuration is rejected instead of producing NaN camera frames',()=>{
  for(const config of [{amplitude_deg:11},{amplitude_deg:NaN},{frequency_hz:0},{enabled:'yes'}])
    assert.throws(()=>new WalkingCameraSway(config));
  const sway=new WalkingCameraSway();
  assert.throws(()=>sway.advance(-1,walking));assert.throws(()=>sway.advance(.1,{vx:NaN}));
});
