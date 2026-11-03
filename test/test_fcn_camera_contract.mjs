import test from 'node:test';
import assert from 'node:assert/strict';
import {fcnCameraFormat} from '../environments/dangjin/fcn-contract.js';

function status(width){
  const p={backend:`fcn${width}-360`,input_width:width,input_height:width/2,
    class_grid_shape:[4,width/64,width/32],weights_sha256:'a'};
  return {model:{...p,bundle_sha256:'a'},perception:p,camera_transport:`fcn-atlas-${width}x${width/2}`};
}
test('both native camera resolutions negotiate exact capture dimensions',()=>{
  for(const width of [512,1024])assert.deepEqual(fcnCameraFormat(status(width)),
    {width,height:width/2,grid:[4,width/64,width/32]});
  assert.equal(fcnCameraFormat({model:{backend:'flyvis'}}),null);
});
test('hash, source size, atlas and output-grid mismatches reject capture',()=>{
  for(const mutate of [s=>s.model.bundle_sha256='b',s=>s.perception.input_width=512,
    s=>s.perception.input_height=256,s=>s.camera_transport='fcn-atlas-512x256',
    s=>s.perception.class_grid_shape=[4,8,16]]){
    const s=status(1024);mutate(s);assert.throws(()=>fcnCameraFormat(s));
  }
});
