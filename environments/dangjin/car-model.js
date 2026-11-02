import * as THREE from 'three';
import {GLTFLoader} from './vendor/GLTFLoader.js';

// CC0 Khronos ToyCar; attribution and source are preserved beside the asset.
export async function loadCarModel(){
  const source=globalThis.DANGJIN_OFFLINE?.assets['./assets/ToyCar.glb']||'./assets/ToyCar.glb';
  const gltf=await new GLTFLoader().loadAsync(source),root=new THREE.Group();
  // Fabric is a display cloth beneath the toy, not part of the vehicle.
  for(const child of [...gltf.scene.children])if(!child.isCamera&&child.name!=='Fabric')root.add(child);
  let bounds=new THREE.Box3().setFromObject(root),size=bounds.getSize(new THREE.Vector3());
  if(size.z>size.x)root.rotation.y=Math.PI/2;
  bounds.setFromObject(root);bounds.getSize(size);
  const vehicle=new THREE.Group();vehicle.add(root);
  vehicle.scale.set(4.3/size.x,1.65/size.y,1.8/size.z);
  bounds.setFromObject(vehicle);const center=bounds.getCenter(new THREE.Vector3());
  vehicle.position.set(-center.x,-bounds.min.y,-center.z);
  root.traverse(obj=>{if(obj.isMesh){obj.userData.semantic='hazard';obj.castShadow=true;
    // Screen-space transmission isn't appropriate inside the sensor cubemap.
    const materials=Array.isArray(obj.material)?obj.material:[obj.material];
    for(const m of materials)if(m.transmission>0){m.transmission=0;m.transparent=true;m.opacity=.55;}
  }});
  return vehicle;
}
