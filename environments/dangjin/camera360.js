import * as THREE from 'three';

export function validateCamera360Profile(p){
  if(p?.schema!=='connectum.camera360.v1'||p.source?.projection!=='equirectangular'||
     p.source.width!==2880||p.source.height!==1440||p.views?.width!==480||p.views.height!==270||
     p.views.vertical_fov_deg!==90||JSON.stringify(p.views.order)!=='["front","left","rear","right"]'||
     p.views.interpolation!=='bilinear-srgb'||p.views.pixel_centers!=='half-pixel'||p.alignment?.positive_heading!=='left'||
     ![p.alignment.forward_heading_deg,p.mount?.height_m,p.mount?.pitch_deg,p.mount?.roll_deg].every(Number.isFinite)||
     !(p.mount.height_m>0&&p.mount.height_m<5))throw Error('360° 카메라 입력 프로필이 호환되지 않습니다.');
  return p;
}
function canonical(value){
  if(Array.isArray(value))return value.map(canonical);
  if(value&&typeof value==='object')return Object.fromEntries(Object.keys(value).sort().map(k=>[k,canonical(value[k])]));
  return value;
}
export async function camera360ProfileHash(profile){
  validateCamera360Profile(profile);
  const digest=await crypto.subtle.digest('SHA-256',new TextEncoder().encode(JSON.stringify(canonical(profile))));
  return Array.from(new Uint8Array(digest),v=>v.toString(16).padStart(2,'0')).join('');
}

// CPU reference for the shader and Python projector. Pixel (0,0) means the
// centre of the top-left pixel, not the corner of the image.
export function sourcePixelForView(profile,sector,x,y){
  const right=(x-239.5)/135,up=(134.5-y)/135;
  const heading=(profile.alignment.forward_heading_deg+sector*90)*Math.PI/180;
  const lon=Math.atan2(right,1)-heading,lat=Math.atan2(up,Math.hypot(1,right));
  const u=((lon/(2*Math.PI)+.5)%1+1)%1;
  return [u*2880-.5,(.5-lat/Math.PI)*1440-.5];
}

const vertex='varying vec2 vUv; void main(){vUv=uv;gl_Position=vec4(position.xy,0.,1.);}';

// Both the simulated panorama and real stitched images are sampled as encoded
// sRGB bytes, like surround_camera.py. Do not tone-map the pixels a second time.
export class SurroundCameraCapture {
  constructor(renderer,profile){
    this.renderer=renderer;this.profile=validateCamera360Profile(profile);
    this.cubeTarget=new THREE.WebGLCubeRenderTarget(720,{generateMipmaps:false,minFilter:THREE.LinearFilter});
    this.cubeCamera=new THREE.CubeCamera(.05,650,this.cubeTarget);
    const target=(w,h)=>new THREE.WebGLRenderTarget(w,h,{depthBuffer:false,minFilter:THREE.LinearFilter,magFilter:THREE.LinearFilter});
    this.rawTarget=target(2880,1440);this.rawTarget.texture.wrapS=THREE.RepeatWrapping;
    this.atlasTarget=target(960,540);this.atlasTarget.scissorTest=true;
    this.singleTarget=target(480,270);
    this.scene=new THREE.Scene();this.camera=new THREE.OrthographicCamera(-1,1,1,-1,0,2);this.camera.position.z=1;
    this.rawMaterial=new THREE.ShaderMaterial({uniforms:{environment:{value:this.cubeTarget.texture},basis:{value:new THREE.Matrix3()}},vertexShader:vertex,
      fragmentShader:`precision highp float; varying vec2 vUv; uniform samplerCube environment; uniform mat3 basis;
        vec3 srgb(vec3 c){return mix(12.92*c,1.055*pow(max(c,vec3(0.)),vec3(1./2.4))-.055,step(vec3(.0031308),c));}
        void main(){float lon=(vUv.x-.5)*6.283185307;float lat=(vUv.y-.5)*3.141592654;
          vec3 d=vec3(sin(lon)*cos(lat),sin(lat),-cos(lon)*cos(lat));
          gl_FragColor=textureCube(environment,basis*d);
          #include <tonemapping_fragment>
          gl_FragColor.rgb=srgb(gl_FragColor.rgb);
        }`});
    this.viewMaterial=new THREE.ShaderMaterial({toneMapped:false,uniforms:{panorama:{value:this.rawTarget.texture},heading:{value:0}},vertexShader:vertex,
      fragmentShader:`precision highp float; varying vec2 vUv; uniform sampler2D panorama; uniform float heading;
        void main(){vec2 ray=vec2((vUv.x*2.-1.)*480./270.,vUv.y*2.-1.);
          float lon=atan(ray.x,1.)-heading;float lat=atan(ray.y,sqrt(1.+ray.x*ray.x));
          gl_FragColor=texture2D(panorama,vec2(fract(lon/6.283185307+.5),.5+lat/3.141592654));
        }`});
    this.copyMaterial=new THREE.ShaderMaterial({toneMapped:false,uniforms:{panorama:{value:this.rawTarget.texture}},vertexShader:vertex,
      fragmentShader:'varying vec2 vUv; uniform sampler2D panorama; void main(){gl_FragColor=texture2D(panorama,vUv);}'});
    this.quad=new THREE.Mesh(new THREE.PlaneGeometry(2,2),this.rawMaterial);this.scene.add(this.quad);
  }
  renderSource(scene,frontCamera){
    this.cubeCamera.position.copy(frontCamera.position);this.cubeCamera.updateMatrixWorld();
    this.cubeCamera.update(this.renderer,scene);
    // At the device panorama's centre, heading=0 points robot-right. The user's
    // heading=90 / panorama x=width/4 points robot-forward.
    const rotation=new THREE.Matrix4().makeRotationFromQuaternion(frontCamera.quaternion.clone().multiply(
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),-this.profile.alignment.forward_heading_deg*Math.PI/180)));
    this.rawMaterial.uniforms.basis.value.setFromMatrix4(rotation);
    this.quad.material=this.rawMaterial;this.renderer.setRenderTarget(this.rawTarget);this.renderer.render(this.scene,this.camera);
    this.viewMaterial.uniforms.panorama.value=this.rawTarget.texture;
  }
  renderView(sector,target=this.singleTarget){
    if(!Number.isInteger(sector)||sector<0||sector>3)throw Error('Invalid 360 camera sector');
    this.viewMaterial.uniforms.heading.value=(this.profile.alignment.forward_heading_deg+sector*90)*Math.PI/180;
    this.quad.material=this.viewMaterial;this.renderer.setRenderTarget(target);this.renderer.render(this.scene,this.camera);
    return target;
  }
  renderAtlas(width=480,height=270){
    this.atlasTarget.setSize(2*width,2*height);
    for(let sector=0;sector<4;sector++){
      const x=(sector%2)*width,y=(1-Math.floor(sector/2))*height;
      this.atlasTarget.viewport.set(x,y,width,height);this.atlasTarget.scissor.set(x,y,width,height);
      this.renderView(sector,this.atlasTarget);
    }
    return this.atlasTarget;
  }
  renderPreview(target){
    this.quad.material=this.copyMaterial;this.renderer.setRenderTarget(target);this.renderer.render(this.scene,this.camera);
  }
}
