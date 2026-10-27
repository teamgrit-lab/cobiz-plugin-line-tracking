import * as THREE from 'three';
import {localToSite,stepPose} from './world-geometry.js';

const $=id=>document.getElementById(id);
const W=480,H=270,DT=.5;
const zero={vx:0,vy:0,yaw_rate:0};
const reasonText={lap_complete:'우사 한 바퀴 완료 · 자동 정지',collecting:'인식 학습 표본 수집 중',no_go_zone:'중앙도로 통행 금지',obstacle:'구조물·장비 충돌 방지',road_edge:'도로 경계 도달',path_unavailable:'관측된 중앙 경로 없음',path_low_confidence:'경로 관측 범위 부족',planner_error:'추론 오류',tracking:'도로 중앙 추종 중',tracking_slow_turn:'커브 감속 중',paused:'일시 정지',idle:'주행 준비',backend:'실제 Flyvis 서버 연결 필요'};

function canvas(width,height){const c=document.createElement('canvas');c.width=width;c.height=height;return c;}
function rgbaCanvas(bytes,c){const ctx=c.getContext('2d');const image=ctx.createImageData(c.width,c.height);const stride=c.width*4;for(let y=0;y<c.height;y++)image.data.set(bytes.subarray((c.height-1-y)*stride,(c.height-y)*stride),y*stride);ctx.putImageData(image,0,0);}
const png=c=>c.toDataURL('image/png').split(',')[1];

export class RobotSimulation {
  constructor({scene,renderer,camera,controls,model,roofs,vegetation,world,onOpen}){
    Object.assign(this,{scene,renderer,camera,controls,model,roofs,vegetation,world,onOpen});
    this.active=false;this.running=false;this.busy=false;this.generation=0;this.sequence=0;this.stamp=1;this.next=0;this.distance=0;this.samples=[];this.events=[];this.session=crypto.randomUUID();this.missionKey=crypto.randomUUID();this.scenario='patrol';this.viewYaw=0;this.mission=null;this.navTarget=null;this.pose={...world.site.simulation.start};this.command={...zero};this.lastStatus='idle';
    this.group=new THREE.Group();this.visuals=new THREE.Group();scene.add(this.group,this.visuals);this.group.visible=false;this.visuals.visible=false;
    this.makeRobot();
    this.cubeTarget=new THREE.WebGLCubeRenderTarget(256,{generateMipmaps:false,minFilter:THREE.LinearFilter});
    this.cubeCamera=new THREE.CubeCamera(.05,650,this.cubeTarget);scene.add(this.cubeCamera);
    this.frontCamera=new THREE.PerspectiveCamera(90,W/H,.05,650);
    this.frontTarget=new THREE.WebGLRenderTarget(W,H,{depthBuffer:true});this.panoTarget=new THREE.WebGLRenderTarget(768,384,{depthBuffer:false});
    this.frontTarget.texture.colorSpace=THREE.SRGBColorSpace;this.panoTarget.texture.colorSpace=THREE.SRGBColorSpace;
    this.quadScene=new THREE.Scene();this.quadCamera=new THREE.OrthographicCamera(-1,1,1,-1,0,2);this.quadCamera.position.z=1;
    this.shader=new THREE.ShaderMaterial({uniforms:{environment:{value:this.cubeTarget.texture},panorama:{value:0},basis:{value:new THREE.Matrix3()},aspect:{value:W/H}},
      vertexShader:'varying vec2 vUv; void main(){vUv=uv;gl_Position=vec4(position.xy,0.,1.);}',
      fragmentShader:`precision highp float; varying vec2 vUv; uniform samplerCube environment; uniform int panorama; uniform mat3 basis; uniform float aspect;
        void main(){vec3 d;if(panorama==1){float lon=(vUv.x-.5)*6.283185307;float lat=(vUv.y-.5)*3.141592654;d=vec3(sin(lon)*cos(lat),sin(lat),-cos(lon)*cos(lat));}else{d=normalize(vec3((vUv.x*2.-1.)*aspect,vUv.y*2.-1.,-1.));}
        gl_FragColor=textureCube(environment,basis*d);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
        }`});
    this.quadScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2,2),this.shader));
    this.rgb=canvas(W,H);this.labels=canvas(W,H);this.hazard=canvas(W,H);this.rawBytes=new Uint8Array(W*H*4);this.panoBytes=new Uint8Array(768*384*4);
    this.views=[this.rgb,...Array.from({length:3},()=>canvas(W,H))];
    this.black=new THREE.MeshBasicMaterial({color:0,side:THREE.DoubleSide});this.white=new THREE.MeshBasicMaterial({color:0xffffff,side:THREE.DoubleSide});this.red=new THREE.MeshBasicMaterial({color:0xff0000,side:THREE.DoubleSide});
    this.pathLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:'#66e1c5',depthTest:false}));this.pathLine.renderOrder=10;this.visuals.add(this.pathLine);
    this.trailLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:'#d38b42'}));this.visuals.add(this.trailLine);this.trail=[];
    this.bind();this.reset('patrol');
  }
  makeRobot(){
    const steel=new THREE.MeshStandardMaterial({color:'#283f40',roughness:.6}),orange=new THREE.MeshStandardMaterial({color:'#ddaa58',roughness:.5});
    const add=(geometry,material,x,y,z)=>{const mesh=new THREE.Mesh(geometry,material);mesh.position.set(x,y,z);mesh.castShadow=true;this.group.add(mesh);return mesh;};
    add(new THREE.BoxGeometry(1.1,.27,.52),steel,0,.62,0);add(new THREE.BoxGeometry(.85,.06,.44),orange,0,.79,0);
    this.legs=[];for(const x of [-.39,.39])for(const z of [-.26,.26]){const leg=add(new THREE.BoxGeometry(.12,.5,.12),steel,x,.3,z);this.legs.push(leg);add(new THREE.BoxGeometry(.21,.1,.16),steel,x,.08,z);}
    add(new THREE.CylinderGeometry(.035,.035,.72,8),steel,0,1.15,0);
    add(new THREE.SphereGeometry(.14,12,8),orange,0,1.6,0);
    add(new THREE.SphereGeometry(.065,10,8),steel,.12,1.6,0);add(new THREE.SphereGeometry(.065,10,8),steel,-.12,1.6,0);
  }
  bind(){
    $('sim-panel-toggle').onclick=()=>{
      const collapsed=$('sim-panel-toggle').getAttribute('aria-expanded')==='true';
      $('sim-panel-toggle').setAttribute('aria-expanded',String(!collapsed));
      $('sim-panel-toggle').textContent=collapsed?'로봇 화면 펼치기 ▴':'로봇 화면 접기 ▾';
      $('robot-panel-body').hidden=collapsed;$('robot-panel').classList.toggle('collapsed',collapsed);
    };
    $('robot-mode').onclick=()=>this.open();$('sim-start').onclick=()=>this.running?this.pause():this.start();
    $('sim-reset').onclick=()=>this.reset($('sim-scenario').value);
    $('sim-scenario').onchange=()=>this.reset($('sim-scenario').value);
    $('sim-nudge').onclick=()=>{this.pause();const point=localToSite([0,.6],this.pose);const next={...this.pose,x:point[0],y:point[1]};if(!this.world.swept(this.pose,next,.65)){this.pose=next;this.generation++;this.events.push({t:this.stamp,pose:{...next},reason:'manual_offset'});this.place();this.capture();}};
    $('sim-collect').onclick=()=>this.collect();
    $('sim-obstacle').onchange=e=>this.placeObstacle(e.target.checked);
    $('sim-export').onclick=()=>{const blob=new Blob([JSON.stringify({schema:'dangjin.simulation.trace.v1',model:this.health?.model,perception:this.health?.perception,mission:this.mission,physics:'kinematic',samples:this.samples,events:this.events},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download='dangjin-simulation-trace.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
    document.addEventListener('visibilitychange',()=>{if(document.hidden&&this.running)this.pause();});
  }
  open(){this.onOpen();document.body.classList.add('robot-active');this.active=true;this.group.visible=true;this.visuals.visible=true;$('robot-panel').hidden=false;$('sim-controls').hidden=false;$('photo-card').hidden=true;$('model-mode').setAttribute('aria-pressed','false');$('robot-mode').setAttribute('aria-pressed','true');this.place();this.capture();this.follow(true);this.checkBackend();}
  close(){this.pause();document.body.classList.remove('robot-active');this.active=false;this.group.visible=false;this.visuals.visible=false;$('robot-panel').hidden=true;$('sim-controls').hidden=true;$('robot-mode').setAttribute('aria-pressed','false');}
  status(reason,detail=''){this.lastStatus=reason;$('sim-status').textContent=reasonText[reason]||reason;$('sim-detail').textContent=detail;$('sim-status').dataset.reason=reason;}
  async checkBackend(){
    try{const response=await fetch('/api/status',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error('서버 미연결');const value=await response.json();if(value.model?.backend!=='flyvis'||!value.model.flow_decoder_loaded||!value.perception||value.perception.uses_ground_truth_at_runtime)throw Error('학습된 회로 필요');if(value.layout_revision!==this.world.site.layout_revision)throw Error('지도 변경 후 서버를 다시 시작하세요.');this.health=value;$('sim-backend').textContent=value.perception.backend==='flyvis-road-readout'?`Flyvis ${value.model.version} · 360° 차도 검출`:`RGB SegFormer + Flyvis ${value.model.version}`;$('sim-start').disabled=false;return true;}
    catch{this.health=null;$('sim-backend').textContent='시뮬레이션 서버: localhost:8766';$('sim-start').disabled=true;this.status('backend','README의 시뮬레이션 실행 명령을 사용하세요.');return false;}
  }
  async start(){if(!this.active)this.open();if(!await this.checkBackend())return;this.running=true;this.next=0;$('sim-start').textContent='Ⅱ 일시 정지';this.status('tracking');}
  pause(reason='paused',detail=''){if(this.active)this.events.push({t:this.stamp,pose:{...this.pose},reason,detail});this.running=false;this.command={...zero};this.generation++;$('sim-start').textContent='▶ 추종 시작';this.status(reason,detail);this.updateMetrics();}
  reset(scenario){
    this.pause('idle');this.scenario=scenario;this.missionKey=crypto.randomUUID();this.mission=null;this.navTarget=null;this.viewYaw=0;this.stamp=1;this.distance=0;this.samples=[];this.events=[];this.trail=[];this.sequence=0;this.pathLine.geometry.setFromPoints([]);this.trailLine.geometry.setFromPoints([]);
    $('sim-mask').getContext('2d').clearRect(0,0,$('sim-mask').width,$('sim-mask').height);
    this.world.extraObstacles=[];if(this.equipment){this.visuals.remove(this.equipment);this.equipment=null;}$('sim-obstacle').checked=false;
    const names={bend:['compost-outer',.30],east:['south-east',.92],west:['west-north',.13]};
    if(scenario==='patrol'){const points=this.world.perimeterRoute(),p=points[0],q=points[12];this.pose={x:p[0],y:p[1],heading:Math.atan2(q[1]-p[1],q[0]-p[0])};this.navTarget=points[30];}
    else if(scenario==='blocked')this.pose={x:145.65,y:184,heading:-Math.PI/2};
    else{const [name,t]=names[scenario]||names.bend,road=this.world.roads.find(r=>r.id===name),p=road.curve.getPointAt(t),v=road.curve.getTangentAt(t);this.pose={x:p.x,y:-p.z,heading:Math.atan2(-v.z,v.x)};}
    this.place();this.updateMetrics();if(this.active){this.capture();this.follow(true);}
  }
  placeObstacle(enabled){
    this.pause();this.world.extraObstacles=[];
    if(this.equipment){this.visuals.remove(this.equipment);this.equipment=null;}
    if(enabled){
      const p=localToSite([7,0],this.pose);const b={id:'test-equipment',x:p[0]-2.5,y:p[1]-2.5,length:5,width:5};this.world.extraObstacles=[b];
      const g=new THREE.Group();g.userData.testObstacle=true;
      const body=new THREE.Mesh(new THREE.BoxGeometry(5,1.3,5),new THREE.MeshStandardMaterial({color:'#d79b43'}));body.position.set(p[0],.7,-p[1]);body.userData.semantic='hazard';g.add(body);
      const cab=new THREE.Mesh(new THREE.BoxGeometry(1.8,1.4,1.5),new THREE.MeshStandardMaterial({color:'#334b4b'}));cab.position.set(p[0],2,-p[1]);cab.userData.semantic='hazard';g.add(cab);this.visuals.add(g);this.equipment=g;
    }
    if(this.active)this.capture();
  }
  place(){this.group.position.set(this.pose.x,0,-this.pose.y);this.group.rotation.y=this.pose.heading;}
  follow(instant=false){if(!$('sim-follow').checked)return;const p=localToSite([-13,-13],this.pose),behind=localToSite([-6,0],this.pose),target=new THREE.Vector3(behind[0],0,-behind[1]),eye=new THREE.Vector3(p[0],16,-p[1]);if(instant){this.camera.position.copy(eye);this.controls.target.copy(target);}else{this.camera.position.lerp(eye,.1);this.controls.target.lerp(target,.1);}}
  capture(truth=false){
    const {renderer,scene}=this;const saved={target:renderer.getRenderTarget(),tone:renderer.toneMapping,color:renderer.outputColorSpace,bg:scene.background,fog:scene.fog,model:this.model.visible,roof:this.roofs.visible,veg:this.vegetation.visible,robot:this.group.visible,visuals:this.visuals.visible,shadow:renderer.shadowMap.autoUpdate};
    const x=this.pose.x,y=this.pose.y,h=this.world.site.simulation.camera_height_m;
    this.viewYaw=this.navTarget?Math.max(-1.25,Math.min(1.25,Math.atan2(Math.sin(Math.atan2(this.navTarget[1]-y,this.navTarget[0]-x)-this.pose.heading),Math.cos(Math.atan2(this.navTarget[1]-y,this.navTarget[0]-x)-this.pose.heading)))):0;
    this.cameraPose={...this.pose,heading:this.pose.heading+this.viewYaw};
    this.frontCamera.position.set(x,h,-y);const f=localToSite([10,0],this.cameraPose);this.frontCamera.lookAt(f[0],h,-f[1]);this.frontCamera.updateMatrixWorld();
    this.cubeCamera.position.copy(this.frontCamera.position);
    this.model.visible=true;this.roofs.visible=true;this.vegetation.visible=true;this.group.visible=false;
    // Only planned lines/trail are hidden. A test obstruction is physical evidence.
    this.pathLine.visible=false;this.trailLine.visible=false;this.visuals.visible=true;
    const annotations=[];scene.traverse(obj=>{if(obj.userData.sensorHidden){annotations.push([obj,obj.visible]);obj.visible=false;}});
    try{
      scene.background=new THREE.Color('#c9d9df');scene.fog=new THREE.Fog('#c9d9df',250,600);renderer.shadowMap.autoUpdate=false;
      this.cubeCamera.update(renderer,scene);
      for(let sector=0;sector<(truth?1:4);sector++){
        const aim=localToSite([10,0],{...this.cameraPose,heading:this.cameraPose.heading+sector*Math.PI/2});
        this.frontCamera.lookAt(aim[0],h,-aim[1]);this.frontCamera.updateMatrixWorld();
        this.shader.uniforms.basis.value.setFromMatrix4(this.frontCamera.matrixWorld);
        this.shader.uniforms.panorama.value=0;renderer.setRenderTarget(this.frontTarget);renderer.render(this.quadScene,this.quadCamera);renderer.readRenderTargetPixels(this.frontTarget,0,0,W,H,this.rawBytes);rgbaCanvas(this.rawBytes,this.views[sector]);
      }
      this.frontCamera.lookAt(f[0],h,-f[1]);this.frontCamera.updateMatrixWorld();
      this.shader.uniforms.basis.value.setFromMatrix4(this.frontCamera.matrixWorld);
      this.shader.uniforms.panorama.value=1;renderer.setRenderTarget(this.panoTarget);renderer.render(this.quadScene,this.quadCamera);renderer.readRenderTargetPixels(this.panoTarget,0,0,768,384,this.panoBytes);rgbaCanvas(this.panoBytes,$('sim-panorama'));
      if(truth){
      const restore=[];scene.background=new THREE.Color(0);scene.fog=null;renderer.toneMapping=THREE.NoToneMapping;renderer.outputColorSpace=THREE.LinearSRGBColorSpace;
      scene.traverse(obj=>{
        if(obj.isMesh){restore.push([obj,obj.material,obj.visible]);if(obj.userData.semanticOnly)obj.visible=true;obj.material=obj.userData.semantic==='road'?this.white:obj.userData.semantic==='hazard'?this.red:this.black;}
        else if(obj.isLine||obj.isPoints||obj.isSprite){restore.push([obj,null,obj.visible]);obj.visible=false;}
      });
      try{renderer.setRenderTarget(this.frontTarget);renderer.render(scene,this.frontCamera);renderer.readRenderTargetPixels(this.frontTarget,0,0,W,H,this.rawBytes);}
      finally{for(const [obj,material,visible] of restore){if(material)obj.material=material;obj.visible=visible;}}
      const labelImage=this.labels.getContext('2d').createImageData(W,H),hazardImage=this.hazard.getContext('2d').createImageData(W,H);
      const preview=$('sim-mask').getContext('2d'),previewImage=preview.createImageData(W,H);
      this.maskRoad=new Uint8Array(W*H);this.maskHazard=new Uint8Array(W*H);
      for(let i=0;i<W*H;i++){
        const src=((H-1-Math.floor(i/W))*W+i%W)*4,r=this.rawBytes[src],g=this.rawBytes[src+1];const road=r>180&&g>180?1:0,hazard=r>180&&g<80?1:0;this.maskRoad[i]=road;this.maskHazard[i]=hazard;
        for(let ch=0;ch<3;ch++){labelImage.data[i*4+ch]=road;hazardImage.data[i*4+ch]=hazard;}
        labelImage.data[i*4+3]=hazardImage.data[i*4+3]=255;
        previewImage.data.set(road?[94,167,140,255]:hazard?[209,105,73,255]:[27,48,50,255],i*4);
      }
      this.labels.getContext('2d').putImageData(labelImage,0,0);this.hazard.getContext('2d').putImageData(hazardImage,0,0);preview.putImageData(previewImage,0,0);
      }
      $('sim-forward').getContext('2d').drawImage(this.rgb,0,0);
      this.localConfig=this.calibration();
      return truth?{image_png:png(this.rgb),selected_png:png(this.labels)}:{image_png:png(this.rgb),views_png:this.views.map(png)};
    }finally{
      for(const [obj,visible] of annotations)obj.visible=visible;
      renderer.setRenderTarget(saved.target);renderer.toneMapping=saved.tone;renderer.outputColorSpace=saved.color;renderer.shadowMap.autoUpdate=saved.shadow;scene.background=saved.bg;scene.fog=saved.fog;this.model.visible=saved.model;this.roofs.visible=saved.roof;this.vegetation.visible=saved.veg;this.group.visible=saved.robot;this.visuals.visible=saved.visuals;this.pathLine.visible=true;this.trailLine.visible=true;
    }
  }
  pixel(point){const p=localToSite(point,this.cameraPose||this.pose),v=new THREE.Vector3(p[0],.08,-p[1]).project(this.frontCamera);return [(v.x*.5+.5)*W-.5,(-v.y*.5+.5)*H-.5];}
  calibration(){
    const near=2.5,far=10,half=4.2;
    // Intrinsics in the robot frame stay identical across movement; avoid
    // floating-point world projections changing the server geometry signature.
    const f=H/2,height=this.world.site.simulation.camera_height_m-.08;
    const roi=[[near,half],[near,-half],[far,-half],[far,half]].flatMap(([forward,left])=>[(W/2-.5-f*left/forward)/(W-1),(H/2-.5+f*height/forward)/(H-1)]);
    return {near_distance_m:near,far_distance_m:far,ground_half_width_m:half,search_half_width_m:half,roi_polygon:roi,roi_exclusion_polygon:[],bev_width_px:168,bev_height_px:80};
  }
  async plan(now){
    this.busy=true;const generation=this.generation,pose={...this.pose},stamp=this.stamp,id=`${this.session}:${generation}:${++this.sequence}`;
    const started=performance.now();
    try{
      const pixels=this.capture();
      const request={schema:'line_tracking.connectome.v1',stream_id:`${this.session}:${generation}`,request_id:id,stamp_sec:stamp,requested_modes:[1],local_config:this.localConfig,planner_config:{half_width_m:.3,margin_m:.35,min_length_m:1,max_center_offset_m:.2,flow_weight:.02,road_corridor_smoothing_m:1,road_building_cost_weight:0,road_building_distance_m:2},pose,speed_mps:.8,camera_yaw:this.viewYaw,scenario:this.scenario,mission_id:this.missionKey,...pixels};
      const response=await fetch('/api/step',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(request),signal:AbortSignal.timeout(10000)});
      const reply=await response.json();if(generation!==this.generation||!this.running)return;
      if(!response.ok)throw Error(reply.error||'추론 요청 실패');
      if(reply.request_id!==id||reply.stream_id!==request.stream_id||reply.stamp_sec!==stamp||reply.model?.bundle_sha256!==this.health?.model.bundle_sha256||reply.model?.backend!=='flyvis'||reply.perception?.weights_sha256!==this.health?.perception.weights_sha256)throw Error('프레임 또는 모델 불일치');
      const command=reply.command;
      if(!command||!Number.isFinite(command.vx)||!Number.isFinite(command.yaw_rate)||command.vx<0||command.vx>.800001||Math.abs(command.yaw_rate)>.350001)throw Error('제어 한계 위반');
      const points=reply.paths?.['1']?.points_xy||[];
      const worldPoints=points.map(p=>localToSite(p,this.cameraPose));
      this.mission=reply.mission;this.navTarget=reply.mission?.target_site||null;
      const prediction=new Image();prediction.src='data:image/png;base64,'+(reply.surround_prediction_png||reply.prediction_png);await prediction.decode();if(generation!==this.generation)return;
      const mask=$('sim-mask');mask.width=prediction.width;mask.height=prediction.height;mask.getContext('2d').drawImage(prediction,0,0);
      mask.nextElementSibling.textContent=reply.surround_prediction_png?'360° 차도 검출 · 초록: 통행 추정 / 회색: 미관측':'RGB 차도 추정 · 정답 마스크 미사용';
      this.updateMetrics();
      for(let i=1;i<worldPoints.length;i++){
        const a={x:worldPoints[i-1][0],y:worldPoints[i-1][1]},b={x:worldPoints[i][0],y:worldPoints[i][1]};
        const blocked=this.world.swept(a,b,.65);if(blocked){this.pause(blocked);return;}
      }
      this.pathLine.geometry.dispose();this.pathLine.geometry=new THREE.BufferGeometry().setFromPoints(worldPoints.map(p=>new THREE.Vector3(p[0],.25,-p[1])));
      const overlay=$('sim-forward').getContext('2d');overlay.strokeStyle='#50f6c1';overlay.lineWidth=3;overlay.beginPath();points.forEach((p,i)=>{const [x,y]=this.pixel(p);i?overlay.lineTo(x,y):overlay.moveTo(x,y);});overlay.stroke();
      if(command.vx===0){this.pause(command.reason);return;}
      let next=pose;
      for(let i=0;i<10;i++){const step=stepPose(next,command,DT/10);const reason=this.world.swept(next,step,.65);if(reason){this.pause(reason);return;}next=step;}
      this.command=command;this.distance+=Math.hypot(next.x-pose.x,next.y-pose.y);this.pose=next;this.stamp+=DT;
      const nearest=this.world.nearestRoad(next.x,next.y);const duration=performance.now()-started;
      this.samples.push({t:stamp,pose,next_pose:next,command,center_error_m:nearest.distance,planner_ms:duration,flow_ms:reply.timing.flow_ms,model_inferred_frames:reply.model.inferred_frames,perception:reply.perception_metrics,mission:reply.mission});
      this.trail.push(new THREE.Vector3(next.x,.22,-next.y));if(this.trail.length>5000)this.trail.shift();this.trailLine.geometry.dispose();this.trailLine.geometry=new THREE.BufferGeometry().setFromPoints(this.trail);
      this.place();this.status(command.reason,`${reply.perception_metrics.views===4?'360° 4방향 검출':'차도 인식'} ${reply.perception_metrics.ms.toFixed(0)} ms · 광류 ${reply.timing.flow_ms.toFixed(0)} ms`);this.updateMetrics();
      const rate=Number($('sim-rate').value);this.next=performance.now()+Math.max(0,DT*1000/rate-duration);
    }catch(error){if(generation===this.generation)this.pause('planner_error',error.message);}
    finally{this.busy=false;}
  }
  updateMetrics(){
    $('sim-progress').textContent=this.mission?`${(this.mission.fraction*100).toFixed(1)}% · ${this.mission.progress_m.toFixed(0)} / ${this.mission.length_m.toFixed(0)} m`:this.scenario==='patrol'?'0.0% · 순환 시작점':'구간 시험';
    $('sim-distance').textContent=`${this.distance.toFixed(1)} m`;$('sim-speed').textContent=`${this.command.vx.toFixed(2)} m/s`;
    const n=this.samples.length;const rms=n?Math.sqrt(this.samples.reduce((sum,s)=>sum+s.center_error_m**2,0)/n):0;
    $('sim-error').textContent=`${rms.toFixed(2)} m`;$('sim-frames').textContent=String(n);$('sim-time').textContent=`${((this.stamp-1)).toFixed(1)} s`;
  }
  async collect(){
    this.pause();const saved={...this.pose},nav=this.navTarget;this.navTarget=null;
    const route=this.world.perimeterRoute();$('sim-start').disabled=true;$('sim-collect').disabled=true;
    $('sim-mask').width=W;$('sim-mask').height=H;
    const caption=$('sim-mask').nextElementSibling;caption.textContent='학습 표본용 정답 · 주행 입력과 분리';
    try{
      for(let i=0;i<128;i++){
        const index=Math.floor((i+.37)*route.length/128)%route.length,p=route[index],q=route[(index+8)%route.length],heading=Math.atan2(q[1]-p[1],q[0]-p[0]);
        const offset=((i*7)%11-5)*.17;
        for(let sector=0;sector<4;sector++){
          this.pose={x:p[0]-Math.sin(heading)*offset,y:p[1]+Math.cos(heading)*offset,heading:heading+((i*3)%9-4)*.04+sector*Math.PI/2};
          const pixels=this.capture(true),sample=i*4+sector;
          const response=await fetch('/api/training-frame',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({index:sample,dataset:'connectome360',pose:this.pose,...pixels})});
          if(!response.ok)throw Error('표본 저장 실패');this.status('collecting',`${sample+1} / 512 · 4방향`);
          await new Promise(resolve=>requestAnimationFrame(resolve));
        }
      }
      this.status('idle','4방향 RGB 학습 표본 512개 저장 · 주행 중에는 정답 마스크를 사용하지 않습니다.');
    }catch(error){this.status('planner_error',error.message);}
    finally{this.pose=saved;this.navTarget=nav;this.place();this.capture();caption.textContent='360° 차도 검출 · 초록: 통행 추정 / 회색: 미관측';$('sim-mask').width=768;$('sim-mask').height=384;$('sim-start').disabled=false;$('sim-collect').disabled=false;}
  }
  tick(now){if(!this.active)return;this.follow();if(this.running&&!this.busy&&now>=this.next)this.plan(now);}
}
