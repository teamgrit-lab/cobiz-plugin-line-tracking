// Walking-camera pitch in simulation time. No wall clock or navigation inputs.
// 1.7 Hz is an assumed walking cadence, not a measured A2 IMU calibration.
export class WalkingCameraSway {
  constructor({enabled=true,amplitude_deg=5,frequency_hz=1.7}={}){
    if(typeof enabled!=='boolean'||!Number.isFinite(amplitude_deg)||amplitude_deg<0||amplitude_deg>10||
       !Number.isFinite(frequency_hz)||frequency_hz<.5||frequency_hz>3)throw Error('Invalid camera sway configuration');
    this.config={enabled,amplitude_deg,frequency_hz};this.reset();
  }
  reset(){this.elapsed=0;this.walkingTime=0;this.gain=0;this.pitch=0;}
  setEnabled(enabled){if(typeof enabled!=='boolean')throw Error('Camera sway enabled must be boolean');if(enabled!==this.config.enabled){this.config.enabled=enabled;this.reset();}}
  advance(dt,command){
    if(!Number.isFinite(dt)||dt<0||!['vx','vy','yaw_rate'].every(k=>Number.isFinite(command[k]??0)))throw Error('Finite camera motion required');
    if(!this.config.enabled)return;
    this.elapsed+=dt;
    const moving=Math.hypot(command.vx??0,command.vy??0)>.001||Math.abs(command.yaw_rate??0)>.001;
    if(moving){
      this.walkingTime+=dt;
      this.gain=1-(1-this.gain)*Math.exp(-dt/.15);
      this.pitch=this.config.amplitude_deg*this.gain*Math.sin(2*Math.PI*this.config.frequency_hz*this.walkingTime);
    }else{
      // Stop stepping and settle toward level, without generating new oscillations.
      const decay=Math.exp(-dt/.15);this.gain*=decay;this.pitch*=decay;
      if(this.gain<1e-6){this.gain=0;this.pitch=0;}
    }
  }
  snapshot(){return {schema:'dangjin.camera_sway.v1',...this.config,axis:'pitch',
    pitch_deg:this.config.enabled?this.pitch:0,elapsed_s:this.elapsed,walking_s:this.walkingTime};}
}

// Call after lookAt(): camera local X is its right axis; positive pitch looks up.
export function applyCameraSway(camera,sample){
  if(sample.enabled)camera.rotateX(sample.pitch_deg*Math.PI/180);
  camera.updateMatrixWorld();
}
