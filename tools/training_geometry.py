"""Optional offline route supervision; never imported by deployed inference."""
import json
from pathlib import Path
import numpy as np
from camera_trajectory import track_camera_path

def teacher(metadata,route,max_speed=.8):
    pose=metadata['pose'];p=np.array([pose['x'],pose['y']]);heading=pose['heading']
    # Offline reference centerline labels, never called by the deployed actor.
    delta=route-p;nearest=int(np.argmin(np.linalg.norm(delta,axis=1)))
    points=route[(nearest+np.arange(0,61))%len(route)]-p
    c,s=np.cos(heading),np.sin(heading);rotation=np.array([[c,-s],[s,c]])
    local=points@rotation
    command=track_camera_path(local,0,max_speed)
    return np.array([command.vx/max_speed,command.yaw_rate/.35],np.float32)


def load_simulation_world(directory):
    path = Path(directory) / 'world.json'
    if not path.is_file():
        raise ValueError('Optional simulation assets are required for geometry supervision; '
                         'restore the local simulator and set --simulation-dir to its directory')
    return json.loads(path.read_text())
