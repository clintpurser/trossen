# Claude Notes for Trossen ViperX-300s Module

## Module Development

### Reloading the module (without restarting viam-server)
```bash
viam module reload-local --cloud-config ~/Downloads/viam-local-vx300s-main.json
```
Note: May get "Gzip: invalid header" error but module might still reload. Check server logs.

### If reload doesn't work, restart server
```bash
pkill viam-server; viam-server -config ~/Downloads/viam-local-vx300s-main.json > /tmp/viam-server.log 2>&1 &
```

### Starting viam-server locally
```bash
viam-server -config ~/Downloads/viam-local-vx300s-main.json > /tmp/viam-server.log 2>&1 &
```

## Kinematics

- Joint limits in kinematics JSON must be in **DEGREES**, not radians!
- Translations are in **millimeters**
- Link IDs must match the 3D model part names for visualizer to work

## 3D Models

- GLB files go in `arm/3d_models/viperx-300s/`
- Model names must match link IDs in kinematics JSON
- STL files from Interbotix are in mm, scale by 0.001 when converting to GLB for meters
- Models loaded at runtime via `VIAM_MODULE_ROOT` env var

## Dynamixel

- Singleton driver shared between arm and gripper via `GetDriver`/`ReleaseDriver`
- Hardware errors (data limit, overload, etc.) are ignored to match Python SDK behavior
