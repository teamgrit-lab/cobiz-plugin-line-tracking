# Tracking configuration migration

The supported Sidewalk pipeline is MaskFormer R50 with PyTorch. Both the public
`r50-fp16-640x360` and local `r50-finetuned-fp16-640x360` profiles retain their
existing weights, label policies, temporal smoothing and controller settings.

Stop the existing tracking services on the robot before upgrading. From the
repository root, preview and apply the deployment configuration migration:

```bash
python3 tools/migrate_tracking_config.py --env-file .env
python3 tools/migrate_tracking_config.py --env-file .env --write
```

The write creates `.env.before-line-tracking` with mode 0600 and refuses to
overwrite an existing backup. It preserves configured model paths, model revision,
checkpoint manifest SHA-256, camera geometry and control values. Conflicting old
and new keys require an explicit resolution. Retired model selections must be
changed to a supported R50 profile before migration; the tool never chooses a
replacement checkpoint. Shell-exported deployment keys must also be updated.

| Previous interface | Current interface |
| --- | --- |
| `SWIN_L_*` | `LINE_TRACKING_*` |
| `SWIN_L_ENGINE_DIR` | `LINE_TRACKING_MODEL_DIR` |
| `SWIN_L_DEBUG_IMAGE` | `LINE_TRACKING_IMAGE_NAME` |
| `debugging-swin-l` / `test-swin-l` | `debugging-line-tracking` / `test-line-tracking` |
| `/line_tracking/swin_l/local_path` | `/line_tracking/local_path` |
| `/line_tracking/swin_l/metrics` | `/line_tracking/metrics` |
| `swin_l_local_path_debug.py` | `line_tracking_runtime.py` |
| `swin_l_drive_control.py` | `drive_control.py` |
| `swin_l_rosbag_overlay.py` | `rosbag_overlay.py` |
| `best_so_far_runtime.py` | `segmentation_runtime.py` |
| `swin_l_updates` report key | `segmentation_updates` |
| `inference_backend=legacy` | `inference_backend=r50` |

Retired Swin-L profiles, engine preparation/build services, hybrid backend flags
and automatic fallback settings have been removed. FCN Road keeps TensorRT and
its existing model package contract. Update telemetry subscribers and replay
consumers with the table above. The `actual-activate`, task event and Sport request
interfaces retain their names.

The base image stack is unchanged. An already installed Jetson base may be tagged
with the current deployment name before building:

```bash
docker tag cobiz:jetson-swin-l-l4t-r36.5.0 cobiz:jetson-line-tracking-l4t-r36.5.0
docker compose config --quiet
docker compose build actual-activate
```

Stop/remove the old debug container before starting `debugging-line-tracking`.
Recreate `actual-activate` with the same selected Road override when deploying.
Migration changes names, not the selected fine-tuned checkpoint or motion limits.
The previous names appear only in this guide and migration tests/tooling.
