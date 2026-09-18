# Swin-L ONNX deployment artifact

Expected local file:

```text
mask2former-swin-l-mapillary-224x384.onnx
```

The model is intentionally ignored by Git. It must be exported from:

- model: `facebook/mask2former-swin-large-mapillary-vistas-semantic`
- revision: `4772b6bf101d91f2534c106dc524d906aeb3c68a`
- input: RGB float32 `[1, 3, 224, 384]`
- preprocessing: scale `1/255`, ImageNet mean/std normalization
- expected outputs:
  - semantic logits `[1, C, H, W]`, or
  - class logits `[1, Q, C+1]` and mask logits `[1, Q, H, W]`

Do not silently replace the checkpoint or preprocessing contract. Validate the
exported graph on the target Jetson/OpenCV build, then record its checksum in
the deployment inventory before enabling robot control.
