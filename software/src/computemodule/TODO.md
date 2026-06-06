# aperturecomputemodule TODO

- [ ] Guard against null `config_` before StreamConfiguration
- [ ] Separate capture thread from UDP send thread
- [ ] Validate sensor dimensions before buffer transforms
- [ ] Reduce repeated camera setup logging
- [ ] Add controlClnt_ and controlPort
- [ ] Leave ReceiveApertureCapture() frame-only.
- [ ] Add ReceiveGainMsg(), HandleGainMsg(), and SendGainReply().
- [ ] Run frame and control in separate threads.
- [ ] Protect any shared gain state with a mutex.

Stage 1:
  Startup map + raw Bayer scatter
  Display as grayscale/debug only

Stage 2:
  Extend startup map with Bayer channel metadata
  Composite into RGB/equirect output on Compute

Stage 3:
  Add weighted blending / hole filling

Stage 4, optional:
  Move demosaic/color conversion to Aperture if Compute still becomes bottleneck
