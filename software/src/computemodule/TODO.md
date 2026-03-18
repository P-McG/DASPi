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
