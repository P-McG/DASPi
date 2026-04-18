# DASPi - Pipeline

In DASPi, a **pipeline** describes the end-to-end flow of data as it moves through a series of processing stages, from capture to final output. Each stage performs a specific task and passes its results forward, enabling efficient, real-time operation.

A typical DASPi pipeline follows this structure:

1. **Acquisition**
Camera hardware captures raw image data (e.g., Bayer frames via libcamera).
2. **Buffer Mapping & Preparation**
Frame buffers are mapped into memory and prepared for processing (e.g., format handling, masking, or bit-depth alignment).
3. **Transformation / Pre-processing**
Data is transformed into a usable form (e.g., masking regions, rearranging pixels, or applying geometric mappings).
4. **Transmission**
Frames are segmented and transmitted across nodes using UDP for low-latency delivery.
5. **Reception & Reassembly**
A receiving node collects packets (e.g., via epoll), reorders them, and reconstructs complete frames.
6. **Processing / Stitching**
Reconstructed frames are processed (e.g., spherical stitching, overlap blending, or analysis).
7. **Output**
Final results are produced, such as stitched images, processed data streams, or visualization outputs.

This pipeline design enables:
- **Parallel execution** of stages for higher throughput
- **Low-latency streaming** for real-time applications
- **Scalability** across multiple compute nodes
- **Modularity**, allowing individual stages to be optimized or replaced independently

In DASPi, the pipeline is the core abstraction that ties together camera capture, network transport, and distributed processing into a cohesive real-time system.

## DASPi – Pipeline (Data Flow)
```
┌───────────────┐
│   Camera HW   │
│ (libcamera)   │
└───────┬───────┘
        │  Raw Bayer Frame
        ▼
┌──────────────────────────┐
│ Buffer Mapping           │
│ (mmap / frame access)    │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Pre-processing           │
│ - Masking                │
│ - Bit-depth alignment    │
│ - Pixel transforms       │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Frame Packetization      │
│ - Chunk into packets     │
│ - Add FrameHeader        │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Transmission             │
│ (UDP send)               │
│ :contentReference[oaicite:0]{index=0} │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Reception                │
│ (epoll UDP listener)     │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Reassembly               │
│ - Reorder packets        │
│ - Detect loss            │
│ - Rebuild frame          │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Processing / Stitching   │
│ - Overlap blending       │
│ - Spherical mapping      │
└───────┬──────────────────┘
        │
        ▼
┌──────────────────────────┐
│ Output                   │
│ - Final image            │
│ - Stream / visualization │
└──────────────────────────┘
```
**Notes**
- Each block represents a **stage in the pipeline**; stages can run in parallel.
- The system is **stream-oriented**: frames continuously flow through all stages.
- **Custom headers + sequencing** allow DASPi to rebuild frames despite UDP being unordered.
- The pipeline is designed for **low latency over reliability**, with recovery handled at the application layer.
