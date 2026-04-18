# DASPi – UDP (User Datagram Protocol)

**DASPi UDP** uses **UDP** as the transport layer for high-speed, real-time data exchange between distributed DASPi nodes.

In the DASPi architecture, camera and aperture modules stream image data as a sequence of UDP packets rather than using connection-oriented protocols. This avoids handshake and retransmission overhead, enabling **low-latency delivery of high-bandwidth frame data**.

Each transmitted frame is:

- **Partitioned into packets** (to fit within network MTU limits)
- Tagged with a **custom frame header** (e.g., frame ID, packet index, total packets)
- Sent as a burst of UDP datagrams

On the receiving side:
- An **epoll-based UDP listener** efficiently handles incoming packets
- Packets are **buffered and reordered** using their header metadata
- Complete frames are **reassembled** once all expected packets arrive (or partially reconstructed if loss occurs)

This approach enables:
- **Deterministic, low-latency streaming** for real-time imaging pipelines
- **High throughput** on constrained devices (e.g., Raspberry Pi compute modules)
- **Scalable multi-node ingestion**, where multiple cameras stream concurrently

Because UDP does not guarantee delivery or ordering, DASPi implements reliability at the application layer:
- **Frame-level validation** (detect missing packets)
- **Reordering buffers** for out-of-order delivery
- **Loss tolerance strategies** (e.g., partial frame usage or drop policies)

In DASPi, UDP forms the core of the distributed frame pipeline, allowing efficient transmission of raw or processed image data from acquisition nodes to compute and stitching modules.

## DASPi - UDP Setup
- FrameHeader structure
- Packet format
- Ordering / loss handling
- Epoll usage
- Throughput considerations (Pi Zero vs Pi 5)
