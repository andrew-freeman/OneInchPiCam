# Minimal RAW-over-UDP Camera Streaming

This project implements a preview-oriented RAW-over-UDP pipeline with an explicit, debuggable on-wire format. It includes a synthetic-frame sender and a receiver with lightweight ISP-style preview and optional recording.

## Build

```bash
cmake -S . -B build
cmake --build build
```

OpenCV is optional and automatically detected for preview display. The receiver still runs headless without it.

## Protocol

All multi-byte fields are network byte order. The payload immediately follows the header.

```c
struct RawUdpHeader {
    uint32_t magic;           // ASCII 'RAW1' (0x52415731)
    uint16_t version;         // protocol version, start at 1
    uint16_t header_size;     // sizeof(RawUdpHeader)

    uint32_t flow_id;         // camera / stream ID
    uint32_t frame_id;        // monotonically increasing frame counter

    uint32_t fragment_id;     // index of this fragment
    uint32_t fragment_count;  // total fragments for this frame

    uint16_t width;           // image width in pixels
    uint16_t height;          // image height in pixels

    uint8_t  bit_depth;       // 8, 10, 12, 14, 16
    uint8_t  pixel_format;    // 0=MONO, 1=RGGB, 2=BGGR, 3=GRBG, 4=GBRG
    uint8_t  packing;         // 0=UNPACKED_16, 1=PACKED_12 (future)
    uint8_t  reserved;

    uint64_t timestamp_us;    // sender timestamp, microseconds, monotonic

    uint32_t payload_offset;  // byte offset into frame
    uint32_t payload_size;    // bytes in this packet

    uint32_t header_crc32;    // CRC of header (with this field zeroed)
    uint32_t payload_crc32;   // CRC of payload
};
```

### Payload

* UNPACKED_16 little-endian samples.
* For bit_depth < 16, the high bits are unused.
* No packed 12-bit implementation is provided yet; the field is reserved for future work.

## Sender (`rawudp_sender`)

The sender is deterministic and stateless: it fragments each frame independently and uses `sleep_until` for timing. Synthetic patterns remove the need for a physical camera.

```
./build/rawudp_sender [options]
  --dest <ip>            Destination IP (default 127.0.0.1)
  --port <port>          Destination UDP port (default 9000)
  --width <px>           Frame width (default 640)
  --height <px>          Frame height (default 480)
  --bit-depth <bits>     Bit depth (8-16, default 12)
  --pixel-format <fmt>   mono,rggb,bggr,grbg,gbrg (default rggb)
  --pattern <name>       mono, bars, checker, box, file (default mono)
  --payload-size <bytes> Fragment payload size (default 1200)
  --fps <value>          Frames per second (default 30)
  --frames <n>           Number of frames (0 = infinite)
  --flow-id <id>         Flow identifier (default 1)
  --raw-file <path>      Replay raw file instead of synthetic pattern
```

Synthetic patterns:

* `mono` ramp
* `bars` Bayer color bars
* `checker` checkerboard
* `box` moving bright box
* `file` raw file replay (16-bit little-endian)

## Receiver (`rawudp_receiver`)

The receiver validates headers, CRCs, and fragment layout, reassembles frames deterministically, and supports optional preview/recording.

```
./build/rawudp_receiver [options]
  --port <port>           UDP listen port (default 9000)
  --expiration <frames>   Drop incomplete frames after this many newer ones (default 5)
  --headless              Disable preview (still validates and records)
  --view-mode <mode>      mono, green, half, bilinear (default half)
  --black-level <value>   Black level subtraction (default 0)
  --wb <r> <g> <b>        White balance gains (default 1 1 1)
  --gamma <value>         Gamma value (default 1.0)
  --record-dir <path>     Save completed frames as .raw files
  --flow-id <id>          Filter specific flow (default accept all)
```

Preview processing:

* Black level subtract → normalize to 8-bit
* Optional white balance gains and gamma
* View modes:
  * `mono` – grayscale from raw values
  * `green` – green plane only
  * `half` – 2x2 half-resolution debayer
  * `bilinear` – full-resolution bilinear debayer

Recording produces `.raw` files named with frame ID, resolution, and bit depth.

## Inspecting traffic

Use `tcpdump` or Wireshark to inspect packets:

```bash
sudo tcpdump -i lo -XX udp port 9000
```

You should see the `RAW1` magic, explicit fragment metadata, and CRC fields for every packet.

## Extending to real cameras

* Replace the synthetic generator in `src/generator.cpp` with a camera capture path that fills the `std::vector<uint16_t>` frame buffer.
* Keep the fragmentation logic unchanged to preserve the on-wire ABI.
* For sensors that output packed formats, extend the `Packing` enum and adjust payload sizing and preview unpacking accordingly.
