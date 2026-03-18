auto Aperture::BrightenImage(std::vector<uint8_t> &&input){
	log_verbose("[Aperture::BrightenImage2]");
	{
       std::vector<std::thread> workers;
        const size_t total = input.size();      // total number of output pixels
        const size_t chunk = total / processingThreads_;
        
        std::cout << "[Aperture::FrameBufferTransformation2] Branching threads" << std::endl;
        for (size_t i = 0; i < size_t(processingThreads_); ++i) {
            size_t start = i * chunk;
            size_t end = (i == size_t(processingThreads_) - 1) ? total : (i + 1) * chunk;
        
            std::cout << "[Aperture::FrameBufferTransformation2] Starting Brighten thread" << std::endl;
            workers.emplace_back([&, i, start, end]() {
                std::cout << "[Aperture::FrameBufferTransformation2] Setting Brighten thread affinity" << std::endl;
                
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(processingThreads_, &cpuset);
                //pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
                pthread_setname_np(pthread_self(), "Brighten");
                ScopedTimer t(("BrightenImageChunked2Thread " + std::to_string(i)).c_str());
        
                std::cout << "[Thread " << i << "] started, start=" << start << ", end=" << end << std::endl;
        
                try {
                    // Write directly into the correct segment of finalResult
                    std::cout << "[Aperture::BrightenImage2" << std::endl;
                    BrightenImageChunked2_NEON(input.data(), start, end, 6/*IMAGE BRIGHTENING SHIFT VALUE*//*, 1.5, sensorWidthValue_*/);
                } catch (const std::exception& e) {
                    std::cerr << "[Thread " << i << "] exception: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "[Thread " << i << "] unknown exception" << std::endl;
                }
        
                std::cout << "[Thread " << i << "] completed" << std::endl;
            });
        }
        for (auto& t : workers) t.join();
    }
	return std::move(input);
}

void Aperture::BrightenImageChunked2_NEON(uint8_t *buffer, size_t start, size_t end, size_t shift) {
    log_verbose("[Aperture::BrightenImageChunked2_NEON]");
    if (shift > 15) shift = 15;

    const size_t count = end - start;
    uint8_t* data = buffer + start;

    const size_t simdWidth = 8;  // 8x uint8_t per 128-bit NEON register
    size_t i = 0;

    int16x8_t shiftAmount = vdupq_n_s16(static_cast<int16_t>(shift));

    for (; i + simdWidth <= count; i += simdWidth) {
        uint16x8_t pixels = vld1q_u16(data + i);           // load 8 pixels
        uint16x8_t bright = vshlq_u16(pixels, shiftAmount); // shift left
        vst1q_u16(data + i, bright);                       // store back
    }

    // Remainder loop (scalar)
    for (; i < count; ++i) {
        data[i] <<= shift;
    }
}

void Aperture::BrightenImageChunked2_NEON(uint8_t *buffer, size_t start, size_t end, size_t shift, float scale) {
    log_verbose("[Aperture::BrightenImageChunked2_NEON]");
    if (shift > 15) shift = 15;

    const size_t count = end - start;
    uint8_t* data = buffer + start;

    const size_t simdWidth = 8;
    size_t i = 0;

    int16x8_t shiftAmount = vdupq_n_s16(static_cast<int16_t>(shift));
    float32x4_t scaleFactor4 = vdupq_n_f32(scale);
    float32x4_t maxVal = vdupq_n_f32(65535.0f);

    for (; i + simdWidth <= count; i += simdWidth) {
        uint16x8_t pixels = vld1q_u16(data + i);
        uint16x8_t bright = vshlq_u16(pixels, shiftAmount);

        // Split into two 4-element vectors to convert to float
        uint32x4_t low_u32 = vmovl_u16(vget_low_u16(bright));
        uint32x4_t high_u32 = vmovl_u16(vget_high_u16(bright));
        float32x4_t low_f = vcvtq_f32_u32(low_u32);
        float32x4_t high_f = vcvtq_f32_u32(high_u32);

        // Multiply by scale
        low_f = vmulq_f32(low_f, scaleFactor4);
        high_f = vmulq_f32(high_f, scaleFactor4);

        // Clamp to max 65535
        low_f = vminq_f32(low_f, maxVal);
        high_f = vminq_f32(high_f, maxVal);

        // Convert back to int and pack
        uint16x4_t low_u16 = vmovn_u32(vcvtq_u32_f32(low_f));
        uint16x4_t high_u16 = vmovn_u32(vcvtq_u32_f32(high_f));
        vst1q_u16(data + i, vcombine_u16(low_u16, high_u16));
    }

    // Scalar fallback for remainder
    for (; i < count; ++i) {
        uint32_t val = static_cast<uint32_t>(data[i]) << shift;
        val = static_cast<uint32_t>(val * scale);
        data[i] = static_cast<uint8_t>(std::min(val, 65535u));
    }
}

void Aperture::BrightenImageChunked2_NEON(uint8_t *buffer, size_t start, size_t end, size_t shift, float scale, size_t imageWidth) {
    log_verbose("[Aperture::BrightenImageChunked2_NEON]");
    if (shift > 15) shift = 15;

    const size_t count = end - start;
    uint8_t* data = buffer + start;

    const size_t simdWidth = 8;
    size_t i = 0;

    //float32x4_t scaleFactor4 = vdupq_n_f32(scale);
    //float32x4_t maxVal = vdupq_n_f32(65535.0f);

    for (; i + simdWidth <= count; i += simdWidth) {
        uint16x8_t pixels = vld1q_u16(data + i);

        // Compute pixel indices relative to full image
        size_t idx0 = start + i;

        uint16x8_t adjusted = pixels;

        // Process each pixel individually in SIMD lane
        for (int lane = 0; lane < 8; ++lane) {
            size_t idx = idx0 + lane;
            size_t row = idx / imageWidth;
            size_t col = idx % imageWidth;

            bool isGreen = (row + col) % 2 == 1;
            int s = isGreen ? static_cast<int>(shift - 1) : static_cast<int>(shift);
            s = std::max(0, std::min(s, 15));  // clamp to [0, 15]

            // Apply shift and store to temp
            uint8_t val = vgetq_lane_u16(pixels, lane);
            val <<= s;

            // Scale
            float fval = static_cast<float>(val) * scale;
            fval = std::min(fval, 65535.0f);
            val = static_cast<uint8_t>(fval);

            adjusted = vsetq_lane_u16(val, adjusted, lane);
        }

        vst1q_u16(data + i, adjusted);
    }

    // Scalar fallback
    for (; i < count; ++i) {
        size_t idx = start + i;
        size_t row = idx / imageWidth;
        size_t col = idx % imageWidth;

        bool isGreen = (row + col) % 2 == 1;
        int s = isGreen ? static_cast<int>(shift - 1) : static_cast<int>(shift);
        s = std::max(0, std::min(s, 15));

        uint32_t val = static_cast<uint32_t>(data[i]) << s;
        val = static_cast<uint32_t>(val * scale);
        data[i] = static_cast<uint8_t>(std::min(val, 65535u));
    }
}
