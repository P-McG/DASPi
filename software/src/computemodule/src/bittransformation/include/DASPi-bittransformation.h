        auto BrightenImage(std::vector<uint8_t> &&input);
        //void BrightenImageChunked2(uint8_t *buffer, size_t start, size_t end, size_t Shift);
        void BrightenImageChunked2_NEON(uint8_t* buffer, size_t start, size_t end, size_t shift);
        void BrightenImageChunked2_NEON(uint8_t *buffer, size_t start, size_t end, size_t shift, float scale);
        void BrightenImageChunked2_NEON(uint8_t *buffer, size_t start, size_t end, size_t shift, float scale, size_t imageWidth);
