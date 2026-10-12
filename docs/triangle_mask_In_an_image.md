In a 1456x1088 image, the what is the orientation {0deg, 90 deg} that gives the largest triangle when the the image and the triangle share the same centroid?

Worked for 33s

0° gives the largest equilateral triangle in a 1456 × 1088 image, assuming 0° means the apex points right and 90° means the apex points vertically.

The 0° orientation provides 33.3% more triangle area.

Implication for DASPi
Your original mask radius was:

inline constexpr double maskRadius{
    std::numbers::sqrt3 / 3.0 * sensorHeightValue_
};

This produces a radius of approximately 628.16 pixels, which is ideal for a horizontally pointing equilateral triangle.

When you rotated the mask to point upward, that radius became too large for the sensor height, clipping its apex.

If you want to retain an upward-pointing triangle, use:

inline constexpr double maskRadius{
    sensorHeightValue_ / 2.0
};

That produces a fully contained triangle with radius 544 pixels.

For maximizing DASPi's captured triangular pixel area, I recommend 0° (or 180°), rather than 90° or 270°. However, changing the orientation would also require updating the camera-to-sphere rotations and rechecking the seam geometry.


