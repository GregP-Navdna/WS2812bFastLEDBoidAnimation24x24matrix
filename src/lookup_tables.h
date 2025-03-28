#ifndef LOOKUP_TABLES_H
#define LOOKUP_TABLES_H

#include <Arduino.h>

// Size of the sin/cos lookup table (larger = more precision, more memory)
#define SIN_TABLE_SIZE 256

// Size of the sqrt lookup table
#define SQRT_TABLE_SIZE 256 
#define SQRT_TABLE_RANGE 1024  // Maximum value that can be sqrt'd using the table

// Pre-computed sin table for angles 0-359 degrees (mapped to 0-255)
const PROGMEM float sinTable[SIN_TABLE_SIZE] = {
    0.0000f, 0.0245f, 0.0491f, 0.0736f, 0.0980f, 0.1224f, 0.1467f, 0.1710f,
    0.1951f, 0.2191f, 0.2430f, 0.2667f, 0.2903f, 0.3137f, 0.3369f, 0.3599f,
    0.3827f, 0.4052f, 0.4276f, 0.4496f, 0.4714f, 0.4929f, 0.5141f, 0.5350f,
    0.5556f, 0.5758f, 0.5957f, 0.6152f, 0.6344f, 0.6532f, 0.6716f, 0.6895f,
    0.7071f, 0.7242f, 0.7410f, 0.7572f, 0.7730f, 0.7883f, 0.8032f, 0.8176f,
    0.8315f, 0.8449f, 0.8577f, 0.8701f, 0.8819f, 0.8932f, 0.9040f, 0.9142f,
    0.9239f, 0.9330f, 0.9415f, 0.9495f, 0.9569f, 0.9638f, 0.9700f, 0.9757f,
    0.9808f, 0.9853f, 0.9892f, 0.9925f, 0.9952f, 0.9973f, 0.9988f, 0.9997f,
    1.0000f, 0.9997f, 0.9988f, 0.9973f, 0.9952f, 0.9925f, 0.9892f, 0.9853f,
    0.9808f, 0.9757f, 0.9700f, 0.9638f, 0.9569f, 0.9495f, 0.9415f, 0.9330f,
    0.9239f, 0.9142f, 0.9040f, 0.8932f, 0.8819f, 0.8701f, 0.8577f, 0.8449f,
    0.8315f, 0.8176f, 0.8032f, 0.7883f, 0.7730f, 0.7572f, 0.7410f, 0.7242f,
    0.7071f, 0.6895f, 0.6716f, 0.6532f, 0.6344f, 0.6152f, 0.5957f, 0.5758f,
    0.5556f, 0.5350f, 0.5141f, 0.4929f, 0.4714f, 0.4496f, 0.4276f, 0.4052f,
    0.3827f, 0.3599f, 0.3369f, 0.3137f, 0.2903f, 0.2667f, 0.2430f, 0.2191f,
    0.1951f, 0.1710f, 0.1467f, 0.1224f, 0.0980f, 0.0736f, 0.0491f, 0.0245f,
    0.0000f, -0.0245f, -0.0491f, -0.0736f, -0.0980f, -0.1224f, -0.1467f, -0.1710f,
    -0.1951f, -0.2191f, -0.2430f, -0.2667f, -0.2903f, -0.3137f, -0.3369f, -0.3599f,
    -0.3827f, -0.4052f, -0.4276f, -0.4496f, -0.4714f, -0.4929f, -0.5141f, -0.5350f,
    -0.5556f, -0.5758f, -0.5957f, -0.6152f, -0.6344f, -0.6532f, -0.6716f, -0.6895f,
    -0.7071f, -0.7242f, -0.7410f, -0.7572f, -0.7730f, -0.7883f, -0.8032f, -0.8176f,
    -0.8315f, -0.8449f, -0.8577f, -0.8701f, -0.8819f, -0.8932f, -0.9040f, -0.9142f,
    -0.9239f, -0.9330f, -0.9415f, -0.9495f, -0.9569f, -0.9638f, -0.9700f, -0.9757f,
    -0.9808f, -0.9853f, -0.9892f, -0.9925f, -0.9952f, -0.9973f, -0.9988f, -0.9997f,
    -1.0000f, -0.9997f, -0.9988f, -0.9973f, -0.9952f, -0.9925f, -0.9892f, -0.9853f,
    -0.9808f, -0.9757f, -0.9700f, -0.9638f, -0.9569f, -0.9495f, -0.9415f, -0.9330f,
    -0.9239f, -0.9142f, -0.9040f, -0.8932f, -0.8819f, -0.8701f, -0.8577f, -0.8449f,
    -0.8315f, -0.8176f, -0.8032f, -0.7883f, -0.7730f, -0.7572f, -0.7410f, -0.7242f,
    -0.7071f, -0.6895f, -0.6716f, -0.6532f, -0.6344f, -0.6152f, -0.5957f, -0.5758f,
    -0.5556f, -0.5350f, -0.5141f, -0.4929f, -0.4714f, -0.4496f, -0.4276f, -0.4052f,
    -0.3827f, -0.3599f, -0.3369f, -0.3137f, -0.2903f, -0.2667f, -0.2430f, -0.2191f,
    -0.1951f, -0.1710f, -0.1467f, -0.1224f, -0.0980f, -0.0736f, -0.0491f, -0.0245f
};

// Pre-computed square root table for values 0-1023
const PROGMEM float sqrtTable[SQRT_TABLE_SIZE] = {
    0.0000f, 1.0000f, 1.4142f, 1.7321f, 2.0000f, 2.2361f, 2.4495f, 2.6458f,
    2.8284f, 3.0000f, 3.1623f, 3.3166f, 3.4641f, 3.6056f, 3.7417f, 3.8730f,
    4.0000f, 4.1231f, 4.2426f, 4.3589f, 4.4721f, 4.5826f, 4.6904f, 4.7958f,
    4.8990f, 5.0000f, 5.0990f, 5.1962f, 5.2915f, 5.3852f, 5.4772f, 5.5678f,
    5.6569f, 5.7446f, 5.8310f, 5.9161f, 6.0000f, 6.0828f, 6.1644f, 6.2450f,
    6.3246f, 6.4031f, 6.4807f, 6.5574f, 6.6332f, 6.7082f, 6.7823f, 6.8557f,
    6.9282f, 7.0000f, 7.0711f, 7.1414f, 7.2111f, 7.2801f, 7.3485f, 7.4162f,
    7.4833f, 7.5498f, 7.6158f, 7.6811f, 7.7460f, 7.8102f, 7.8740f, 7.9373f,
    8.0000f, 8.0623f, 8.1240f, 8.1854f, 8.2462f, 8.3066f, 8.3666f, 8.4261f,
    8.4853f, 8.5440f, 8.6023f, 8.6603f, 8.7178f, 8.7750f, 8.8318f, 8.8882f,
    8.9443f, 9.0000f, 9.0554f, 9.1104f, 9.1652f, 9.2195f, 9.2736f, 9.3274f,
    9.3808f, 9.4340f, 9.4868f, 9.5394f, 9.5917f, 9.6437f, 9.6954f, 9.7468f,
    9.7980f, 9.8489f, 9.8995f, 9.9499f, 10.0000f, 10.0499f, 10.0995f, 10.1489f,
    10.1980f, 10.2470f, 10.2956f, 10.3441f, 10.3923f, 10.4403f, 10.4881f, 10.5357f,
    10.5830f, 10.6301f, 10.6771f, 10.7238f, 10.7703f, 10.8167f, 10.8628f, 10.9087f,
    10.9545f, 11.0000f, 11.0454f, 11.0905f, 11.1355f, 11.1803f, 11.2250f, 11.2694f,
    11.3137f, 11.3578f, 11.4018f, 11.4455f, 11.4891f, 11.5326f, 11.5758f, 11.6190f,
    11.6619f, 11.7047f, 11.7473f, 11.7898f, 11.8322f, 11.8743f, 11.9164f, 11.9583f,
    12.0000f, 12.0416f, 12.0830f, 12.1244f, 12.1655f, 12.2066f, 12.2474f, 12.2882f,
    12.3288f, 12.3693f, 12.4097f, 12.4499f, 12.4900f, 12.5300f, 12.5698f, 12.6095f,
    12.6491f, 12.6886f, 12.7279f, 12.7671f, 12.8062f, 12.8452f, 12.8841f, 12.9228f,
    12.9615f, 13.0000f, 13.0384f, 13.0767f, 13.1149f, 13.1529f, 13.1909f, 13.2288f,
    13.2665f, 13.3041f, 13.3417f, 13.3791f, 13.4164f, 13.4536f, 13.4907f, 13.5277f,
    13.5647f, 13.6015f, 13.6382f, 13.6748f, 13.7113f, 13.7477f, 13.7840f, 13.8203f,
    13.8564f, 13.8924f, 13.9284f, 13.9642f, 14.0000f, 14.0357f, 14.0712f, 14.1067f,
    14.1421f, 14.1774f, 14.2127f, 14.2478f, 14.2829f, 14.3178f, 14.3527f, 14.3875f,
    14.4222f, 14.4568f, 14.4914f, 14.5258f, 14.5602f, 14.5945f, 14.6287f, 14.6629f,
    14.6969f, 14.7309f, 14.7648f, 14.7986f, 14.8324f, 14.8661f, 14.8997f, 14.9332f,
    14.9666f, 15.0000f, 15.0333f, 15.0665f, 15.0997f, 15.1327f, 15.1658f, 15.1987f,
    15.2315f, 15.2643f, 15.2971f, 15.3297f, 15.3623f, 15.3948f, 15.4272f, 15.4596f,
    15.4919f, 15.5242f, 15.5563f, 15.5885f, 15.6205f, 15.6525f, 15.6844f, 15.7162f
};

// Get sin from lookup table (angle in degrees, normalized to 0-255)
inline float fastSin(uint8_t angle) {
    return pgm_read_float(&sinTable[angle & (SIN_TABLE_SIZE - 1)]);
}

// Get cos from lookup table (angle in degrees, normalized to 0-255)
inline float fastCos(uint8_t angle) {
    return pgm_read_float(&sinTable[((angle + (SIN_TABLE_SIZE / 4)) & (SIN_TABLE_SIZE - 1))]);
}

// Get square root from lookup table
inline float fastSqrt(float value) {
    // Clamp the input value to the table range
    if (value <= 0) return 0;
    if (value >= SQRT_TABLE_RANGE) {
        // For values outside our table, fall back to regular sqrt
        // but this should be rare in our use case
        return sqrt(value);
    }
    
    // Scale the value to fit the table
    uint16_t index = (uint16_t)((value * (SQRT_TABLE_SIZE - 1)) / SQRT_TABLE_RANGE);
    
    // Get the table value
    return pgm_read_float(&sqrtTable[index]);
}

// Fast inverse square root (Quake III algorithm)
inline float fastInvSqrt(float number) {
    // Use the famous Quake III fast inverse square root
    long i;
    float x2, y;
    const float threehalfs = 1.5F;
    
    x2 = number * 0.5F;
    y = number;
    i = *(long*)&y;                 // bit level conversion
    i = 0x5f3759df - (i >> 1);      // what is this magic number?
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // first iteration
    // y = y * (threehalfs - (x2 * y * y)); // second iteration (can be removed)
    
    return y;
}

// Fast atan2 approximation (for angle calculation)
inline float fastAtan2(float y, float x) {
    // Constants
    const float ONEQTR_PI = M_PI / 4.0;
    const float THRQTR_PI = 3.0 * M_PI / 4.0;
    
    // First check for special cases
    if (x == 0.0f) {
        if (y > 0.0f) return M_PI / 2.0f;
        if (y < 0.0f) return -M_PI / 2.0f;
        return 0.0f; // undefined, but return 0
    }
    
    float abs_y = abs(y);
    float angle;
    
    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y);
        angle = ONEQTR_PI - ONEQTR_PI * r;
    } else {
        float r = (x + abs_y) / (abs_y - x);
        angle = THRQTR_PI - ONEQTR_PI * r;
    }
    
    return y < 0.0f ? -angle : angle;
}

// Fast conversion from radians to degrees (0-255)
inline uint8_t fastRadToDegNorm(float rad) {
    // Map 0-2π to 0-255
    return (uint8_t)((rad * 40.7436f) + 0.5f) & 0xFF; // 40.7436 = 255/(2*PI)
}

#endif // LOOKUP_TABLES_H
