#define NOISE_MULTIPLIER 10
#define PI_MOTION_MODEL 3.14159265358979323846

#define ALPHA1 0.001 * NOISE_MULTIPLIER                 // Rotation 
#define ALPHA2 0.001 * NOISE_MULTIPLIER                 // Rotation
#define ALPHA3 0.01 * NOISE_MULTIPLIER                  // Translation x
#define ALPHA4 0.001 * NOISE_MULTIPLIER                 // Translation y

// Orignal
// alpha1: 0.001
// alpha2: 0.001
// alpha3: 0.01                 // Uncertainty on translation
// alpha4: 0.001