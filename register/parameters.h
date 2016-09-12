//下采样参数
const float VOXEL_GRID_SIZE = 0.01f;

//FPFH特征参数
const double FEATURES_RADIUS = 0.3;

//FPFH allignment parameters
const double SAC_MAX_CORRESPONDENCE_DIST = 0.05;
const double SAC_MIN_SAMPLE_DIST = 0.5;
const int SAC_MAX_ITERATIONS = 1000;

//ICP allingment parameters
const int ICP_MAX_ITERATIONS = 50;
const double ICP_MAX_CORRESPONDENCE_DISTANCE = 0.25;
const double ICP_TRANSFORMATION_EPSILON = 1e-6;
const double ICP_EUCLIDEAN_FITNESS_EPSILON = 1e-5;
const double ICP_MAX_DIST = 0.01;