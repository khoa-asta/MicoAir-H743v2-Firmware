/* Generated from this vehicle's PX4 export. Units match PX4 parameters.
 * Body trim is Rz(yaw) * Ry(pitch) * Rx(roll), applied AFTER calibration.
 * Do not reuse these constants on another physical board.
 */
#ifndef IMU_CALIBRATION_PARAMS_H
#define IMU_CALIBRATION_PARAMS_H

/* Offset gia tốc kế [X, Y, Z], đơn vị m/s^2.
 * Trừ sau khi đổi trục thanh ghi sang hệ hiệu chuẩn PX4 và đổi sang SI.
 * Đây là độ lệch cố định của phép đo, không phải trọng lực cần loại bỏ.
 */
static const float imu_accel_offset_mps2[3] = {
    -0.064559116959571838f, 0.104830786585807800f, 0.039021991193294525f
};

/* Hệ số sửa độ nhạy gia tốc kế [X, Y, Z], không có đơn vị.
 * corrected = (measured - offset) * scale_factor.
 * Hệ số này sửa sai số tỷ lệ thực tế còn lại sau đổi counts bằng dải đo danh định.
 */
static const float imu_accel_scale_factor[3] = {
    0.994386136531829834f, 0.992727577686309814f, 0.994651675224304199f
};

/* Offset con quay hồi chuyển [X, Y, Z], đơn vị rad/s.
 * Trừ trong hệ hiệu chuẩn PX4, trước khi quay sang hệ thân bằng ma trận bên dưới.
 * Đây là độ lệch tốc độ góc, không phải góc nghiêng của drone.
 */
static const float imu_gyro_offset_rad_s[3] = {
    0.001139122643508017f, -0.003373337211087346f, -0.000782614224590361f
};

/* Ma trận quay bù góc lắp đặt cố định, không có đơn vị: body = R * corrected.
 * Áp dụng cho cả accel và gyro SAU các bước đổi trục, đơn vị và offset/scale.
 * Ma trận này chỉ chứa phần trim; không chứa phép đổi trục [-Y, -X, -Z].
 * Hàng 0/1/2: thành phần đầu ra X/Y/Z của hệ thân FRD.
 * Cột 0/1/2: thành phần đầu vào X/Y/Z đã hiệu chuẩn, trước trim.
 * Ví dụ: body_x = R[0][0]*corrected_x + R[0][1]*corrected_y + R[0][2]*corrected_z.
 */
static const float imu_mounting_rotation[3][3] = {
    /*       input X              input Y              input Z          */
    {0.999995246843675f, -0.000018180202544f, 0.003083173614710f}, /* body X */
    {0.000000000000000f, 0.999982615582131f, 0.005896484844335f},  /* body Y */
    {-0.003083227214820f, -0.005896456817421f, 0.999977862508437f}, /* body Z */
};

#endif /* IMU_CALIBRATION_PARAMS_H */
