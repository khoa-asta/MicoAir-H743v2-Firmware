/**
 *  @brief Du doan IMU, cap nhat gravity, tu ke va khi ap.
 */
#include "estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define N ESTIMATOR_ERROR_SIZE
#define THETA 0
#define VELOCITY 3
#define POSITION 6
#define GYRO_BIAS 9
#define ACCEL_BIAS 12
#define GRAVITY 9.80665f
#define PI 3.14159265358979323846f

/* Kiem tra du lieu truoc khi thay doi trang thai. */
static bool VectorValid(const float v[3], float limit)
{
    return v != NULL && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]) && fabsf(v[0]) <= limit && fabsf(v[1]) <= limit && fabsf(v[2]) <= limit;
}

static float VectorNorm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* skew(v) * u = v cross u. */
static void Skew(const float v[3], float s[9])
{
    s[0] = 0.0f;
    s[1] = -v[2];
    s[2] = v[1];
    s[3] = v[2];
    s[4] = 0.0f;
    s[5] = -v[0];
    s[6] = -v[1];
    s[7] = v[0];
    s[8] = 0.0f;
}

static void Identity(float a[N][N])
{
    memset(a, 0, sizeof(float) * N * N);
    for (int i = 0; i < N; ++i)
    {
        a[i][i] = 1.0f;
    }
}

/* out = A * P * A^T. out duoc phep trung p. */
static void TransformCovariance(Estimator_t *est, const float p[N][N], float out[N][N])
{
    memset(est->temp, 0, sizeof(est->temp));
    for (int i = 0; i < N; ++i)
    {
        for (int k = 0; k < N; ++k)
        {
            const float a = est->matrix[i][k];
            if (a == 0.0f)
            {
                continue;
            }
            for (int j = 0; j < N; ++j)
            {
                est->temp[i][j] += a * p[k][j];
            }
        }
    }
    for (int i = 0; i < N; ++i)
    {
        for (int j = i; j < N; ++j)
        {
            float value = 0.0f;
            for (int k = 0; k < N; ++k)
            {
                value += est->temp[i][k] * est->matrix[j][k];
            }
            out[i][j] = value;
            out[j][i] = value;
        }
    }
}

/* Chi commit covariance huu han voi duong cheo duong. */
static bool CovarianceValid(const float p[N][N])
{
    for (int i = 0; i < N; ++i)
    {
        if (!(p[i][i] > 0.0f))
        {
            return false;
        }
        for (int j = 0; j < N; ++j)
        {
            if (!isfinite(p[i][j]))
            {
                return false;
            }
        }
    }
    return true;
}

/* Cap nhat 1..3 phep do co nhieu doc lap, cung mot diem tuyen tinh hoa. */
static bool Correct(Estimator_t *est, const float residual[3], const float h[3][N], int count, float variance)
{
    float dx[N] = {0.0f};
    float ph[N];
    float gain[N];
    float v_new[3];
    float p_new[3];
    float bg_new[3];
    float ba_new[3];
    float skew[9];
    Quaternion_t q_new;

    /* Gate moi thanh phan tren cung prior; khong sua state khi bi loai. */
    est->last_nis = 0.0f;
    for (int row = 0; row < count; ++row)
    {
        float innovation_variance = variance;
        for (int i = 0; i < N; ++i)
        {
            float value = 0.0f;
            for (int j = 0; j < N; ++j)
            {
                value += est->p[i][j] * h[row][j];
            }
            innovation_variance += h[row][i] * value;
        }
        if (!isfinite(innovation_variance) || innovation_variance <= 0.0f)
        {
            est->healthy = false;
            return false;
        }
        const float nis = residual[row] * residual[row] / innovation_variance;
        est->last_nis = fmaxf(est->last_nis, nis);
        if (!isfinite(nis) || nis > est->config.innovation_gate * est->config.innovation_gate)
        {
            return false;
        }
    }

    memcpy(est->work_p, est->p, sizeof(est->p));
    for (int row = 0; row < count; ++row)
    {
        float innovation = residual[row];
        float s = variance;
        for (int i = 0; i < N; ++i)
        {
            ph[i] = 0.0f;
            for (int j = 0; j < N; ++j)
            {
                ph[i] += est->work_p[i][j] * h[row][j];
            }
            s += h[row][i] * ph[i];
            innovation -= h[row][i] * dx[i];
        }
        if (!isfinite(s) || s <= 0.0f)
        {
            est->healthy = false;
            return false;
        }
        for (int i = 0; i < N; ++i)
        {
            gain[i] = ph[i] / s;
            dx[i] += gain[i] * innovation;
        }
        /* Joseph dang scalar: P - K*PH^T - PH*K^T + K*S*K^T. */
        for (int i = 0; i < N; ++i)
        {
            for (int j = i; j < N; ++j)
            {
                const float value = est->work_p[i][j] - gain[i] * ph[j] - ph[i] * gain[j] + gain[i] * s * gain[j];
                est->work_p[i][j] = value;
                est->work_p[j][i] = value;
            }
        }
    }

    /* Tu choi buoc sua qua lon so voi mo hinh goc nho. */
    if (!VectorValid(dx, 0.5f) || VectorNorm(dx) > 0.5f)
    {
        return false;
    }
    if (!Quaternion_ApplyGlobalError(&est->q_nb, dx, &q_new))
    {
        est->healthy = false;
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        v_new[i] = est->v_n[i] + dx[VELOCITY + i];
        p_new[i] = est->p_n[i] + dx[POSITION + i];
        bg_new[i] = est->bg[i] + dx[GYRO_BIAS + i];
        ba_new[i] = est->ba[i] + dx[ACCEL_BIAS + i];
    }
    if (!VectorValid(v_new, 1.0e6f) || !VectorValid(p_new, 1.0e8f) || !VectorValid(bg_new, 5.0f) || !VectorValid(ba_new, 30.0f))
    {
        est->healthy = false;
        return false;
    }

    /* Reset sai so GLOBAL: J_theta = I + 0.5 * skew(dx_theta). */
    Identity(est->matrix);
    Skew(dx, skew);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            est->matrix[i][j] += 0.5f * skew[3 * i + j];
        }
    }
    TransformCovariance(est, est->work_p, est->work_p);
    if (!CovarianceValid(est->work_p))
    {
        est->healthy = false;
        return false;
    }
    est->q_nb = q_new;
    memcpy(est->v_n, v_new, sizeof(v_new));
    memcpy(est->p_n, p_new, sizeof(p_new));
    memcpy(est->bg, bg_new, sizeof(bg_new));
    memcpy(est->ba, ba_new, sizeof(ba_new));
    memcpy(est->p, est->work_p, sizeof(est->p));
    /* dx la bien cuc bo, lan cap nhat sau bat dau tu 0. */
    return true;
}

void Estimator_DefaultConfig(Estimator_Config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    config->gyro_noise = 0.003f;
    config->accel_noise = 0.08f;
    config->gyro_bias_walk = 0.0001f;
    config->accel_bias_walk = 0.003f;
    config->gravity_std = 0.8f;
    config->mag_std_ut = 2.0f;
    config->baro_std_m = 0.8f;
    config->accel_norm_gate = 1.2f;
    config->mag_norm_gate = 0.25f;
    config->innovation_gate = 5.0f;
    config->declination_rad = 0.0f;
    config->max_dt_s = 0.01f;
}

bool Estimator_Init(Estimator_t *est, const Estimator_Config_t *config, const float accel_mps2[3], const float gyro_rad_s[3], const float mag_body_ut[3], float altitude_m)
{
    Quaternion_t q;
    float mag_n[3] = {0.0f};
    float mag_norm = 0.0f;
    float roll;
    float pitch;
    float yaw = 0.0f;
    float values[11];
    Estimator_Config_t config_copy;

    if (est == NULL || config == NULL || !VectorValid(accel_mps2, 300.0f) || !VectorValid(gyro_rad_s, 0.15f) || !isfinite(altitude_m) || fabsf(altitude_m) > 20000.0f)
    {
        return false;
    }
    values[0] = config->gyro_noise;
    values[1] = config->accel_noise;
    values[2] = config->gyro_bias_walk;
    values[3] = config->accel_bias_walk;
    values[4] = config->gravity_std;
    values[5] = config->mag_std_ut;
    values[6] = config->baro_std_m;
    values[7] = config->accel_norm_gate;
    values[8] = config->mag_norm_gate;
    values[9] = config->innovation_gate;
    values[10] = config->max_dt_s;
    for (int i = 0; i < 11; ++i)
    {
        if (!isfinite(values[i]) || values[i] <= 0.0f || values[i] > 100.0f)
        {
            return false;
        }
    }
    if (!isfinite(config->declination_rad) || fabsf(config->declination_rad) > PI || config->max_dt_s > 0.02f || fabsf(VectorNorm(accel_mps2) - GRAVITY) > config->accel_norm_gate)
    {
        return false;
    }

    roll = atan2f(-accel_mps2[1], -accel_mps2[2]);
    pitch = atan2f(accel_mps2[0], hypotf(accel_mps2[1], accel_mps2[2]));
    if (!Quaternion_FromEulerZYX(roll, pitch, 0.0f, &q))
    {
        return false;
    }
    if (mag_body_ut != NULL)
    {
        if (!VectorValid(mag_body_ut, 100.0f))
        {
            return false;
        }
        mag_norm = VectorNorm(mag_body_ut);
        if (mag_norm < 10.0f || mag_norm > 100.0f || !Quaternion_RotateVector(&q, mag_body_ut, mag_n) || hypotf(mag_n[0], mag_n[1]) < 5.0f)
        {
            return false;
        }
        yaw = atan2f(-mag_n[1], mag_n[0]) + config->declination_rad;
        if (!Quaternion_FromEulerZYX(roll, pitch, yaw, &q) || !Quaternion_RotateVector(&q, mag_body_ut, mag_n))
        {
            return false;
        }
    }

    config_copy = *config;
    memset(est, 0, sizeof(*est));
    est->config = config_copy;
    est->q_nb = q;
    est->baro_reference_m = altitude_m;
    est->mag_enabled = mag_body_ut != NULL;
    est->mag_reference_norm = mag_norm;
    memcpy(est->mag_reference_n, mag_n, sizeof(mag_n));
    memcpy(est->bg, gyro_rad_s, sizeof(est->bg));
    memcpy(est->last_gyro, gyro_rad_s, sizeof(est->last_gyro));
    for (int i = 0; i < 3; ++i)
    {
        est->p[THETA + i][THETA + i] = 0.0872665f * 0.0872665f;
        est->p[VELOCITY + i][VELOCITY + i] = 0.25f;
        est->p[POSITION + i][POSITION + i] = 1.0f;
        est->p[GYRO_BIAS + i][GYRO_BIAS + i] = 0.03f * 0.03f;
        est->p[ACCEL_BIAS + i][ACCEL_BIAS + i] = 0.5f * 0.5f;
    }
    est->p[2][2] = est->mag_enabled ? 0.174533f * 0.174533f : 1.047198f * 1.047198f;
    est->mag_age_s = est->mag_enabled ? 0.0f : 1000.0f;
    est->baro_age_s = 1000.0f;
    est->initialized = true;
    est->healthy = true;
    return true;
}

bool Estimator_Predict(Estimator_t *est, const float gyro_rad_s[3], const float accel_mps2[3], float dt_s)
{
    float r[9];
    float skew[9];
    float omega[3];
    float force_b[3];
    float force_n[3];
    float v_new[3];
    float p_new[3];
    Quaternion_t q_new;

    if (est == NULL || !est->initialized || !est->healthy || !VectorValid(gyro_rad_s, 40.0f) || !VectorValid(accel_mps2, 300.0f) || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > est->config.max_dt_s)
    {
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        omega[i] = gyro_rad_s[i] - est->bg[i];
        force_b[i] = accel_mps2[i] - est->ba[i];
    }
    if (!Quaternion_ToRotationMatrix(&est->q_nb, r) || !Quaternion_RotateVector(&est->q_nb, force_b, force_n) || !Quaternion_IntegrateBodyRate(&est->q_nb, omega, dt_s, &q_new))
    {
        est->healthy = false;
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        const float acceleration = force_n[i] + ((i == 2) ? GRAVITY : 0.0f);
        v_new[i] = est->v_n[i] + acceleration * dt_s;
        p_new[i] = est->p_n[i] + est->v_n[i] * dt_s + 0.5f * acceleration * dt_s * dt_s;
    }
    if (!VectorValid(v_new, 1.0e6f) || !VectorValid(p_new, 1.0e8f))
    {
        est->healthy = false;
        return false;
    }

    /* Phi cua sai so GLOBAL; them hang dt^2 cho vi tri. */
    Identity(est->matrix);
    Skew(force_n, skew);
    for (int i = 0; i < 3; ++i)
    {
        est->matrix[POSITION + i][VELOCITY + i] = dt_s;
        for (int j = 0; j < 3; ++j)
        {
            est->matrix[THETA + i][GYRO_BIAS + j] = -r[3 * i + j] * dt_s;
            est->matrix[VELOCITY + i][THETA + j] = -skew[3 * i + j] * dt_s;
            est->matrix[VELOCITY + i][ACCEL_BIAS + j] = -r[3 * i + j] * dt_s;
            est->matrix[POSITION + i][THETA + j] = -0.5f * skew[3 * i + j] * dt_s * dt_s;
            est->matrix[POSITION + i][ACCEL_BIAS + j] = -0.5f * r[3 * i + j] * dt_s * dt_s;
        }
    }
    TransformCovariance(est, est->p, est->work_p);

    /* Qd: nhieu trang lien tuc, cung mat do tren ba truc. */
    const float qg = est->config.gyro_noise * est->config.gyro_noise;
    const float qa = est->config.accel_noise * est->config.accel_noise;
    const float qbg = est->config.gyro_bias_walk * est->config.gyro_bias_walk;
    const float qba = est->config.accel_bias_walk * est->config.accel_bias_walk;
    for (int i = 0; i < 3; ++i)
    {
        est->work_p[THETA + i][THETA + i] += qg * dt_s;
        est->work_p[VELOCITY + i][VELOCITY + i] += qa * dt_s;
        est->work_p[POSITION + i][POSITION + i] += qa * dt_s * dt_s * dt_s / 3.0f;
        est->work_p[POSITION + i][VELOCITY + i] += 0.5f * qa * dt_s * dt_s;
        est->work_p[VELOCITY + i][POSITION + i] += 0.5f * qa * dt_s * dt_s;
        est->work_p[GYRO_BIAS + i][GYRO_BIAS + i] += qbg * dt_s;
        est->work_p[ACCEL_BIAS + i][ACCEL_BIAS + i] += qba * dt_s;
    }
    if (!CovarianceValid(est->work_p))
    {
        est->healthy = false;
        return false;
    }
    est->q_nb = q_new;
    memcpy(est->v_n, v_new, sizeof(v_new));
    memcpy(est->p_n, p_new, sizeof(p_new));
    memcpy(est->p, est->work_p, sizeof(est->p));
    memcpy(est->last_gyro, gyro_rad_s, sizeof(est->last_gyro));
    est->accel_age_s = fminf(est->accel_age_s + dt_s, 1000.0f);
    est->mag_age_s = fminf(est->mag_age_s + dt_s, 1000.0f);
    est->baro_age_s = fminf(est->baro_age_s + dt_s, 1000.0f);
    return true;
}

bool Estimator_UpdateAccel(Estimator_t *est, const float accel_mps2[3], bool low_dynamics)
{
    const float gravity_force_n[3] = {0.0f, 0.0f, -GRAVITY};
    float r[9];
    float force_b[3];
    float measured_force[3];
    float residual[3];
    float h[3][N] = {{0.0f}};
    float skew[9];

    if (est == NULL || !est->initialized || !est->healthy || !VectorValid(accel_mps2, 300.0f))
    {
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        measured_force[i] = accel_mps2[i] - est->ba[i];
    }
    if (!low_dynamics || fabsf(VectorNorm(measured_force) - GRAVITY) > est->config.accel_norm_gate)
    {
        est->rejected_accel++;
        return false;
    }
    if (!Quaternion_ToRotationMatrix(&est->q_nb, r) || !Quaternion_RotateVectorInverse(&est->q_nb, gravity_force_n, force_b))
    {
        est->healthy = false;
        return false;
    }
    Skew(gravity_force_n, skew);
    for (int i = 0; i < 3; ++i)
    {
        residual[i] = accel_mps2[i] - (force_b[i] + est->ba[i]);
        h[i][ACCEL_BIAS + i] = 1.0f;
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                h[i][THETA + j] += r[3 * k + i] * skew[3 * k + j];
            }
        }
    }
    if (!Correct(est, residual, h, 3, est->config.gravity_std * est->config.gravity_std))
    {
        est->rejected_accel++;
        return false;
    }
    est->accel_age_s = 0.0f;
    return true;
}

bool Estimator_UpdateMag(Estimator_t *est, const float mag_body_ut[3])
{
    float r[9];
    float expected[3];
    float residual[3];
    float h[3][N] = {{0.0f}};
    float skew[9];

    if (est == NULL || !est->initialized || !est->healthy || !est->mag_enabled || !VectorValid(mag_body_ut, 100.0f))
    {
        return false;
    }
    if (fabsf(VectorNorm(mag_body_ut) - est->mag_reference_norm) > est->config.mag_norm_gate * est->mag_reference_norm)
    {
        est->rejected_mag++;
        return false;
    }
    if (!Quaternion_ToRotationMatrix(&est->q_nb, r) || !Quaternion_RotateVectorInverse(&est->q_nb, est->mag_reference_n, expected))
    {
        est->healthy = false;
        return false;
    }
    Skew(est->mag_reference_n, skew);
    for (int i = 0; i < 3; ++i)
    {
        residual[i] = mag_body_ut[i] - expected[i];
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                h[i][THETA + j] += r[3 * k + i] * skew[3 * k + j];
            }
        }
    }
    if (!Correct(est, residual, h, 3, est->config.mag_std_ut * est->config.mag_std_ut))
    {
        est->rejected_mag++;
        return false;
    }
    est->mag_age_s = 0.0f;
    return true;
}

bool Estimator_UpdateBaro(Estimator_t *est, float altitude_m)
{
    float residual[3] = {0.0f};
    float h[3][N] = {{0.0f}};

    if (est == NULL || !est->initialized || !est->healthy || !isfinite(altitude_m) || fabsf(altitude_m) > 20000.0f)
    {
        return false;
    }
    residual[0] = altitude_m - est->baro_reference_m + est->p_n[2];
    h[0][POSITION + 2] = -1.0f;
    if (!Correct(est, residual, h, 1, est->config.baro_std_m * est->config.baro_std_m))
    {
        est->rejected_baro++;
        return false;
    }
    est->baro_age_s = 0.0f;
    return true;
}

bool Estimator_GetOutput(const Estimator_t *est, Estimator_Output_t *out)
{
    if (est == NULL || out == NULL || !est->initialized)
    {
        return false;
    }
    out->q_nb = est->q_nb;
    memcpy(out->velocity_n_mps, est->v_n, sizeof(est->v_n));
    memcpy(out->position_n_m, est->p_n, sizeof(est->p_n));
    memcpy(out->gyro_bias_rad_s, est->bg, sizeof(est->bg));
    memcpy(out->accel_bias_mps2, est->ba, sizeof(est->ba));
    for (int i = 0; i < 3; ++i)
    {
        out->body_rate_rad_s[i] = est->last_gyro[i] - est->bg[i];
    }
    out->height_m = -est->p_n[2];
    out->climb_rate_mps = -est->v_n[2];
    out->numerical_ok = est->healthy;
    out->tilt_aided = est->healthy && est->accel_age_s < 0.5f;
    out->yaw_aided = est->healthy && est->mag_enabled && est->mag_age_s < 0.5f;
    out->height_aided = est->healthy && est->baro_age_s < 0.5f;
    out->horizontal_aided = false;
    return true;
}
