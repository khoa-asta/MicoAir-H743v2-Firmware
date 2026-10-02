/**
 *  @brief Phep quay Hamilton cho ESKF va bo dieu khien.
 */
#include "quaternion.h"

#include <math.h>
#include <stddef.h>

#define QUATERNION_MIN_NORM          (1.0e-6f)
#define QUATERNION_SMALL_ANGLE_RAD   (1.0e-3f)
#define QUATERNION_MATRIX_TOL        (1.0e-3f)

/** Kiem tra bon thanh phan huu han. */
static bool Quaternion_IsFinite(const Quaternion_t *q)
{
    return (q != NULL) && isfinite(q->w) && isfinite(q->x) && isfinite(q->y) && isfinite(q->z);
}

/* Kiem tra vector co ba thanh phan huu han. */
static bool Quaternion_VectorIsFinite(const float v[3])
{
    return (v != NULL) && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* Chia theo phan tu lon nhat de tinh norm on dinh. */
static bool Quaternion_GetScaled(const Quaternion_t *q, Quaternion_t *scaled, float *scale, float *scaled_norm)
{
    float max_abs;
    Quaternion_t temp;
    float norm;

    if (!Quaternion_IsFinite(q))
    {
        return false;
    }

    max_abs = fmaxf(fmaxf(fabsf(q->w), fabsf(q->x)), fmaxf(fabsf(q->y), fabsf(q->z)));
    if (max_abs == 0.0f)
    {
        return false;
    }

    temp.w = q->w / max_abs;
    temp.x = q->x / max_abs;
    temp.y = q->y / max_abs;
    temp.z = q->z / max_abs;
    norm = sqrtf(temp.w * temp.w + temp.x * temp.x + temp.y * temp.y + temp.z * temp.z);

    if (max_abs < QUATERNION_MIN_NORM / norm)
    {
        return false;
    }

    *scaled = temp;
    *scale = max_abs;
    *scaled_norm = norm;
    return true;
}

/* Kiem tra truc chuan va dinh thuc gan +1. */
static bool Quaternion_MatrixIsRotation(const float r[9])
{
    size_t i;
    size_t j;
    size_t k;
    float determinant;

    if (r == NULL)
    {
        return false;
    }

    for (i = 0U; i < 9U; ++i)
    {
        if (!isfinite(r[i]) || fabsf(r[i]) > 1.0f + QUATERNION_MATRIX_TOL)
        {
            return false;
        }
    }

    for (i = 0U; i < 3U; ++i)
    {
        for (j = i; j < 3U; ++j)
        {
            float dot = 0.0f;
            const float expected = (i == j) ? 1.0f : 0.0f;

            for (k = 0U; k < 3U; ++k)
            {
                dot += r[3U * i + k] * r[3U * j + k];
            }
            if (fabsf(dot - expected) > QUATERNION_MATRIX_TOL)
            {
                return false;
            }
        }
    }

    determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
    return fabsf(determinant - 1.0f) <= QUATERNION_MATRIX_TOL;
}

/* Tao quaternion [1, 0, 0, 0]. */
bool Quaternion_Identity(Quaternion_t *out)
{
    const Quaternion_t identity = {1.0f, 0.0f, 0.0f, 0.0f};

    if (out == NULL)
    {
        return false;
    }
    *out = identity;
    return true;
}

/* Tinh do dai quaternion. */
float Quaternion_Norm(const Quaternion_t *q)
{
    if (!Quaternion_IsFinite(q))
    {
        return NAN;
    }
    return hypotf(hypotf(q->w, q->x), hypotf(q->y, q->z));
}

/* Dua quaternion ve do dai bang 1. */
bool Quaternion_Normalize(const Quaternion_t *q, Quaternion_t *out)
{
    Quaternion_t temp;
    float scale;
    float norm;
    float inverse_norm;

    if (out == NULL || !Quaternion_GetScaled(q, &temp, &scale, &norm))
    {
        return false;
    }

    inverse_norm = 1.0f / norm;
    temp.w *= inverse_norm;
    temp.x *= inverse_norm;
    temp.y *= inverse_norm;
    temp.z *= inverse_norm;
    *out = temp;
    return true;
}

/* Doi dau ba thanh phan vector. */
bool Quaternion_Conjugate(const Quaternion_t *q, Quaternion_t *out)
{
    Quaternion_t temp;

    if (out == NULL || !Quaternion_IsFinite(q))
    {
        return false;
    }

    temp.w = q->w;
    temp.x = -q->x;
    temp.y = -q->y;
    temp.z = -q->z;
    *out = temp;
    return true;
}

/* Tinh quaternion nghich dao. */
bool Quaternion_Inverse(const Quaternion_t *q, Quaternion_t *out)
{
    Quaternion_t temp;
    float scale;
    float norm;
    float factor;

    if (out == NULL || !Quaternion_GetScaled(q, &temp, &scale, &norm))
    {
        return false;
    }

    factor = (1.0f / scale) / (norm * norm);
    temp.w *= factor;
    temp.x *= -factor;
    temp.y *= -factor;
    temp.z *= -factor;
    if (!Quaternion_IsFinite(&temp))
    {
        return false;
    }
    *out = temp;
    return true;
}

/* Tinh a nhan b theo quy uoc Hamilton. */
bool Quaternion_Multiply(const Quaternion_t *a, const Quaternion_t *b, Quaternion_t *out)
{
    Quaternion_t temp;

    if (out == NULL || !Quaternion_IsFinite(a) || !Quaternion_IsFinite(b))
    {
        return false;
    }

    temp.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    temp.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    temp.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    temp.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;

    if (!Quaternion_IsFinite(&temp))
    {
        return false;
    }
    *out = temp;
    return true;
}

/* Doi vector quay radian sang quaternion. */
bool Quaternion_FromRotationVector(const float rotation_rad[3], Quaternion_t *out)
{
    Quaternion_t temp;
    float theta;

    if (out == NULL || !Quaternion_VectorIsFinite(rotation_rad))
    {
        return false;
    }

    theta = hypotf(hypotf(rotation_rad[0], rotation_rad[1]), rotation_rad[2]);
    if (!isfinite(theta))
    {
        return false;
    }

    if (theta < QUATERNION_SMALL_ANGLE_RAD)
    {
        /* Khai trien gan 0 de tranh chia cho goc rat nho. */
        const float theta2 = theta * theta;
        const float theta4 = theta2 * theta2;
        const float factor = 0.5f - theta2 / 48.0f + theta4 / 3840.0f;

        temp.w = 1.0f - theta2 / 8.0f + theta4 / 384.0f;
        temp.x = factor * rotation_rad[0];
        temp.y = factor * rotation_rad[1];
        temp.z = factor * rotation_rad[2];
    }
    else
    {
        const float sin_half = sinf(0.5f * theta);

        temp.w = cosf(0.5f * theta);
        temp.x = (rotation_rad[0] / theta) * sin_half;
        temp.y = (rotation_rad[1] / theta) * sin_half;
        temp.z = (rotation_rad[2] / theta) * sin_half;
    }

    return Quaternion_Normalize(&temp, out);
}

/* Lay vector quay ngan nhat, goc tu 0 den pi. */
bool Quaternion_ToRotationVector(const Quaternion_t *q, float rotation_rad[3])
{
    Quaternion_t unit;
    float vector_norm;
    float factor;

    if (rotation_rad == NULL || !Quaternion_Normalize(q, &unit))
    {
        return false;
    }

    /* Chon dai dien co w khong am. */
    if (unit.w < 0.0f)
    {
        unit.w = -unit.w;
        unit.x = -unit.x;
        unit.y = -unit.y;
        unit.z = -unit.z;
    }

    vector_norm = hypotf(hypotf(unit.x, unit.y), unit.z);
    if (vector_norm < QUATERNION_SMALL_ANGLE_RAD)
    {
        const float s2 = vector_norm * vector_norm;

        factor = 2.0f + s2 / 3.0f + 3.0f * s2 * s2 / 20.0f;
    }
    else
    {
        factor = 2.0f * atan2f(vector_norm, unit.w) / vector_norm;
    }

    rotation_rad[0] = factor * unit.x;
    rotation_rad[1] = factor * unit.y;
    rotation_rad[2] = factor * unit.z;
    return true;
}

/* Tao ma tran quay body sang navigation. */
bool Quaternion_ToRotationMatrix(const Quaternion_t *q_nb, float r_nb[9])
{
    Quaternion_t q;
    float xx;
    float yy;
    float zz;
    float xy;
    float xz;
    float yz;
    float wx;
    float wy;
    float wz;

    if (r_nb == NULL || !Quaternion_Normalize(q_nb, &q))
    {
        return false;
    }

    xx = q.x * q.x;
    yy = q.y * q.y;
    zz = q.z * q.z;
    xy = q.x * q.y;
    xz = q.x * q.z;
    yz = q.y * q.z;
    wx = q.w * q.x;
    wy = q.w * q.y;
    wz = q.w * q.z;

    r_nb[0] = 1.0f - 2.0f * (yy + zz);
    r_nb[1] = 2.0f * (xy - wz);
    r_nb[2] = 2.0f * (xz + wy);
    r_nb[3] = 2.0f * (xy + wz);
    r_nb[4] = 1.0f - 2.0f * (xx + zz);
    r_nb[5] = 2.0f * (yz - wx);
    r_nb[6] = 2.0f * (xz - wy);
    r_nb[7] = 2.0f * (yz + wx);
    r_nb[8] = 1.0f - 2.0f * (xx + yy);
    return true;
}

/* Tao quaternion tu ma tran quay hop le. */
bool Quaternion_FromRotationMatrix(const float r_nb[9], Quaternion_t *out)
{
    Quaternion_t temp;
    Quaternion_t unit;
    float trace;
    float s;

    if (out == NULL || !Quaternion_MatrixIsRotation(r_nb))
    {
        return false;
    }

    /* Chon nhanh on dinh ca khi goc quay gan 180 do. */
    trace = r_nb[0] + r_nb[4] + r_nb[8];

    if (trace > 0.0f)
    {
        s = 2.0f * sqrtf(1.0f + trace);
        temp.w = 0.25f * s;
        temp.x = (r_nb[7] - r_nb[5]) / s;
        temp.y = (r_nb[2] - r_nb[6]) / s;
        temp.z = (r_nb[3] - r_nb[1]) / s;
    }
    else if (r_nb[0] >= r_nb[4] && r_nb[0] >= r_nb[8])
    {
        s = 2.0f * sqrtf(1.0f + r_nb[0] - r_nb[4] - r_nb[8]);
        temp.w = (r_nb[7] - r_nb[5]) / s;
        temp.x = 0.25f * s;
        temp.y = (r_nb[1] + r_nb[3]) / s;
        temp.z = (r_nb[2] + r_nb[6]) / s;
    }
    else if (r_nb[4] >= r_nb[8])
    {
        s = 2.0f * sqrtf(1.0f + r_nb[4] - r_nb[0] - r_nb[8]);
        temp.w = (r_nb[2] - r_nb[6]) / s;
        temp.x = (r_nb[1] + r_nb[3]) / s;
        temp.y = 0.25f * s;
        temp.z = (r_nb[5] + r_nb[7]) / s;
    }
    else
    {
        s = 2.0f * sqrtf(1.0f + r_nb[8] - r_nb[0] - r_nb[4]);
        temp.w = (r_nb[3] - r_nb[1]) / s;
        temp.x = (r_nb[2] + r_nb[6]) / s;
        temp.y = (r_nb[5] + r_nb[7]) / s;
        temp.z = 0.25f * s;
    }

    if (!Quaternion_Normalize(&temp, &unit))
    {
        return false;
    }
    /* Chon dai dien co w khong am. */
    if (unit.w < 0.0f)
    {
        unit.w = -unit.w;
        unit.x = -unit.x;
        unit.y = -unit.y;
        unit.z = -unit.z;
    }
    *out = unit;
    return true;
}

/* Doi vector body sang navigation. */
bool Quaternion_RotateVector(const Quaternion_t *q_nb, const float v_b[3], float v_n[3])
{
    float r[9];
    float temp[3];
    size_t row;

    if (v_n == NULL || !Quaternion_VectorIsFinite(v_b) || !Quaternion_ToRotationMatrix(q_nb, r))
    {
        return false;
    }

    for (row = 0U; row < 3U; ++row)
    {
        temp[row] = r[3U * row] * v_b[0] + r[3U * row + 1U] * v_b[1] + r[3U * row + 2U] * v_b[2];
    }
    if (!Quaternion_VectorIsFinite(temp))
    {
        return false;
    }
    for (row = 0U; row < 3U; ++row)
    {
        v_n[row] = temp[row];
    }
    return true;
}

/* Doi vector navigation sang body. */
bool Quaternion_RotateVectorInverse(const Quaternion_t *q_nb, const float v_n[3], float v_b[3])
{
    float r[9];
    float temp[3];
    size_t row;

    if (v_b == NULL || !Quaternion_VectorIsFinite(v_n) || !Quaternion_ToRotationMatrix(q_nb, r))
    {
        return false;
    }

    for (row = 0U; row < 3U; ++row)
    {
        temp[row] = r[row] * v_n[0] + r[3U + row] * v_n[1] + r[6U + row] * v_n[2];
    }
    if (!Quaternion_VectorIsFinite(temp))
    {
        return false;
    }
    for (row = 0U; row < 3U; ++row)
    {
        v_b[row] = temp[row];
    }
    return true;
}

/* Tao quaternion tu roll, pitch, yaw theo ZYX. */
bool Quaternion_FromEulerZYX(float roll_rad, float pitch_rad, float yaw_rad, Quaternion_t *out)
{
    Quaternion_t temp;
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;

    if (out == NULL || !isfinite(roll_rad) || !isfinite(pitch_rad) || !isfinite(yaw_rad))
    {
        return false;
    }

    cr = cosf(0.5f * roll_rad);
    sr = sinf(0.5f * roll_rad);
    cp = cosf(0.5f * pitch_rad);
    sp = sinf(0.5f * pitch_rad);
    cy = cosf(0.5f * yaw_rad);
    sy = sinf(0.5f * yaw_rad);

    temp.w = cr * cp * cy + sr * sp * sy;
    temp.x = sr * cp * cy - cr * sp * sy;
    temp.y = cr * sp * cy + sr * cp * sy;
    temp.z = cr * cp * sy - sr * sp * cy;
    return Quaternion_Normalize(&temp, out);
}

/* Tich phan gyro body bang cach nhan delta_q ben phai. */
bool Quaternion_IntegrateBodyRate(const Quaternion_t *q_nb, const float omega_body_rad_s[3], float dt_s, Quaternion_t *out)
{
    Quaternion_t unit;
    Quaternion_t delta_q;
    Quaternion_t next;
    float rotation_rad[3];
    size_t i;

    if (out == NULL || !isfinite(dt_s) || dt_s < 0.0f || !Quaternion_VectorIsFinite(omega_body_rad_s) || !Quaternion_Normalize(q_nb, &unit))
    {
        return false;
    }

    for (i = 0U; i < 3U; ++i)
    {
        rotation_rad[i] = omega_body_rad_s[i] * dt_s;
    }
    if (!Quaternion_FromRotationVector(rotation_rad, &delta_q) || !Quaternion_Multiply(&unit, &delta_q, &next))
    {
        return false;
    }
    return Quaternion_Normalize(&next, out);
}

/* Sua tu the bang cach nhan delta_q ben trai. */
bool Quaternion_ApplyGlobalError(const Quaternion_t *q_hat, const float delta_theta_n_rad[3], Quaternion_t *out)
{
    Quaternion_t unit;
    Quaternion_t delta_q;
    Quaternion_t corrected;

    if (out == NULL || !Quaternion_Normalize(q_hat, &unit) || !Quaternion_FromRotationVector(delta_theta_n_rad, &delta_q) || !Quaternion_Multiply(&delta_q, &unit, &corrected))
    {
        return false;
    }
    return Quaternion_Normalize(&corrected, out);
}
