//
// Created by 厉宇桐 on 2021/9/5.
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define MATRIX_SIZE 6
#define MPU_CAL_DISP 1
// #define float_t float

typedef enum MPUCalStatusTypeDef {
    MPUCAL_SUCCEED,
    MPUCAL_READY,
    MPUCAL_FAILED
} MPUCalStatusTypeDef;

typedef struct mpu_cal_ctx_t {
    uint32_t n;
    float_t m_matrix[MATRIX_SIZE][MATRIX_SIZE + 1];
    float_t solve[MATRIX_SIZE];
    MPUCalStatusTypeDef status;
    float_t res_X0;
    float_t res_Y0;
    float_t res_Z0;
    float_t res_A;
    float_t res_B;
    float_t res_C;
} mpu_cal_ctx_t;

void reset_matrix(mpu_cal_ctx_t* mpuCalCtx);
void add_cal_data(mpu_cal_ctx_t* mpuCalCtx, float_t x, float_t y, float_t z);
uint8_t fit_ellipsoid(mpu_cal_ctx_t* mpuCalCtx);
#include "math.h"

//取绝对值
#define Abs(x) (x)<0?(-(x)):(x)

//把矩阵系数全部清除为0

void reset_matrix(mpu_cal_ctx_t* mpuCalCtx) {
    mpuCalCtx->n = 0;
    for (uint8_t row = 0; row < MATRIX_SIZE; row++) {
        mpuCalCtx->solve[row] = 0.0f;
    }

    for (uint8_t row = 0; row < MATRIX_SIZE; row++) {
        for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++)
            mpuCalCtx->m_matrix[row][column] = 0.0f;
    }
    mpuCalCtx->status = MPUCAL_READY;
}

//把输入的数据先生成矩阵的元素的总和
void add_cal_data(mpu_cal_ctx_t* mpuCalCtx, float x, float y, float z) {
    float V[MATRIX_SIZE + 1];
    mpuCalCtx->n++;
    V[0] = y * y;
    V[1] = z * z;
    V[2] = x;
    V[3] = y;
    V[4] = z;
    V[5] = 1.0;
    V[6] = -x * x;
    //构建系数矩阵，并进行累加
    for (uint8_t row = 0; row < MATRIX_SIZE; row++) {
        for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++) {
            mpuCalCtx->m_matrix[row][column] += V[row] * V[column];
        }
    }
    //b向量是m_matrix[row][6]
}

//化简系数矩阵，把除以N带上
static void cal_data_average(mpu_cal_ctx_t* mpuCalCtx) {
    for (uint8_t row = 0; row < MATRIX_SIZE; row++)
        for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++)
            mpuCalCtx->m_matrix[row][column] /= mpuCalCtx->n;
    //b向量是m_matrix[row][6]
}

//显示出来系数矩阵和增广矩阵[A|b]
static void disp_matrix(mpu_cal_ctx_t* mpuCalCtx) {
    for (uint8_t row = 0; row < MATRIX_SIZE; row++) {
        for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++) {
            printf("%23f ", mpuCalCtx->m_matrix[row][column]);
            if (column == MATRIX_SIZE - 1)
                printf("|");
        }
        printf("\r\n");
    }
    printf("\r\n\r\n");
}

//交换两行元素位置
static void swap_row1_row2(mpu_cal_ctx_t* mpuCalCtx, int row_idx1, int row_idx2) {
    float_t tmp;
    for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++) {
        tmp = mpuCalCtx->m_matrix[row_idx1][column];
        mpuCalCtx->m_matrix[row_idx1][column] = mpuCalCtx->m_matrix[row_idx2][column];
        mpuCalCtx->m_matrix[row_idx2][column] = tmp;
    }
}

//用把row行的元素乘以一个系数k
static void row_multi_k(mpu_cal_ctx_t* mpuCalCtx, float_t k, int row_idx) {
    for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++)
        mpuCalCtx->m_matrix[row_idx][column] *= k;
}

//用一个数乘以row1行加到row2行上去
static void row2_add_row1k(mpu_cal_ctx_t* mpuCalCtx, float_t k, int row_idx1, int row_idx2) {
    for (uint8_t column = 0; column < MATRIX_SIZE + 1; column++)
        mpuCalCtx->m_matrix[row_idx2][column] += k * mpuCalCtx->m_matrix[row_idx1][column];
}


//列主元，第k次消元之前，把k行到MATRIX_SIZE的所有行进行冒泡排出最大，排序的依据是k列的元素的大小
void sort_rows(mpu_cal_ctx_t* mpuCalCtx, int k) {
    int row;

    for (row = k + 1; row < MATRIX_SIZE; row++) {
        if (Abs(mpuCalCtx->m_matrix[k][k]) < Abs(mpuCalCtx->m_matrix[row][k])) {
            swap_row1_row2(mpuCalCtx, k, row);
        }
    }
}

//高斯消元法，求行阶梯型矩阵
static uint8_t gauss_elimination(mpu_cal_ctx_t* mpuCalCtx) {
    double k;
    for (uint8_t cnt = 0; cnt < MATRIX_SIZE; cnt++)//进行第k次的运算，主要是针对k行以下的行数把k列的元素都变成0
    {
        //把k行依据k列的元素大小，进行排序
        sort_rows(mpuCalCtx, cnt);
        if (mpuCalCtx->m_matrix[cnt][cnt] == 0)
            return (1);//返回值表示错误
        //把k行下面的行元素全部消成0，整行变化
        for (uint8_t row = cnt + 1; row < MATRIX_SIZE; row++) {
            k = -mpuCalCtx->m_matrix[row][cnt] / mpuCalCtx->m_matrix[cnt][cnt];
            row2_add_row1k(mpuCalCtx, k, cnt, row);
        }
#if MPU_CAL_DISP
        disp_matrix(mpuCalCtx);
#endif
    }
    return 0;
}

//求行最简型矩阵，即把对角线的元素全部化成1
static void simplify_row(mpu_cal_ctx_t* mpuCalCtx) {
    float_t k;
    for (uint8_t row = 0; row < MATRIX_SIZE; row++) {
        k = 1 / mpuCalCtx->m_matrix[row][row];
        row_multi_k(mpuCalCtx, k, row);
    }
#if MPU_CAL_DISP
    disp_matrix(mpuCalCtx);
#endif
}

//求非齐次线性方程组的解
static void solve_matrix(mpu_cal_ctx_t* mpuCalCtx) {
    for (short row = MATRIX_SIZE - 1; row >= 0; row--) {
        mpuCalCtx->solve[row] = mpuCalCtx->m_matrix[row][MATRIX_SIZE];
        for (uint8_t column = MATRIX_SIZE - 1; column >= row + 1; column--)
            mpuCalCtx->solve[row] -= mpuCalCtx->m_matrix[row][column] * mpuCalCtx->solve[column];
    }
#if MPU_CAL_DISP
    printf("  a = %f| b = %f| c = %f| d = %f| e = %f| f = %f ", mpuCalCtx->solve[0], mpuCalCtx->solve[1], mpuCalCtx->solve[2], mpuCalCtx->solve[3], mpuCalCtx->solve[4], mpuCalCtx->solve[5]);
    printf("\r\n");
    printf("\r\n");
#endif

}


uint8_t fit_ellipsoid(mpu_cal_ctx_t* mpuCalCtx) {
    cal_data_average(mpuCalCtx);//对输入的数据到矩阵元素进行归一化
#if MPU_CAL_DISP
    disp_matrix(mpuCalCtx);//显示原始的增广矩阵
#endif

    if (gauss_elimination(mpuCalCtx)) {//求得行阶梯形矩阵
#if MPU_CAL_DISP
        printf("the marix could not be solved\r\n");
#endif
        mpuCalCtx->status = MPUCAL_FAILED;
        return -1;
    } else {
        simplify_row(mpuCalCtx);//化行最简形态
        solve_matrix(mpuCalCtx);//求解a,b,c,d,e,f

        float_t a, b, c, d, e, f;
        a = mpuCalCtx->solve[0];
        b = mpuCalCtx->solve[1];
        c = mpuCalCtx->solve[2];
        d = mpuCalCtx->solve[3];
        e = mpuCalCtx->solve[4];
        f = mpuCalCtx->solve[5];

        float_t X0, Y0, Z0, A, B, C;

        X0 = -c / 2;
        Y0 = -d / (2 * a);
        Z0 = -e / (2 * b);
        A = sqrt((double)(X0 * X0 + a * Y0 * Y0 + b * Z0 * Z0 - f));
        B = A / sqrt((double)a);
        C = A / sqrt((double)b);

        mpuCalCtx->res_X0 = X0;
        mpuCalCtx->res_Y0 = Y0;
        mpuCalCtx->res_Z0 = Z0;
        mpuCalCtx->res_A = A;
        mpuCalCtx->res_B = B;
        mpuCalCtx->res_C = C;

#if MPU_CAL_DISP
        printf("  ((x - x0) / A) ^ 2 + ((y - y0) / B) ^ 2 + ((z - z0) / C) ^ 2 = 1 Ellipsoid result as below：\r\n");
        printf("\r\n");
        printf("  X0 = %f| Y0 = %f| Z0 = %f| A = %f| B = %f| C = %f \r\n", X0, Y0, Z0, A, B, C);
#endif
        mpuCalCtx->status = MPUCAL_SUCCEED;
        return 0;
    }
}

//整个椭球校准的过程
void demo(void) {
    mpu_cal_ctx_t ctx;
    reset_matrix(&ctx);

    //这里输入任意个点加速度参数，尽量在球面上均匀分布
    // add_cal_data(&ctx, 87, -52, -4454);
    // add_cal_data(&ctx, 301, -45, 3859);
    // add_cal_data(&ctx, 274, 4088, -303);
    // add_cal_data(&ctx, 312, -4109, -305);
    // add_cal_data(&ctx, -3805, -24, -390);
    // add_cal_data(&ctx, 4389, 6, -228);
    // add_cal_data(&ctx, 261, 2106, -3848);
    // add_cal_data(&ctx, 327, -2047, -3880);
    // add_cal_data(&ctx, -1963, -13, -3797);
    // add_cal_data(&ctx, 3024, 18, -3449);

//    add_cal_data(&ctx, -0.00128174, -0.0018310, -0.99981689);
//    add_cal_data(&ctx, -0.24090576, -1.18066406, -0.77886963);
//    add_cal_data(&ctx, 0.66064453, -0.39746094, -1.47741699);
//    add_cal_data(&ctx, 0.36529541, -0.04718018, -0.75091553);
//    add_cal_data(&ctx, 0.83874512, -0.63452148, -0.61523438);
//    add_cal_data(&ctx, 0.27532959, -0.13775635, -0.68762207);
//    add_cal_data(&ctx, -0.07562256, 1.22528076, -1.26226807);
//    add_cal_data(&ctx, -0.13549805, 0.17059326, -0.50830078);

    add_cal_data(&ctx, 0, 0, 2);
    add_cal_data(&ctx, 0, 1, 0);
    add_cal_data(&ctx, 1, 0, 0);
    add_cal_data(&ctx, -1, 0, 0);
    add_cal_data(&ctx, 0, -1, 0);
    add_cal_data(&ctx, 0, 0, -2);

    fit_ellipsoid(&ctx);
}

int main() {
    demo();
}