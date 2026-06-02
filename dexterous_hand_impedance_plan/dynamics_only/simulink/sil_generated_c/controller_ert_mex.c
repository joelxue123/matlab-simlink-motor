#include "mex.h"
#include <stdint.h>

typedef struct {
    int16_t integral_e;
    int16_t dob_hat;
} DW_Controller_T;

typedef struct {
    int16_t q_ref;
    int16_t q;
    int16_t qdot;
    int16_t qddot;
    int16_t tau_prev;
    uint8_t mode;
} ExtU_Controller_T;

typedef struct {
    int16_t tau_cmd;
    int16_t tau_load_hat;
} ExtY_Controller_T;

extern DW_Controller_T Controller_DW;
extern ExtU_Controller_T Controller_U;
extern ExtY_Controller_T Controller_Y;
extern void Controller_initialize(void);
extern void Controller_step(void);
extern void Controller_terminate(void);

static int16_t clamp_to_int16(double x)
{
    if (x > 32767.0) {
        return 32767;
    }
    if (x < -32768.0) {
        return -32768;
    }
    return (int16_t)x;
}

static uint8_t clamp_to_uint8(double x)
{
    if (x > 255.0) {
        return 255U;
    }
    if (x < 0.0) {
        return 0U;
    }
    return (uint8_t)x;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const mxArray *input;
    const double *u;
    double *y;
    mwSize n;
    mwSize cols;
    mwSize k;

    if (nrhs != 1) {
        mexErrMsgIdAndTxt("controller_ert_mex:nrhs",
                          "Expected one input: an N-by-6 raw input matrix.");
    }
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("controller_ert_mex:nlhs",
                          "Expected at most one output.");
    }

    input = prhs[0];
    if (!mxIsDouble(input) || mxIsComplex(input)) {
        mexErrMsgIdAndTxt("controller_ert_mex:type",
                          "Input must be a real double matrix.");
    }

    n = mxGetM(input);
    cols = mxGetN(input);
    if (cols != 6) {
        mexErrMsgIdAndTxt("controller_ert_mex:shape",
                          "Input must be N-by-6: q_ref, q, qdot, qddot, tau_prev, mode.");
    }

    u = mxGetDoubles(input);
    plhs[0] = mxCreateDoubleMatrix(n, 2, mxREAL);
    y = mxGetDoubles(plhs[0]);

    Controller_initialize();
    Controller_DW.integral_e = 0;
    Controller_DW.dob_hat = 0;

    for (k = 0; k < n; ++k) {
        Controller_U.q_ref = clamp_to_int16(u[k + 0*n]);
        Controller_U.q = clamp_to_int16(u[k + 1*n]);
        Controller_U.qdot = clamp_to_int16(u[k + 2*n]);
        Controller_U.qddot = clamp_to_int16(u[k + 3*n]);
        Controller_U.tau_prev = clamp_to_int16(u[k + 4*n]);
        Controller_U.mode = clamp_to_uint8(u[k + 5*n]);

        Controller_step();

        y[k + 0*n] = (double)Controller_Y.tau_cmd;
        y[k + 1*n] = (double)Controller_Y.tau_load_hat;
    }

    Controller_terminate();
}
