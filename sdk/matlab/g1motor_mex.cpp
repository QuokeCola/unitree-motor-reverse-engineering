// MEX gateway to the g1motor C++ library. Used through G1MotorBus.m; build with build_mex.m.
//   g1motor_mex('open', port)            port '' = the only FTDI adapter (Linux)
//   g1motor_mex('close')
//   s = g1motor_mex('scan')              struct array, one element per motor found
//   s = g1motor_mex('enable', ids)
//   g1motor_mex('disable', ids)          ids [] = every enabled motor
//   s = g1motor_mex('restart', ids)
//   s = g1motor_mex('command', id, pos, vel, tau, kp, kd)   empty struct if no reply
//   s = g1motor_mex('read', id)
//   g1motor_mex('set_joint', id, gear_ratio, low, high)
//   j = g1motor_mex('get_joint', id)     struct: gear_ratio, low, high, zero, turn_counts
//   b = g1motor_mex('listen', seconds)   uint8 row of raw bytes, nothing transmitted
#include <cmath>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "g1motor/g1motor.hpp"
#include "mex.h"

namespace {

std::unique_ptr<g1motor::MotorBus> bus;

const char* kFields[] = {"id", "mode", "pos", "vel", "tau", "temp_coil", "temp_shell", "voltage", "error", "fault"};
const int kNumFields = 10;

void fill(mxArray* a, mwIndex i, const g1motor::MotorState& s) {
    const double v[] = {double(s.id), double(s.mode), double(s.pos), double(s.vel), double(s.tau), double(s.temp_coil),
                        double(s.temp_shell), s.voltage(), double(s.error)};
    for (int k = 0; k < 9; ++k) mxSetFieldByNumber(a, i, k, mxCreateDoubleScalar(v[k]));
    mxSetFieldByNumber(a, i, 9, mxCreateLogicalScalar(s.fault()));
}

mxArray* states(const std::map<int, g1motor::MotorState>& m) {
    mxArray* a = mxCreateStructMatrix(1, m.size(), kNumFields, kFields);
    mwIndex i = 0;
    for (const auto& kv : m) fill(a, i++, kv.second);
    return a;
}

mxArray* state(const std::optional<g1motor::MotorState>& s) {
    mxArray* a = mxCreateStructMatrix(1, s ? 1 : 0, kNumFields, kFields);
    if (s) fill(a, 0, *s);
    return a;
}

std::vector<int> id_list(const mxArray* a) {
    std::vector<int> ids;
    if (!mxIsNumeric(a)) throw std::runtime_error("ids must be numeric");
    const mwSize n = mxGetNumberOfElements(a);
    for (mwSize i = 0; i < n; ++i) ids.push_back(static_cast<int>(std::llround(mxGetPr(a)[i])));
    return ids;
}

double num(const mxArray* a) {
    if (!mxIsNumeric(a) || mxGetNumberOfElements(a) != 1) throw std::runtime_error("expected a numeric scalar");
    return mxGetScalar(a);
}

void need(int nrhs, int n) {
    if (nrhs < n) throw std::runtime_error("not enough arguments");
}

g1motor::MotorBus& b() {
    if (!bus) throw std::runtime_error("bus is not open");
    return *bus;
}

void at_exit() { bus.reset(); }

}  // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    (void)nlhs;
    std::string error;
    try {
        if (nrhs < 1 || !mxIsChar(prhs[0])) throw std::runtime_error("first argument must be a command name");
        char* raw = mxArrayToString(prhs[0]);
        const std::string cmd(raw);
        mxFree(raw);

        if (cmd == "open") {
            std::string port;
            if (nrhs > 1 && mxIsChar(prhs[1])) { char* p = mxArrayToString(prhs[1]); port = p; mxFree(p); }
            bus.reset();
            bus = std::make_unique<g1motor::MotorBus>(port);
            mexAtExit(at_exit);
        } else if (cmd == "close") {
            bus.reset();
        } else if (cmd == "scan") {
            plhs[0] = states(b().scan());
        } else if (cmd == "enable") {
            need(nrhs, 2);
            plhs[0] = states(b().enable(id_list(prhs[1])));
        } else if (cmd == "disable") {
            b().disable(nrhs > 1 ? id_list(prhs[1]) : std::vector<int>{});
        } else if (cmd == "restart") {
            need(nrhs, 2);
            plhs[0] = states(b().restart(id_list(prhs[1])));
        } else if (cmd == "command") {
            need(nrhs, 7);
            plhs[0] = state(b().command(static_cast<int>(num(prhs[1])), static_cast<std::int32_t>(std::llround(num(prhs[2]))),
                                        static_cast<int>(std::llround(num(prhs[3]))), static_cast<int>(std::llround(num(prhs[4]))),
                                        static_cast<int>(std::llround(num(prhs[5]))), static_cast<int>(std::llround(num(prhs[6])))));
        } else if (cmd == "read") {
            need(nrhs, 2);
            plhs[0] = state(b().read(static_cast<int>(num(prhs[1]))));
        } else if (cmd == "set_joint") {
            need(nrhs, 5);
            b().set_joint(static_cast<int>(num(prhs[1])),
                          g1motor::Joint(num(prhs[2]), std::llround(num(prhs[3])), std::llround(num(prhs[4]))));
        } else if (cmd == "get_joint") {
            need(nrhs, 2);
            const g1motor::Joint& j = b().joint(static_cast<int>(num(prhs[1])));
            const char* f[] = {"gear_ratio", "low", "high", "zero", "turn_counts"};
            plhs[0] = mxCreateStructMatrix(1, 1, 5, f);
            mxSetFieldByNumber(plhs[0], 0, 0, mxCreateDoubleScalar(j.gear_ratio));
            mxSetFieldByNumber(plhs[0], 0, 1, mxCreateDoubleScalar(j.low ? double(*j.low) : NAN));
            mxSetFieldByNumber(plhs[0], 0, 2, mxCreateDoubleScalar(j.high ? double(*j.high) : NAN));
            mxSetFieldByNumber(plhs[0], 0, 3, mxCreateDoubleScalar(double(j.zero)));
            mxSetFieldByNumber(plhs[0], 0, 4, mxCreateDoubleScalar(double(j.turn_counts())));
        } else if (cmd == "listen") {
            need(nrhs, 2);
            const auto bytes = b().listen(num(prhs[1]));
            plhs[0] = mxCreateNumericMatrix(1, bytes.size(), mxUINT8_CLASS, mxREAL);
            if (!bytes.empty()) std::memcpy(mxGetData(plhs[0]), bytes.data(), bytes.size());
        } else {
            throw std::runtime_error("unknown command '" + cmd + "'");
        }
    } catch (const std::exception& e) {
        error = e.what();          // leave the try block first so C++ objects are destroyed before MATLAB jumps out
    }
    if (!error.empty()) mexErrMsgIdAndTxt("g1motor:error", "%s", error.c_str());
}
