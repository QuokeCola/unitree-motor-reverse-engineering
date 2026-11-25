/// @file g1motor_mex.cc
/// @brief MEX gateway from MATLAB to the g1motor C++ library.
///
/// Used through G1MotorBus.m; build with build_mex.m. Commands:
/// @code
///   g1motor_mex('open', port)       port '' = the only FTDI adapter (Linux)
///   g1motor_mex('close')
///   s = g1motor_mex('scan')         struct array, one element per motor
///   s = g1motor_mex('enable', ids)
///   g1motor_mex('disable', ids)     ids [] = every enabled motor
///   s = g1motor_mex('restart', ids)
///   s = g1motor_mex('command', id, pos, vel, tau, kp, kd)
///   s = g1motor_mex('read', id)
///   g1motor_mex('set_joint', id, gear_ratio, low, high)
///   j = g1motor_mex('get_joint', id)
///   b = g1motor_mex('listen', seconds)        uint8 row, nothing sent
///   f = g1motor_mex('encode', cmd_struct)     uint8 row, no bus needed
///   s = g1motor_mex('decode', frame_bytes)    state struct, no bus needed
/// @endcode

#include <cmath>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "g1motor/g1motor.h"
#include "mex.h"

namespace {

/// The one bus of this MATLAB session.
std::unique_ptr<g1motor::MotorBus> g_bus;

/// Field names of the state struct returned to MATLAB.
const char* kStateFields[] = {"id",    "mode",      "pos",        "vel",
                              "tau",   "temp_coil", "temp_shell", "voltage",
                              "error", "fault"};
/// Number of entries in kStateFields.
constexpr int kNumStateFields = 10;

/// Writes one motor state into element `index` of a struct array.
void FillState(mxArray* array, mwIndex index, const g1motor::MotorState& s) {
  const double values[] = {
      static_cast<double>(s.id),         static_cast<double>(s.mode),
      static_cast<double>(s.pos),        static_cast<double>(s.vel),
      static_cast<double>(s.tau),        static_cast<double>(s.temp_coil),
      static_cast<double>(s.temp_shell), s.voltage(),
      static_cast<double>(s.error)};
  for (int field = 0; field < 9; ++field) {
    mxSetFieldByNumber(array, index, field,
                       mxCreateDoubleScalar(values[field]));
  }
  mxSetFieldByNumber(array, index, 9, mxCreateLogicalScalar(s.fault()));
}

/// Converts several states to a 1-by-N struct array.
mxArray* StatesToStruct(const std::map<int, g1motor::MotorState>& states) {
  mxArray* array =
      mxCreateStructMatrix(1, states.size(), kNumStateFields, kStateFields);
  mwIndex index = 0;
  for (const auto& entry : states) FillState(array, index++, entry.second);
  return array;
}

/// Converts an optional state to a 1-by-1 or empty struct.
mxArray* StateToStruct(const std::optional<g1motor::MotorState>& state) {
  mxArray* array =
      mxCreateStructMatrix(1, state ? 1 : 0, kNumStateFields, kStateFields);
  if (state) FillState(array, 0, *state);
  return array;
}

/// Reads a numeric array as a list of motor ids.
std::vector<int> ToIds(const mxArray* array) {
  if (!mxIsNumeric(array)) throw std::runtime_error("ids must be numeric");
  std::vector<int> ids;
  const mwSize count = mxGetNumberOfElements(array);
  for (mwSize i = 0; i < count; ++i) {
    ids.push_back(static_cast<int>(std::llround(mxGetPr(array)[i])));
  }
  return ids;
}

/// Reads a numeric scalar.
double ToScalar(const mxArray* array) {
  if (!mxIsNumeric(array) || mxGetNumberOfElements(array) != 1) {
    throw std::runtime_error("expected a numeric scalar");
  }
  return mxGetScalar(array);
}

/// Reads a numeric scalar as an integer.
int ToInt(const mxArray* array) {
  return static_cast<int>(std::llround(ToScalar(array)));
}

/// Reads a field of a struct as an integer; 0 if the field is missing.
int FieldOr0(const mxArray* s, const char* name) {
  const mxArray* field = mxGetField(s, 0, name);
  return field ? ToInt(field) : 0;
}

/// Throws if fewer than `needed` arguments were given.
void RequireArgs(int nrhs, int needed) {
  if (nrhs < needed) throw std::runtime_error("not enough arguments");
}

/// @return The open bus; throws if there is none.
g1motor::MotorBus& Bus() {
  if (!g_bus) throw std::runtime_error("bus is not open");
  return *g_bus;
}

/// Closes the bus when MATLAB clears the MEX file or exits.
void CloseAtExit() { g_bus.reset(); }

/// Runs one command; throws on any error.
void Dispatch(const std::string& command, mxArray* plhs[], int nrhs,
              const mxArray* prhs[]) {
  if (command == "open") {
    std::string port;
    if (nrhs > 1 && mxIsChar(prhs[1])) {
      char* text = mxArrayToString(prhs[1]);
      port = text;
      mxFree(text);
    }
    g_bus.reset();
    g_bus = std::make_unique<g1motor::MotorBus>(port);
    mexAtExit(CloseAtExit);
  } else if (command == "close") {
    g_bus.reset();
  } else if (command == "scan") {
    plhs[0] = StatesToStruct(Bus().Scan());
  } else if (command == "enable") {
    RequireArgs(nrhs, 2);
    plhs[0] = StatesToStruct(Bus().Enable(ToIds(prhs[1])));
  } else if (command == "disable") {
    Bus().Disable(nrhs > 1 ? ToIds(prhs[1]) : std::vector<int>{});
  } else if (command == "restart") {
    RequireArgs(nrhs, 2);
    plhs[0] = StatesToStruct(Bus().Restart(ToIds(prhs[1])));
  } else if (command == "command") {
    RequireArgs(nrhs, 7);
    plhs[0] = StateToStruct(Bus().Command(
        ToInt(prhs[1]),
        static_cast<std::int32_t>(std::llround(ToScalar(prhs[2]))),
        ToInt(prhs[3]), ToInt(prhs[4]), ToInt(prhs[5]), ToInt(prhs[6])));
  } else if (command == "read") {
    RequireArgs(nrhs, 2);
    plhs[0] = StateToStruct(Bus().Read(ToInt(prhs[1])));
  } else if (command == "set_joint") {
    RequireArgs(nrhs, 5);
    Bus().set_joint(
        ToInt(prhs[1]),
        g1motor::Joint(ToScalar(prhs[2]), std::llround(ToScalar(prhs[3])),
                       std::llround(ToScalar(prhs[4]))));
  } else if (command == "get_joint") {
    RequireArgs(nrhs, 2);
    const g1motor::Joint& joint = Bus().joint(ToInt(prhs[1]));
    const char* fields[] = {"gear_ratio", "low", "high", "zero", "turn_counts"};
    plhs[0] = mxCreateStructMatrix(1, 1, 5, fields);
    const double values[] = {
        joint.gear_ratio, joint.low ? static_cast<double>(*joint.low) : NAN,
        joint.high ? static_cast<double>(*joint.high) : NAN,
        static_cast<double>(joint.zero),
        static_cast<double>(joint.turn_counts())};
    for (int field = 0; field < 5; ++field) {
      mxSetFieldByNumber(plhs[0], 0, field,
                         mxCreateDoubleScalar(values[field]));
    }
  } else if (command == "listen") {
    RequireArgs(nrhs, 2);
    const auto bytes = Bus().Listen(ToScalar(prhs[1]));
    plhs[0] = mxCreateNumericMatrix(1, bytes.size(), mxUINT8_CLASS, mxREAL);
    if (!bytes.empty()) {
      std::memcpy(mxGetData(plhs[0]), bytes.data(), bytes.size());
    }
  } else if (command == "encode") {
    RequireArgs(nrhs, 2);
    if (!mxIsStruct(prhs[1])) throw std::runtime_error("expected a struct");
    g1motor::MotorCmd cmd;
    cmd.id = FieldOr0(prhs[1], "id");
    cmd.mode = FieldOr0(prhs[1], "mode");
    cmd.tau = FieldOr0(prhs[1], "tau");
    cmd.vel = FieldOr0(prhs[1], "vel");
    cmd.pos = FieldOr0(prhs[1], "pos");
    cmd.kp = FieldOr0(prhs[1], "kp");
    cmd.kd = FieldOr0(prhs[1], "kd");
    cmd.flag = FieldOr0(prhs[1], "flag") != 0;
    cmd.byte3 = static_cast<std::uint8_t>(FieldOr0(prhs[1], "byte3"));
    const auto frame = cmd.Encode();
    plhs[0] = mxCreateNumericMatrix(1, frame.size(), mxUINT8_CLASS, mxREAL);
    std::memcpy(mxGetData(plhs[0]), frame.data(), frame.size());
  } else if (command == "decode") {
    RequireArgs(nrhs, 2);
    if (!mxIsUint8(prhs[1])) throw std::runtime_error("expected uint8 bytes");
    plhs[0] = StateToStruct(g1motor::MotorState::Decode(
        static_cast<const std::uint8_t*>(mxGetData(prhs[1])),
        mxGetNumberOfElements(prhs[1])));
  } else {
    throw std::runtime_error("unknown command '" + command + "'");
  }
}

}  // namespace

/// @brief MEX entry point: `g1motor_mex(command, args...)`.
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  (void)nlhs;
  // The message is kept outside the try block so that C++ objects are
  // destroyed before mexErrMsgIdAndTxt jumps back into MATLAB.
  std::string error;
  try {
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
      throw std::runtime_error("first argument must be a command name");
    }
    char* text = mxArrayToString(prhs[0]);
    const std::string command(text);
    mxFree(text);
    Dispatch(command, plhs, nrhs, prhs);
  } catch (const std::exception& e) {
    error = e.what();
  }
  if (!error.empty()) mexErrMsgIdAndTxt("g1motor:error", "%s", error.c_str());
}
