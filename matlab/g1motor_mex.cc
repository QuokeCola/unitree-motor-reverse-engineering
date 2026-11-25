/// @file g1motor_mex.cc
/// @brief MEX gateway from the MATLAB package +g1motor to the C++ library.
///
/// Called only by the files in +g1motor; build with build_mex.m. Motor states
/// cross the boundary as rows of 12 numbers and commands as rows of 9:
/// @code
///   state row: id mode tau vel pos temp_coil temp_shell voltage_raw error
///              word18 byte20 byte21
///   cmd row:   id mode tau vel pos kp kd flag byte3
/// @endcode
/// Commands: open, close, listen, transfer, scan, read, enable, disable,
/// restart, command, is_enabled (these need an open bus), and encode,
/// decode_cmd, decode, find_reply, crc (these do not).

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

/// Number of values in a state row.
constexpr int STATE_WIDTH = 12;
/// Number of values in a command row.
constexpr int CMD_WIDTH = 9;

/// Writes one motor state into row `row` of a matrix with `rows` rows.
void fill_state(double* data, mwSize rows, mwSize row,
                const g1motor::MotorState& s) {
  const double values[STATE_WIDTH] = {
      static_cast<double>(s.id),         static_cast<double>(s.mode),
      static_cast<double>(s.tau),        static_cast<double>(s.vel),
      static_cast<double>(s.pos),        static_cast<double>(s.temp_coil),
      static_cast<double>(s.temp_shell), static_cast<double>(s.voltage_raw),
      static_cast<double>(s.error),      static_cast<double>(s.word18),
      static_cast<double>(s.byte20),     static_cast<double>(s.byte21)};
  for (int column = 0; column < STATE_WIDTH; ++column) {
    data[row + rows * column] = values[column];
  }
}

/// @return An N-by-12 matrix with one row per motor.
mxArray* states_to_matrix(const std::map<int, g1motor::MotorState>& states) {
  mxArray* matrix = mxCreateDoubleMatrix(states.size(), STATE_WIDTH, mxREAL);
  mwSize row = 0;
  for (const auto& entry : states) {
    fill_state(mxGetPr(matrix), states.size(), row++, entry.second);
  }
  return matrix;
}

/// @return A 1-by-12 row, or a 0-by-12 matrix if there is no state.
mxArray* state_to_matrix(const std::optional<g1motor::MotorState>& state) {
  mxArray* matrix = mxCreateDoubleMatrix(state ? 1 : 0, STATE_WIDTH, mxREAL);
  if (state) fill_state(mxGetPr(matrix), 1, 0, *state);
  return matrix;
}

/// @return The values of a numeric array as integers.
std::vector<int> to_ints(const mxArray* array) {
  if (!mxIsNumeric(array)) throw std::invalid_argument("expected numbers");
  std::vector<int> values;
  const mwSize count = mxGetNumberOfElements(array);
  for (mwSize i = 0; i < count; ++i) {
    values.push_back(static_cast<int>(std::llround(mxGetPr(array)[i])));
  }
  return values;
}

/// @return The value of a numeric scalar.
double to_scalar(const mxArray* array) {
  if (!mxIsNumeric(array) || mxGetNumberOfElements(array) != 1) {
    throw std::invalid_argument("expected a numeric scalar");
  }
  return mxGetScalar(array);
}

/// @return The value of a numeric scalar as an integer.
int to_int(const mxArray* array) {
  return static_cast<int>(std::llround(to_scalar(array)));
}

/// @return The command described by a 9-value row.
g1motor::MotorCmd to_cmd(const mxArray* array) {
  const auto v = to_ints(array);
  if (v.size() != CMD_WIDTH) throw std::invalid_argument("bad command row");
  g1motor::MotorCmd cmd;
  cmd.id = v[0];
  cmd.mode = v[1];
  cmd.tau = v[2];
  cmd.vel = v[3];
  cmd.pos = v[4];
  cmd.kp = v[5];
  cmd.kd = v[6];
  cmd.flag = v[7] != 0;
  cmd.byte3 = static_cast<std::uint8_t>(v[8]);
  return cmd;
}

/// @return The bytes of a uint8 array.
std::vector<std::uint8_t> to_bytes(const mxArray* array) {
  if (!mxIsUint8(array)) throw std::invalid_argument("expected uint8 bytes");
  const auto* data = static_cast<const std::uint8_t*>(mxGetData(array));
  return std::vector<std::uint8_t>(data, data + mxGetNumberOfElements(array));
}

/// @return A uint8 row holding the given bytes.
mxArray* bytes_to_array(const std::uint8_t* data, std::size_t len) {
  mxArray* array = mxCreateNumericMatrix(1, len, mxUINT8_CLASS, mxREAL);
  if (len > 0) std::memcpy(mxGetData(array), data, len);
  return array;
}

/// Throws if fewer than `needed` arguments were given.
void require_args(int nrhs, int needed) {
  if (nrhs < needed) throw std::invalid_argument("not enough arguments");
}

/// @return The open bus; throws if there is none.
g1motor::MotorBus& bus() {
  if (!g_bus) throw std::runtime_error("bus is not open");
  return *g_bus;
}

/// Closes the bus when MATLAB clears the MEX file or exits.
void close_at_exit() { g_bus.reset(); }

/// Runs one command; throws on any error.
void dispatch(const std::string& command, mxArray* plhs[], int nrhs,
              const mxArray* prhs[]) {
  if (command == "open") {
    require_args(nrhs, 4);
    char* text = mxArrayToString(prhs[1]);
    const std::string port = text ? text : "";
    mxFree(text);
    g_bus.reset();
    g_bus = std::make_unique<g1motor::MotorBus>(port, to_scalar(prhs[2]),
                                                to_scalar(prhs[3]) != 0);
    mexAtExit(close_at_exit);
  } else if (command == "close") {
    g_bus.reset();
  } else if (command == "listen") {
    require_args(nrhs, 2);
    const auto bytes = bus().listen(to_scalar(prhs[1]));
    plhs[0] = bytes_to_array(bytes.data(), bytes.size());
  } else if (command == "transfer") {
    require_args(nrhs, 2);
    plhs[0] = state_to_matrix(bus().transfer(to_cmd(prhs[1])));
  } else if (command == "scan") {
    require_args(nrhs, 2);
    plhs[0] = states_to_matrix(bus().scan(to_int(prhs[1])));
  } else if (command == "read") {
    require_args(nrhs, 2);
    plhs[0] = state_to_matrix(bus().read(to_int(prhs[1])));
  } else if (command == "enable") {
    require_args(nrhs, 2);
    plhs[0] = states_to_matrix(bus().enable(to_ints(prhs[1])));
  } else if (command == "disable") {
    require_args(nrhs, 2);
    bus().disable(to_ints(prhs[1]));
  } else if (command == "restart") {
    require_args(nrhs, 3);
    plhs[0] =
        states_to_matrix(bus().restart(to_ints(prhs[1]), to_scalar(prhs[2])));
  } else if (command == "command") {
    require_args(nrhs, 7);
    plhs[0] = state_to_matrix(bus().command(
        to_int(prhs[1]),
        static_cast<std::int32_t>(std::llround(to_scalar(prhs[2]))),
        to_int(prhs[3]), to_int(prhs[4]), to_int(prhs[5]), to_int(prhs[6])));
  } else if (command == "is_enabled") {
    require_args(nrhs, 2);
    plhs[0] = mxCreateLogicalScalar(bus().is_enabled(to_int(prhs[1])));
  } else if (command == "encode") {
    require_args(nrhs, 2);
    const auto frame = to_cmd(prhs[1]).encode();
    plhs[0] = bytes_to_array(frame.data(), frame.size());
  } else if (command == "decode_cmd") {
    require_args(nrhs, 2);
    const auto bytes = to_bytes(prhs[1]);
    const auto cmd = g1motor::MotorCmd::decode(bytes.data(), bytes.size());
    plhs[0] = mxCreateDoubleMatrix(cmd ? 1 : 0, CMD_WIDTH, mxREAL);
    if (cmd) {
      const double values[CMD_WIDTH] = {
          static_cast<double>(cmd->id),   static_cast<double>(cmd->mode),
          static_cast<double>(cmd->tau),  static_cast<double>(cmd->vel),
          static_cast<double>(cmd->pos),  static_cast<double>(cmd->kp),
          static_cast<double>(cmd->kd),   cmd->flag ? 1.0 : 0.0,
          static_cast<double>(cmd->byte3)};
      std::memcpy(mxGetPr(plhs[0]), values, sizeof(values));
    }
  } else if (command == "decode") {
    require_args(nrhs, 2);
    const auto bytes = to_bytes(prhs[1]);
    plhs[0] = state_to_matrix(
        g1motor::MotorState::decode(bytes.data(), bytes.size()));
  } else if (command == "find_reply") {
    require_args(nrhs, 2);
    const auto bytes = to_bytes(prhs[1]);
    plhs[0] = state_to_matrix(g1motor::find_reply(bytes.data(), bytes.size()));
  } else if (command == "crc") {
    require_args(nrhs, 2);
    const auto bytes = to_bytes(prhs[1]);
    plhs[0] = mxCreateDoubleScalar(
        static_cast<double>(g1motor::crc32_core(bytes.data(), bytes.size())));
  } else {
    throw std::invalid_argument("unknown command '" + command + "'");
  }
}

}  // namespace

/// @brief MEX entry point: `g1motor_mex(command, args...)`.
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  (void)nlhs;
  // The message is kept outside the try block so that C++ objects are
  // destroyed before mexErrMsgIdAndTxt jumps back into MATLAB.
  std::string id;
  std::string message;
  try {
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
      throw std::invalid_argument("first argument must be a command name");
    }
    char* text = mxArrayToString(prhs[0]);
    const std::string command(text);
    mxFree(text);
    dispatch(command, plhs, nrhs, prhs);
  } catch (const g1motor::BusBusyError& e) {
    id = "g1motor:BusBusyError";
    message = e.what();
  } catch (const g1motor::MotorFaultError& e) {
    id = "g1motor:MotorFaultError";
    message = e.what();
  } catch (const std::logic_error& e) {  // invalid_argument, out_of_range
    id = "g1motor:ValueError";
    message = e.what();
  } catch (const std::exception& e) {
    id = "g1motor:RuntimeError";
    message = e.what();
  }
  if (!id.empty()) mexErrMsgIdAndTxt(id.c_str(), "%s", message.c_str());
}
