#include "communication/zqwl_canfd_driver.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "communication/canfd_protocol.hpp"
#include "communication/communication.hpp"
#include "communication/modbus_codec.hpp"

namespace ghand_comm
{

namespace
{

int parse_int_default(const std::string & text, int default_value)
{
  try
  {
    return std::stoi(text);
  }
  catch (...)
  {
    return default_value;
  }
}

speed_t map_baud_rate(uint32_t baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: return B2000000;
  }
}

struct CanTiming
{
  uint8_t sjw = 0;
  uint8_t tseg1 = 0;
  uint8_t tseg2 = 0;
  uint16_t brp = 1;
};

bool compute_zqwl_timing(uint32_t target_bitrate, CanTiming * out)
{
  // ZQWL formula: Baud = 60000000 / ((SJW+1 + TSEG1+1 + TSEG2+1) * BRP)
  // where SJW/TSEG1/TSEG2 are REGISTER values in [0,3]/[0,15]/[0,7].
  // Constraint: total_tq = SJW + TSEG1 + TSEG2 + 3
  // Sample point = (SJW + TSEG1 + 2) / total_tq, target ≈ 75%
  constexpr uint32_t kCanClock = 60000000;
  constexpr uint16_t kMaxBrp = 0x0400;
  constexpr uint8_t kMaxTseg1 = 0x0F;
  constexpr uint8_t kMaxTseg2 = 0x07;

  if (target_bitrate == 0) return false;

  for (uint16_t brp = 1; brp <= kMaxBrp; ++brp)
  {
    const uint32_t tq_hz = kCanClock / brp;
    if (tq_hz == 0) continue;
    const uint32_t total_tq = tq_hz / target_bitrate;
    if (total_tq < 4 || total_tq > (kMaxTseg1 + kMaxTseg2 + 3 + 3)) continue;
    if (tq_hz / total_tq != target_bitrate) continue;

    for (uint8_t sjw = 0; sjw <= 3; ++sjw)
    {
      // tseg1 + sjw + 2 ≈ 0.75 * total_tq  →  tseg1 ≈ 0.75*total_tq - sjw - 2
      int tseg1 = static_cast<int>((total_tq * 3) / 4) - sjw - 2;
      if (tseg1 < 0) continue;
      if (tseg1 > static_cast<int>(kMaxTseg1)) continue;

      // total_tq = sjw + tseg1 + tseg2 + 3  →  tseg2 = total_tq - 3 - sjw - tseg1
      int tseg2 = static_cast<int>(total_tq) - 3 - sjw - tseg1;
      if (tseg2 < 0) continue;
      if (tseg2 > static_cast<int>(kMaxTseg2)) continue;

      // Verify sample point in reasonable range (70-80%)
      const int sp = ((sjw + 1 + tseg1 + 1) * 100) / static_cast<int>(total_tq);
      if (sp < 70 || sp > 80) continue;

      out->brp = brp;
      out->tseg1 = static_cast<uint8_t>(tseg1);
      out->tseg2 = static_cast<uint8_t>(tseg2);
      out->sjw = sjw;
      return true;
    }
  }
  return false;
}

std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> tokens;
  std::string::size_type start = 0;
  std::string::size_type end = text.find(delimiter);
  while (end != std::string::npos)
  {
    std::string token = text.substr(start, end - start);
    if (!token.empty()) tokens.push_back(token);
    start = end + 1;
    end = text.find(delimiter, start);
  }
  std::string token = text.substr(start);
  if (!token.empty()) tokens.push_back(token);
  return tokens;
}

}  // namespace

struct ZqwlCanfdDriver::Impl
{
  std::string device_path_ = "/dev/ttyACM0";
  uint32_t serial_baud_rate_ = 2000000;
  uint32_t can_bitrate_ = 1000000;
  uint32_t dbit_rate_ = 5000000;
  uint8_t can_channel_ = 0;
  uint8_t device_id_ = 0x32;

  // Timeout tracking: if no CAN data received within this window,
  // write() returns false to trigger safe deactivation.
  std::chrono::steady_clock::time_point last_rx_time_;
  bool rx_timeout_initialized_ = false;
  static constexpr auto kRxTimeout = std::chrono::seconds(10);

  int serial_fd_ = -1;
  std::atomic<bool> connected_{false};
  std::atomic<bool> run_thread_{false};
  std::thread recv_thread_;

  std::mutex data_mutex_;
  std::mutex io_mutex_;
  CommunicationData latest_data_;

  std::vector<JointId> valid_joints_;
  std::vector<std::pair<std::string, size_t>> tactile_regions_;

  canfd::PacketAssembler assembler_;

  bool OpenSerial();
  void CloseSerial();
  bool SendConfigFrame(uint8_t func_code, const uint8_t * data);
  bool SetCanParameters();
  bool StartCanChannel();
  bool TransmitFrame(uint32_t can_id, const uint8_t * data, uint8_t len);

  void ReceiveLoop();
  void ProcessCanFrame(const std::vector<uint8_t> & frame);
  void ProcessHeartbeat(const std::vector<uint8_t> & frame);
  void ParseReport(const std::vector<uint8_t> & payload);
  void ParseJointReport(const std::vector<uint8_t> & payload);
  void ParseTactileReport(const std::vector<uint8_t> & payload);
};

ZqwlCanfdDriver::ZqwlCanfdDriver() : impl_(std::make_unique<Impl>()) {}

ZqwlCanfdDriver::~ZqwlCanfdDriver() { close(); }

const char * ZqwlCanfdDriver::get_name() const { return "zqwl_canfd"; }

bool ZqwlCanfdDriver::init(const std::unordered_map<std::string, std::string> & params)
{
  auto it = params.find("device");
  if (it != params.end() && !it->second.empty())
  {
    impl_->device_path_ = it->second;
  }

  impl_->serial_baud_rate_ = static_cast<uint32_t>(parse_int_default(
    params.count("baud_rate") ? params.at("baud_rate") : "", 2000000));
  impl_->can_bitrate_ = static_cast<uint32_t>(parse_int_default(
    params.count("bitrate") ? params.at("bitrate") : "", 1000000));
  impl_->dbit_rate_ = static_cast<uint32_t>(parse_int_default(
    params.count("dbit_rate") ? params.at("dbit_rate") : "", 5000000));
  impl_->can_channel_ = static_cast<uint8_t>(parse_int_default(
    params.count("can_channel") ? params.at("can_channel") : "", 0));
  impl_->device_id_ = static_cast<uint8_t>(parse_int_default(
    params.count("device_id") ? params.at("device_id") : "", 50));

  auto jt = params.find("joint_names");
  if (jt != params.end())
  {
    for (const auto & name : split(jt->second, ','))
    {
      const auto id = joint_name_to_id(name);
      if (id.has_value())
      {
        impl_->valid_joints_.push_back(id.value());
      }
    }
  }
  if (impl_->valid_joints_.empty())
  {
    for (uint8_t i = 0; i < static_cast<uint8_t>(JointId::NUM_JOINTS); ++i)
    {
      impl_->valid_joints_.push_back(static_cast<JointId>(i));
    }
  }

  auto tt = params.find("tactile_region_counts");
  if (tt != params.end())
  {
    const auto counts = split(tt->second, ',');
    const std::vector<std::string> names = {"THUMB", "FF", "MF", "RF", "LF"};
    for (size_t i = 0; i < counts.size() && i < names.size(); ++i)
    {
      try
      {
        impl_->tactile_regions_.emplace_back(names[i], std::stoul(counts[i]));
      }
      catch (...)
      {
        // malformed region count, skip
      }
    }
  }

  // Try multiple device paths (ZQWL can re-enumerate as ttyACM0 or ttyACM1).
  {
    std::vector<std::string> paths = {
      impl_->device_path_, "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"};
    bool opened = false;
    for (const auto & p : paths)
    {
      if (p.empty()) continue;
      impl_->device_path_ = p;
      if (impl_->OpenSerial())
      {
        opened = true;
        break;
      }
    }
    if (!opened) return false;
  }

  if (!impl_->SetCanParameters())
  {
    impl_->CloseSerial();
    return false;
  }

  // Small delay for device to apply parameters.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!impl_->StartCanChannel())
  {
    impl_->CloseSerial();
    return false;
  }

  // ZQWL devices reset their USB interface after the CAN channel is enabled.
  // Close the current serial handle, wait for re-enumeration, then reopen.
  impl_->CloseSerial();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  std::vector<std::string> candidates = {
    impl_->device_path_, "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"};
  bool reopened = false;
  for (const auto & path : candidates)
  {
    if (path.empty()) continue;
    impl_->device_path_ = path;
    if (impl_->OpenSerial())
    {
      reopened = true;
      break;
    }
  }
  if (!reopened)
  {
    return false;
  }

  impl_->connected_.store(true);
  impl_->last_rx_time_ = std::chrono::steady_clock::now();
  impl_->rx_timeout_initialized_ = true;

  // Seed latest_data_ so read() returns valid zero-values immediately,
  // avoiding the Jazzy framework segfault when controllers load without CAN data.
  {
    CommunicationData seed;
    seed.valid = true;
    seed.joints.resize(impl_->valid_joints_.size());
    for (auto & js : seed.joints)
    {
      js.position = 0.0f;
      js.velocity = 0.0f;
      js.status = 0;
      js.error = 0;
    }
    impl_->latest_data_ = std::move(seed);
  }

  impl_->run_thread_.store(true);
  impl_->recv_thread_ = std::thread([this]() { impl_->ReceiveLoop(); });

  return true;
}

bool ZqwlCanfdDriver::read(CommunicationData & out_data)
{
  std::lock_guard<std::mutex> lock(impl_->data_mutex_);
  out_data = impl_->latest_data_;
  bool ret = impl_->connected_.load() && out_data.valid;
  static int rd_cnt = 0;
  if (++rd_cnt % 100 == 0) {
    // fprintf(stderr, "[zqwl_canfd] read #%d ok=%d valid=%d joints=%zu tactile=%zu\n",
// rd_cnt, (int)ret, (int)out_data.valid,
// out_data.joints.size(), out_data.tactile_regions.size());
    // fflush(stderr);
  }
  return ret;
}

bool ZqwlCanfdDriver::write(const std::vector<JointCommand> & commands)
{
  // Safety: if no CAN data has been received within the timeout window,
  // return false so the framework can deactivate cleanly instead of
  // triggering an inexplicable segfault in the controller_manager RT loop.
  if (impl_->rx_timeout_initialized_)
  {
    const auto elapsed = std::chrono::steady_clock::now() - impl_->last_rx_time_;
    if (elapsed > Impl::kRxTimeout)
    {
      impl_->connected_.store(false);
      return false;
    }
  }
  (void)commands;

  static int wr_cnt = 0;
  ++wr_cnt;
  if (wr_cnt <= 5) {
    // fprintf(stderr, "[zqwl_canfd] write #%d entry connected=%d ncmd=%zu\n",
// wr_cnt, (int)impl_->connected_.load(), commands.size());
    // fflush(stderr);
  }

  if (!impl_->connected_.load() || commands.empty())
  {
    if (wr_cnt <= 5) { fprintf(stderr, "[zqwl_canfd] write #%d early return\n", wr_cnt); fflush(stderr); }
    return true;
  }

  const auto sync_payload = canfd::encode_write_single_register(0x0010, 0x0000);
  const uint32_t sync_can_id = canfd::pack_modbus_arbitration_id(
    0x0A, impl_->device_id_, 0, canfd::modbus::kWriteSingleRegister, 1, 1, 0, 0);

  if (wr_cnt <= 5) {
    // fprintf(stderr, "[zqwl_canfd] write #%d sending sync frame fid=%d can_id=0x%X\n",
// wr_cnt, impl_->serial_fd_, sync_can_id);
    // fflush(stderr);
  }

  if (!impl_->TransmitFrame(sync_can_id, sync_payload.data(), static_cast<uint8_t>(sync_payload.size())))
  {
    // fprintf(stderr, "[zqwl_canfd] write #%d sync TransmitFrame FAILED\n", wr_cnt);
    // fflush(stderr);
    return false;
  }

  for (const auto & cmd : commands)
  {
    const auto & reg_map = holding_register_map();
    auto it = reg_map.find(cmd.id);
    if (it == reg_map.end()) continue;

    const auto regs = encode_joint_command(cmd);
    const std::vector<uint8_t> data = {
      static_cast<uint8_t>((regs.first >> 8) & 0xFF),
      static_cast<uint8_t>(regs.first & 0xFF),
      static_cast<uint8_t>((regs.second >> 8) & 0xFF),
      static_cast<uint8_t>(regs.second & 0xFF)};

    const auto payload = canfd::encode_write_multiple_registers(it->second, data);
    const uint32_t can_id = canfd::pack_modbus_arbitration_id(
      0x0A, impl_->device_id_, 0, canfd::modbus::kWriteMultipleRegisters, 1, 1, 0, 0);

    if (!impl_->TransmitFrame(can_id, payload.data(), static_cast<uint8_t>(payload.size())))
    {
      // fprintf(stderr, "[zqwl_canfd] write #%d joint TransmitFrame FAILED\n", wr_cnt);
      // fflush(stderr);
      return false;
    }
  }
  if (wr_cnt <= 5) {
    // fprintf(stderr, "[zqwl_canfd] write #%d done OK\n", wr_cnt);
    // fflush(stderr);
  }
  return true;
}

void ZqwlCanfdDriver::close()
{
  impl_->run_thread_ = false;
  if (impl_->recv_thread_.joinable())
  {
    impl_->recv_thread_.join();
  }
  impl_->CloseSerial();
  impl_->connected_ = false;
}

bool ZqwlCanfdDriver::Impl::OpenSerial()
{
  serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0)
  {
    fprintf(stderr, "[zqwl_canfd] OpenSerial FAILED path=%s errno=%d %s\n",
            device_path_.c_str(), errno, strerror(errno));
    fflush(stderr);
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    CloseSerial();
    return false;
  }

  cfsetispeed(&tty, map_baud_rate(serial_baud_rate_));
  cfsetospeed(&tty, map_baud_rate(serial_baud_rate_));

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit
  tty.c_cflag &= ~PARENB;                        // no parity
  tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                       // no flow control
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 100 ms timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    CloseSerial();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void ZqwlCanfdDriver::Impl::CloseSerial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ZqwlCanfdDriver::Impl::SendConfigFrame(uint8_t func_code, const uint8_t * data)
{
  if (serial_fd_ < 0) return false;

  uint8_t frame[22] = {0};
  frame[0] = 0x49;
  frame[1] = 0x3B;
  frame[2] = func_code;
  frame[3] = 0x57;  // write
  if (data)
  {
    std::memcpy(&frame[4], data, 16);
  }
  frame[20] = 0x45;
  frame[21] = 0x2E;

  const ssize_t written = ::write(serial_fd_, frame, sizeof(frame));
  return written == static_cast<ssize_t>(sizeof(frame));
}

bool ZqwlCanfdDriver::Impl::SetCanParameters()
{
  // Use timing values from the reference BrainCo hand project.
  // Arbitration: 1.154 MHz, sample point ~77%
  // Data phase:  4.615 MHz, sample point ~77%
  uint8_t data[16] = {0};
  data[0] = can_channel_;          // CAN channel
  data[1] = 0x01;                  // use custom baud rate
  data[2] = 1;                     // abit.sjw
  data[3] = 7;                     // abit.tseg1
  data[4] = 2;                     // abit.tseg2
  data[5] = 0;                     // abit.brp high
  data[6] = 4;                     // abit.brp low (= 4)
  data[7] = 1;                     // dbit.sjw
  data[8] = 7;                     // dbit.tseg1
  data[9] = 2;                     // dbit.tseg2
  data[10] = 0;                    // dbit.brp high
  data[11] = 1;                    // dbit.brp low (= 1)
  // data[12..15] reserved
  return SendConfigFrame(0x42, data);
}

bool ZqwlCanfdDriver::Impl::StartCanChannel()
{
  uint8_t data[16] = {0};
  data[0] = 0x01;  // parameter effective
  data[1] = (can_channel_ == 0) ? 0x01 : 0x00;  // CAN0 on
  data[2] = (can_channel_ == 1) ? 0x01 : 0x00;  // CAN1 on
  // data[3..15] reserved
  return SendConfigFrame(0x44, data);
}

bool ZqwlCanfdDriver::Impl::TransmitFrame(uint32_t can_id, const uint8_t * data, uint8_t len)
{
  static int tx_cnt = 0;
  ++tx_cnt;

  if (serial_fd_ < 0 || !connected_.load()) {
    if (tx_cnt <= 3) { fprintf(stderr, "[zqwl_canfd] tx #%d early bail fd=%d connected=%d\n", tx_cnt, serial_fd_, (int)connected_.load()); fflush(stderr); }
    return false;
  }

  const uint8_t dlc = canfd::nearest_valid_canfd_length(len);
  if (dlc > 64) return false;

  std::vector<uint8_t> frame;
  frame.reserve(7 + dlc);
  frame.push_back(0x5A);

  // Byte1: channel high bits + DLC
  uint8_t byte1 = static_cast<uint8_t>((can_channel_ & 0x01) << 7) | (dlc & 0x7F);
  frame.push_back(byte1);

  // Byte2: tx type, channel high bits, frame format, frame type, BRS
  uint8_t byte2 = 0x00;
  byte2 |= static_cast<uint8_t>((can_channel_ & 0x06) << 3);  // bits 4:3
  byte2 |= 0x08;  // extended frame
  byte2 |= 0x02;  // CANFD BRS
  frame.push_back(byte2);

  // CAN ID: 4 bytes, MSB has CANFD flag
  frame.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF) | 0x80);
  frame.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(can_id & 0xFF));

  // Data
  if (data != nullptr && len > 0)
  {
    frame.insert(frame.end(), data, data + len);
  }
  // Pad if dlc > len
  if (dlc > len)
  {
    frame.insert(frame.end(), dlc - len, 0x00);
  }

  frame.push_back(0xA5);

  {
    static int tx_fd_cnt = 0;
    ++tx_fd_cnt;
    if (tx_fd_cnt <= 3) {
      // fprintf(stderr, "[zqwl_canfd] tx #%d about to write fd=%d frame_sz=%zu\n",
// tx_fd_cnt, serial_fd_, frame.size());
      // fflush(stderr);
    }
  }
  std::lock_guard<std::mutex> lock(io_mutex_);
  const ssize_t written = ::write(serial_fd_, frame.data(), frame.size());
  if (written != static_cast<ssize_t>(frame.size()))
  {
    std::cerr << "[zqwl_canfd] TransmitFrame failed: fd=" << serial_fd_
              << " expected=" << frame.size() << " written=" << written
              << " errno=" << errno << std::endl;
    return false;
  }
  return true;
}

void ZqwlCanfdDriver::Impl::ReceiveLoop()
{
  std::vector<uint8_t> buffer;
  buffer.reserve(256);
  size_t parse_offset = 0;

  static int loop_iters = 0;
  static int bytes_read = 0;
  static int zero_reads = 0;
  // fprintf(stderr, "\n[zqwl_canfd] ReceiveLoop STARTED fd=%d\n", serial_fd_);
  // fflush(stderr);

  while (run_thread_.load())
  {
    ++loop_iters;
    if (loop_iters % 500 == 0) {
      // fprintf(stderr, "[zqwl_canfd] recv loop iter=%d bytes=%d zeros=%d buf=%zu\n",
// loop_iters, bytes_read, zero_reads, buffer.size());
      // fflush(stderr);
    }

    if (serial_fd_ < 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    uint8_t byte = 0;
    const ssize_t n = ::read(serial_fd_, &byte, 1);
    if (n <= 0)
    {
      ++zero_reads;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    ++bytes_read;

    buffer.push_back(byte);

    // Trim until we see a known start byte.
    while (parse_offset < buffer.size() && buffer[parse_offset] != 0x49 && buffer[parse_offset] != 0x5A)
    {
      ++parse_offset;
    }

    if (parse_offset >= buffer.size()) continue;

    // Config response: 0x49 0x3B ... 0x45 0x2E, fixed 22 bytes.
    if (buffer[parse_offset] == 0x49 && parse_offset + 1 < buffer.size() && buffer[parse_offset + 1] == 0x3B)
    {
      if (buffer.size() - parse_offset >= 22)
      {
        if (buffer[parse_offset + 20] == 0x45 && buffer[parse_offset + 21] == 0x2E)
        {
          // config response accepted; no further action for now
        }
        parse_offset += 22;
      }
      continue;
    }

    // Heartbeat or CAN data: starts with 0x5A, ends with 0xA5.
    if (buffer[parse_offset] == 0x5A)
    {
      static int frame_detect = 0;
      ++frame_detect;
      if (frame_detect <= 3) {
        // fprintf(stderr, "[zqwl_canfd] recv parser: found 0x5A at offset=%zu buf_sz=%zu byte0=0x%02X byte1=0x%02X\n",
// parse_offset, buffer.size(),
// buffer[parse_offset], parse_offset+1 < buffer.size() ? buffer[parse_offset+1] : 0);
        // fflush(stderr);
      }
      // Determine expected length.
      size_t expected_len = 0;
      if (parse_offset + 1 < buffer.size() && (buffer[parse_offset + 1] == 0xFF || buffer[parse_offset + 1] == 0xFE))
      {
        expected_len = (buffer[parse_offset + 1] == 0xFF) ? 17 : 32;
      }
      else if (parse_offset + 1 < buffer.size())
      {
        const uint8_t dlc = buffer[parse_offset + 1] & 0x7F;
        expected_len = 7 + dlc;  // 0x5A + info1 + info2 + 4 ID bytes + data + 0xA5
      }

      if (expected_len > 0 && buffer.size() - parse_offset >= expected_len)
      {
        if (buffer[parse_offset + expected_len - 1] == 0xA5)
        {
          std::vector<uint8_t> frame(buffer.begin() + parse_offset, buffer.begin() + parse_offset + expected_len);
          if (frame[1] == 0xFF || frame[1] == 0xFE)
          {
            ProcessHeartbeat(frame);
          }
          else
          {
            ProcessCanFrame(frame);
          }
        }
        parse_offset += expected_len;
      }
      // Prevent unbounded growth if no complete frame arrives.
      if (buffer.size() > 256)
      {
        buffer.clear();
        parse_offset = 0;
      }
    }

    // Compact buffer when parse_offset gets large or when a complete frame is consumed.
    if (parse_offset >= 64 || parse_offset >= buffer.size())
    {
      if (parse_offset < buffer.size())
      {
        buffer.erase(buffer.begin(), buffer.begin() + parse_offset);
      }
      else
      {
        buffer.clear();
      }
      parse_offset = 0;
    }
  }
}

void ZqwlCanfdDriver::Impl::ProcessCanFrame(const std::vector<uint8_t> & frame)
{
  if (frame.size() < 7) return;

  canfd::Frame cf;
  cf.id = (static_cast<uint32_t>(frame[3] & 0x7F) << 24) |
          (static_cast<uint32_t>(frame[4]) << 16) |
          (static_cast<uint32_t>(frame[5]) << 8) |
          static_cast<uint32_t>(frame[6]);
  cf.len = frame[1] & 0x7F;
  if (cf.len > 64) cf.len = 64;
  std::memcpy(cf.data, frame.data() + 7, cf.len);
  cf.is_fd = (frame[3] & 0x80) != 0;
  cf.is_extended = (frame[2] & 0x08) != 0;

  canfd::ArbitrationId arb(cf.id);

  // TEMPORARY: log every CAN frame to diagnose hand communication
  static int can_total = 0;
  ++can_total;
  if (can_total <= 20) {
    fprintf(stderr, "[zqwl_canfd] CAN frame #%d id=0x%08X type=%d dev=%d len=%d\n",
            can_total, cf.id, static_cast<int>(arb.frame_type()),
            static_cast<int>(arb.device_id()), cf.len);
    fflush(stderr);
  }

  if (arb.frame_type() != canfd::FrameType::ACTIVE_REPORT)
  {
    return;
  }

  static int can_cnt = 0;
  if (++can_cnt <= 3) {
    // fprintf(stderr, "[zqwl_canfd] recv CAN #%d id=0x%X len=%d\n", can_cnt, cf.id, cf.len);
    // fflush(stderr);
  }

  std::vector<uint8_t> payload;
  if (assembler_.feed(cf, &payload))
  {
    if (can_cnt <= 3) {
      // fprintf(stderr, "[zqwl_canfd] recv CAN #%d assembled payload_sz=%zu\n", can_cnt, payload.size());
      // fflush(stderr);
    }
    ParseReport(payload);
  }
}

void ZqwlCanfdDriver::Impl::ParseReport(const std::vector<uint8_t> & payload)
{
  if (payload.empty()) return;

  // Record timestamp of last valid data for write() timeout check.
  last_rx_time_ = std::chrono::steady_clock::now();
  rx_timeout_initialized_ = true;

  const uint8_t func_code = payload[0];
  if (func_code == 0x91)
  {
    ParseJointReport(payload);
  }
  else if (func_code == 0x92)
  {
    ParseTactileReport(payload);
  }
}

void ZqwlCanfdDriver::Impl::ParseJointReport(const std::vector<uint8_t> & payload)
{
  std::vector<JointState> all_joints;
  for (size_t i = 1; i + 5 < payload.size(); i += 6)
  {
    const uint8_t * p = payload.data() + i;
    uint16_t raw[3] = {
      static_cast<uint16_t>((p[0] << 8) | p[1]),
      static_cast<uint16_t>((p[2] << 8) | p[3]),
      static_cast<uint16_t>((p[4] << 8) | p[5]),
    };
    all_joints.push_back(parse_joint_data(raw, static_cast<JointId>(all_joints.size())));
  }

  CommunicationData data;
  data.valid = true;
  data.joints.reserve(valid_joints_.size());
  for (auto id : valid_joints_)
  {
    const size_t idx = static_cast<size_t>(id);
    if (idx < all_joints.size())
    {
      data.joints.push_back(all_joints[idx]);
    }
    else
    {
      data.joints.emplace_back();
    }
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.joints = std::move(data.joints);
  latest_data_.valid = true;
}

void ZqwlCanfdDriver::Impl::ParseTactileReport(const std::vector<uint8_t> & payload)
{
  if (payload.size() < 3) return;

  const size_t header_size = 2;
  const size_t region_count = tactile_regions_.empty() ? 5 : tactile_regions_.size();
  const size_t resultant_size = region_count * 6;

  if (payload.size() < 1 + header_size + resultant_size) return;

  std::vector<TactileRegion> regions;
  regions.reserve(region_count);
  std::vector<uint16_t> regs(1 + region_count * 3, 0);

  for (size_t i = 0; i < region_count; ++i)
  {
    const size_t r_off = 1 + header_size + i * 6;
    regs[1 + i * 3 + 0] = static_cast<uint16_t>((payload[r_off] << 8) | payload[r_off + 1]);
    regs[1 + i * 3 + 1] = static_cast<uint16_t>((payload[r_off + 2] << 8) | payload[r_off + 3]);
    regs[1 + i * 3 + 2] = static_cast<uint16_t>((payload[r_off + 4] << 8) | payload[r_off + 5]);
  }

  for (size_t i = 0; i < region_count; ++i)
  {
    TactileRegion tr;
    tr.name = (i < tactile_regions_.size()) ? tactile_regions_[i].first : "";
    tr.resultant_force = parse_tactile_resultant(regs.data(), static_cast<int>(i));

    const size_t sensor_count = (i < tactile_regions_.size()) ? tactile_regions_[i].second : 0;
    if (sensor_count > 0)
    {
      const size_t dist_off = 1 + header_size + region_count * 6;
      size_t dist_start = dist_off;
      for (size_t j = 0; j < i; ++j)
      {
        const size_t count = (j < tactile_regions_.size()) ? tactile_regions_[j].second : 0;
        dist_start += count * 3;
      }
      const size_t dist_len = sensor_count * 3;
      if (dist_start + dist_len <= payload.size())
      {
        tr.distributed_forces = parse_tactile_distributed(
          payload.data() + dist_start, dist_len, static_cast<int>(sensor_count));
      }
    }
    regions.push_back(std::move(tr));
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.tactile_regions = std::move(regions);
  latest_data_.valid = true;
}

void ZqwlCanfdDriver::Impl::ProcessHeartbeat(const std::vector<uint8_t> & frame)
{
  if (frame.size() < 4) return;
  const uint8_t status_byte = frame[3];
  const uint8_t bus_status = status_byte & 0x03;
  if (bus_status == 0x03)
  {
    connected_.store(false);
  }
}

}  // namespace ghand_comm
