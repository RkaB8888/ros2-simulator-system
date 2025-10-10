// src/udp_parsers_cpp/src/camera_jpeg_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vector>
#include <cstring>

using std::placeholders::_1;

class CameraJpegParser : public rclcpp::Node {
public:
  CameraJpegParser() : Node("camera_jpeg_parser")
  {
    // RAW: udp_raw_bridge 가 SensorDataQoS(BEST_EFFORT)로 퍼블리시 → 구독도 동일 QoS
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/camera_jpeg_raw", rclcpp::SensorDataQoS(),
      std::bind(&CameraJpegParser::onRaw, this, _1));

    // 산출: 시각화/소비 편의상 RELIABLE depth 10
    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/image_jpeg/compressed", 10);

    buffer_.reserve(kMaxBufferBytes);
    RCLCPP_INFO(get_logger(),
      "camera_jpeg_parser started (MOR+LEN framing + SOI/EOI fallback, max_buf=%zu)",
      kMaxBufferBytes);
  }

private:
  // JPEG markers
  static constexpr uint8_t SOI0 = 0xFF, SOI1 = 0xD8; // Start Of Image
  static constexpr uint8_t EOI0 = 0xFF, EOI1 = 0xD9; // End Of Image
  // Custom framing "MOR\0" + reserved(4) + len_le(4)
  static constexpr char MOR0 = 'M', MOR1 = 'O', MOR2 = 'R', MOR3 = '\0';
  static constexpr size_t kMorHeaderLen = 12; // 4 + 4 + 4
  // buffer
  static constexpr size_t kMaxBufferBytes = 2 * 1024 * 1024; // 2MB
  static constexpr size_t npos = static_cast<size_t>(-1);

  // ====== Callbacks ======
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
  {
    const auto &chunk = msg->data;
    if (chunk.empty()) return;

    // ---- Fast-path: 한 청크에 SOI..EOI 완결 프레임 ----
    {
      size_t c_soi = findMarker(chunk, 0, SOI0, SOI1);
      if (c_soi != npos) {
        size_t c_eoi = findMarker(chunk, c_soi + 2, EOI0, EOI1);
        if (c_eoi != npos && c_eoi + 2 <= chunk.size()) {
          publishJpeg(chunk.data() + c_soi, c_eoi + 2 - c_soi);
          return;
        }
      }
    }

    // ---- 버퍼에 이어붙이기 ----
    if (buffer_.size() + chunk.size() > kMaxBufferBytes) {
      RCLCPP_WARN(get_logger(), "buffer overflow (%zu + %zu) -> reset",
                  buffer_.size(), chunk.size());
      buffer_.clear();
    }
    buffer_.insert(buffer_.end(), chunk.begin(), chunk.end());

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "cam buf=%zuB (+%zuB), try MOR framing -> SOI/EOI fallback",
      buffer_.size(), chunk.size());

    // ---- 1순위: MOR+길이 기반 프레이밍 ----
    const bool mor_made_frame = try_parse_mor_frames();

    // ---- 실패/미완이면: SOI/EOI 백업 스캔 ----
    if (!mor_made_frame) {
      run_fallback_scan();
    }
  }

  // ====== MOR 조립기 (goto 제거) ======
  bool try_parse_mor_frames()
  {
    bool produced = false;

    while (true) {
      const size_t mor = findMor(buffer_, 0);
      if (mor == npos) break; // MOR 헤더 없음 → 빠져나가서 fallback

      // 헤더 미완성
      if (buffer_.size() < mor + kMorHeaderLen) {
        return produced; // 아직 부족 → 더 받기 (fallback은 onRaw가 호출)
      }

      // 길이(LE, mor+8)
      uint32_t jpeg_len = 0;
      std::memcpy(&jpeg_len, buffer_.data() + mor + 8, sizeof(uint32_t));

      // 페이로드 미완성
      if (buffer_.size() < mor + kMorHeaderLen + jpeg_len) {
        return produced; // 아직 부족 → 더 받기
      }

      // 간단 검증: FF D8 시작
      const uint8_t* jpg = buffer_.data() + mor + kMorHeaderLen;
      if (!(jpg[0] == SOI0 && jpg[1] == SOI1)) {
        RCLCPP_WARN(get_logger(),
          "MOR frame doesn't start with SOI, drop 4 bytes and resync");
        buffer_.erase(buffer_.begin() + static_cast<long>(mor),
                      buffer_.begin() + static_cast<long>(mor + 4));
        continue;
      }

      // 프레임 발행
      publishJpeg(jpg, jpeg_len);
      RCLCPP_INFO(get_logger(), "JPEG frame by MOR len=%u", jpeg_len);

      // 소비(헤더+JPEG) 후 다음 프레임 탐색
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<long>(mor + kMorHeaderLen + jpeg_len));
      produced = true;
    }

    return produced;
  }

  // ====== SOI/EOI 백업 스캐너 ======
  void run_fallback_scan()
  {
    size_t pos = 0;
    while (true) {
      size_t soi = findMarker(buffer_, pos, SOI0, SOI1);
      if (soi == npos) {
        shrinkFrontIfNeeded(/*keep_tail=*/64);
        break;
      }
      size_t eoi = findMarker(buffer_, soi + 2, EOI0, EOI1);
      if (eoi == npos) {
        // 다음 SOI 있으면 미완 프레임 드롭하고 재동기화
        size_t next_soi = findMarker(buffer_, soi + 2, SOI0, SOI1);
        if (next_soi != npos) {
          buffer_.erase(buffer_.begin(),
                        buffer_.begin() + static_cast<long>(next_soi));
          pos = 0;
          continue;
        }
        // 더 받기
        break;
      }

      const size_t frame_len = eoi + 2 - soi;
      publishJpeg(buffer_.data() + soi, frame_len);
      RCLCPP_INFO(get_logger(), "JPEG frame by SOI/EOI %zub", frame_len);

      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<long>(eoi + 2));
      pos = 0; // 이어서 다음 프레임 탐색
    }
  }

  // ====== Utils ======
  // 바이트 벡터에서 2바이트 마커 찾기
  static size_t findMarker(const std::vector<uint8_t>& buf, size_t from,
                           uint8_t m0, uint8_t m1)
  {
    if (buf.size() < 2 || from >= buf.size() - 1) return npos;
    for (size_t i = from; i + 1 < buf.size(); ++i) {
      if (buf[i] == m0 && buf[i + 1] == m1) return i;
    }
    return npos;
  }

  // 단일 청크에서 마커 찾기(오버로드)
  static size_t findMarker(const std::vector<uint8_t>& chunk, uint8_t m0, uint8_t m1)
  {
    return findMarker(chunk, 0, m0, m1);
  }

  // "MOR\0" 시그니처 찾기
  static size_t findMor(const std::vector<uint8_t>& buf, size_t from)
  {
    if (buf.size() < 4 || from >= buf.size() - 3) return npos;
    for (size_t i = from; i + 3 < buf.size(); ++i) {
      if (buf[i] == MOR0 && buf[i+1] == MOR1 && buf[i+2] == MOR2 && buf[i+3] == MOR3)
        return i;
    }
    return npos;
  }

  // CompressedImage 발행
  void publishJpeg(const uint8_t* data, size_t len)
  {
    sensor_msgs::msg::CompressedImage out;
    out.header.stamp = now();
    out.header.frame_id = "camera";
    out.format = "jpeg";
    out.data.resize(len);
    std::memcpy(out.data.data(), data, len);
    pub_->publish(out);
  }

  // 버퍼가 너무 커지면 뒤쪽 꼬리만 남기고 잘라 메모리 보호
  void shrinkFrontIfNeeded(size_t keep_tail)
  {
    if (buffer_.size() <= kMaxBufferBytes) return;
    if (keep_tail >= buffer_.size()) return;

    std::vector<uint8_t> tmp;
    tmp.reserve(kMaxBufferBytes);
    tmp.insert(tmp.end(),
               buffer_.end() - static_cast<long>(keep_tail),
               buffer_.end());
    buffer_.swap(tmp);
    RCLCPP_WARN(get_logger(), "buffer shrink -> keep tail %zub", keep_tail);
  }

  // ====== Members ======
  std::vector<uint8_t> buffer_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraJpegParser>());
  rclcpp::shutdown();
  return 0;
}
