# Kế hoạch Xây dựng Thư viện CANopen C++ với Self-Recovery

## Context

Xây dựng một thư viện CANopen hoàn chỉnh bằng C++ với các đặc điểm:
- Tương đương chức năng với Lely Core
- Can thiệp sâu vào CAN_RAW và CAN_ADMIN
- Cơ chế tự phục hồi khi mất/khôi phục CAN
- Cross-platform (Linux, embedded)

---

## Tính năng Bắt buộc (Compliance Matrix)

| Tính năng | Mức độ | CiA Spec | Tác dụng | Trong Plan |
|-----------|--------|----------|-----------|------------|
| **CANopen Base** | 🔴 Bắt buộc | CiA 301 | Nền tảng CANopen | ✅ Phase 3 |
| **CiA 402** | 🔴 Bắt buộc | CiA 402 | Profile điều khiển motor | ⚠️ **THIẾU - cần bổ sung** |
| **PDO** | 🔴 Bắt buộc | CiA 301 | Điều khiển nhanh, cyclic | ✅ Phase 3.4 |
| **SDO** | 🔴 Bắt buộc | CiA 301 | Cấu hình/đọc ghi tham số | ✅ Phase 3.3 |
| **NMT** | 🔴 Bắt buộc | CiA 301 | Quản lý trạng thái node | ✅ Phase 3.2 |
| **Heartbeat** | 🔴 Bắt buộc | CiA 301 | Phát hiện node mất kết nối | ✅ Phase 3.6 |
| **EMCY** | 🔴 Bắt buộc | CiA 301 | Nhận lỗi khẩn cấp từ drive | ✅ Phase 3.6 |
| **SYNC** | 🟠 Rất nên | CiA 301 | Đồng bộ dữ liệu/thời gian | ⚠️ **THIẾU - cần bổ sung** |
| **Timeout Safety** | 🔴 Bắt buộc | CiA 402 | Không để motor chạy vô hạn | ⚠️ **THIẾU - cần bổ sung** |
| **PDO Mapping** | 🔴 Bắt buộc | CiA 301 | Chọn dữ liệu truyền qua PDO | ✅ Phase 3.4 |
| **Error Recovery** | 🟠 Rất nên | CiA 301 | Phục hồi communication | ✅ Phase 4 |
| **LSS** | 🟡 Nên | CiA 305 | Cấu hình node tự động | ✅ Phase 3.5 |

---

## Kiến trúc Tổng quan

```
┌─────────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                            │
│  (User code, Device drivers, Business logic)                        │
├─────────────────────────────────────────────────────────────────────┤
│                     DEVICE MANAGEMENT LAYER                         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐               │
│  │  Device  │ │  Master  │ │  Slave   │ │  Gateway │               │
│  │ Manager  │ │ Service  │ │ Service  │ │ Service  │               │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘               │
├─────────────────────────────────────────────────────────────────────┤
│                      CANopen PROTOCOL LAYER                         │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐│
│  │  NMT   │ │  SDO   │ │  PDO   │ │  LSS   │ │  EMCY  │ │  SYNC  ││
│  │(0x000) │ │(0x1200)│ │(0x1400)│ │(0x1000)│ │(0x1003)│ │(0x1005)││
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘ └────────┘│
├─────────────────────────────────────────────────────────────────────┤
│                      OBJECT DICTIONARY LAYER                         │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  OD (Object Dictionary) - Static/Dynamic                     │   │
│  │  - Standard objects (CiA 301)                              │   │
│  │  - Manufacturer-specific objects                            │   │
│  └─────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│                       CAN FRAME LAYER                               │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  CAN Frame Parser/Builder                                    │   │
│  │  - CAN 2.0 (Standard/Extended)                              │   │
│  │  - FD (Flexible Data Rate)                                  │   │
│  │  - Error Frame handling                                      │   │
│  └─────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│                    CAN DRIVER LAYER (CAN_RAW + CAN_ADMIN)          │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌────────────┐ │
│  │  Raw Socket  │ │ CAN Filter   │ │   CAN Bus    │ │  CAN err   │ │
│  │   Manager    │ │  Manager    │ │   Monitor    │ │  Handler   │ │
│  └──────────────┘ └──────────────┘ └──────────────┘ └────────────┘ │
├─────────────────────────────────────────────────────────────────────┤
│                    RECOVERY & STATE MANAGEMENT                      │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌────────────┐ │
│  │   Bus Off   │ │  TX/RX Error │ │   TX Queue   │ │   State    │ │
│  │  Detector   │ │  Counter     │ │  Preserv.    │ │  Machine   │ │
│  └──────────────┘ └──────────────┘ └──────────────┘ └────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: CAN Driver Layer (CAN_RAW + CAN_ADMIN)

### Task 1.1: SocketCAN Interface Abstraction
**Mục tiêu:** Tạo abstraction cho SocketCAN với deep intervention

```cpp
// can/raw/socket_can.hpp
namespace canopen {

class SocketCAN {
public:
    // Open/close CAN interface
    int open(const std::string& interface, bool loopback = false);
    void close();

    // Deep CAN_RAW intervention
    int set_filter(const can_filter* filters, size_t count);
    int set_mask(uint32_t can_mask);
    int enable_own_filter(bool enable);

    // CAN_ADMIN operations
    int set_bus_state(BusState state);
    int get_bus_state(BusState& state);
    int get_stats(can_stats& stats);
    int get_restart_ms(uint32_t& ms);
    int set_restart_ms(uint32_t ms);

    // Error frame handling
    int bind_error_frame();
    int enable_fd(bool enable);

    // Read/Write
    ssize_t read(can_frame& frame, struct timespec* timeout);
    ssize_t write(const can_frame& frame);

    // File descriptor for select/poll/epoll
    int get_fd() const;
    int get_error_mask() const;

private:
    int sock_fd_{-1};
    std::string interface_;
    BusState state_{BusState::UNKNOWN};
    ErrorCounter error_counts_;
};

}
```

**Deliverables:**
- [ ] `socket_can.hpp` - SocketCAN abstraction
- [ ] `socket_can.cpp` - Linux implementation
- [ ] `socket_can_fake.cpp` - Testing mock

**File locations:**
- `src/can/raw/socket_can.hpp`
- `src/can/raw/socket_can.cpp`
- `src/can/raw/socket_can_fake.cpp`

---

### Task 1.2: CAN Filter Manager
**Mục tiêu:** Quản lý filter cho multiple CANopen nodes

```cpp
// can/raw/filter_manager.hpp
namespace canopen {

class FilterManager {
public:
    // Add individual filter
    int add_filter(uint32_t can_id, uint32_t can_mask);
    int remove_filter(uint32_t can_id);

    // Bulk filter operations
    int set_filters(const std::vector<can_filter>& filters);
    int clear_all_filters();

    // Predefined filter sets
    int set_nmt_filter();
    int set_pdo_filter(uint8_t node_id);
    int set_all_nodes_filter();

    // Get current filters
    std::vector<can_filter> get_active_filters() const;

private:
    std::vector<can_filter> filters_;
    int sock_fd_{-1};
};

}
```

**Deliverables:**
- [ ] `filter_manager.hpp/cpp`

---

### Task 1.3: CAN Bus Monitor & Error Handler
**Mục tiêu:** Monitor bus state và detect errors

```cpp
// can/raw/bus_monitor.hpp
namespace canopen {

struct BusMetrics {
    uint64_t tx_frames;
    uint64_t rx_frames;
    uint64_t tx_errors;
    uint64_t rx_errors;
    uint8_t tx_error_counter;
    uint8_t rx_error_counter;
    BusState state;
    timespec last_error_time;
};

class BusMonitor : public BusObserver {
public:
    // Start monitoring
    void start(const std::string& interface);
    void stop();

    // Metrics
    BusMetrics get_metrics() const;
    void reset_metrics();

    // Callbacks
    void on_error(can_error_frame_t& error);
    void on_state_change(BusState old_state, BusState new_state);
    void on_recovery_attempt(int attempt_number);

    // Error thresholds
    void set_error_threshold(uint8_t epr, uint8_t ecr);
    bool is_error_passive() const;
    bool is_bus_off() const;

private:
    void read_error_frames();
    void update_metrics(const can_error_frame& err);
};

}
```

**Deliverables:**
- [ ] `bus_monitor.hpp/cpp`

---

## Phase 2: CAN Frame Layer

### Task 2.1: CAN Frame Builder/Parser
**Mục tiêu:** Encode/decode CAN frames

```cpp
// can/frame/frame.hpp
namespace canopen {

class CANFrame {
public:
    // Constructors
    CANFrame() = default;
    CANFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
    explicit CANFrame(const can_frame& frame);

    // Accessors
    uint32_t id() const { return can_id_; }
    void set_id(uint32_t id) { can_id_ = id; }
    bool is_extended_id() const { return can_id_ & CAN_EFF_FLAG; }
    bool is_rtr() const { return data_.empty() && len_ > 0; }
    bool is_error() const { return can_id_ == CAN_ERR_FLAG; }

    // Data access
    const uint8_t* data() const { return data_.data(); }
    uint8_t len() const { return len_; }
    std::span<const uint8_t> payload() const { return {data_.data(), len_}; }

    // CAN 2.0 / FD support
    bool is_fd() const { return flags_ & CANFD_FDF; }
    void set_fd(bool enable);

    // Raw conversion
    const can_frame& to_native() const;
    static CANFrame from_native(const can_frame& frame);

private:
    uint32_t can_id_{0};
    uint8_t len_{0};
    uint8_t flags_{0};
    std::array<uint8_t, 64> data_{};
    mutable can_frame native_frame_;
};

}
```

**Deliverables:**
- [ ] `frame.hpp/cpp`

---

### Task 2.2: CANopen Message Factory
**Mục tiêu:** Tạo các CANopen message theo chuẩn CiA 301

```cpp
// can/msg/message_factory.hpp
namespace canopen {

// CANopen COB-ID Calculator
class COBID {
public:
    static constexpr uint32_t NMT             = 0x000;
    static constexpr uint32_t SYNC             = 0x080;
    static constexpr uint32_t EMCY              = 0x080;
    static constexpr uint32_t TPDO1             = 0x180;
    static constexpr uint32_t RPDO1            = 0x200;
    static constexpr uint32_t TPDO2             = 0x280;
    static constexpr uint32_t RPDO2            = 0x300;
    static constexpr uint32_t TPDO3             = 0x380;
    static constexpr uint32_t RPDO3            = 0x400;
    static constexpr uint32_t TPDO4             = 0x480;
    static constexpr uint32_t RPDO4            = 0x500;
    static constexpr uint32_t TSDO             = 0x580;
    static constexpr uint32_t RSDO             = 0x600;
    static constexpr uint32_t LSS_MASTER       = 0x7E4;
    static constexpr uint32_t LSS_SLAVE        = 0x7E5;

    static uint32_t pdo_tx(uint8_t node_id, uint8_t pdo_num);
    static uint32_t pdo_rx(uint8_t node_id, uint8_t pdo_num);
    static uint32_t sdo_tx(uint8_t node_id);
    static uint32_t sdo_rx(uint8_t node_id);
    static uint32_t emcy(uint8_t node_id);
    static uint32_t nmt(node_id);
};

// Message types
enum class MessageType : uint8_t {
    NMT,
    SYNC,
    PDO1_TX, PDO1_RX,
    PDO2_TX, PDO2_RX,
    PDO3_TX, PDO3_RX,
    PDO4_TX, PDO4_RX,
    SDO_TX, SDO_RX,
    TSDO, RSDO,
    LSS_MASTER, LSS_SLAVE,
    EMCY,
    HEARTBEAT,
    TIME
};

class MessageFactory {
public:
    // NMT Messages
    static CANFrame create_nmt(uint8_t node_id, NMTCommand cmd);
    static NMTCommand parse_nmt(const CANFrame& frame);

    // PDO Messages
    static CANFrame create_pdo_tx(uint8_t node_id, uint8_t pdo_num,
                                   const PDOMapping& mapping);
    static CANFrame create_pdo_rx(uint8_t node_id, uint8_t pdo_num,
                                   const PDOMapping& mapping);

    // SDO Messages
    static CANFrame create_sdo_request(uint8_t node_id, uint16_t index,
                                       uint8_t subindex, SDOCommand cmd);
    static CANFrame create_sdo_response(uint8_t node_id, uint16_t index,
                                        uint8_t subindex, const void* data,
                                        size_t len);

    // SYNC
    static CANFrame create_sync(uint8_t counter = 0);

    // Heartbeat
    static CANFrame create_heartbeat(NMTState state);

    // EMCY
    static CANFrame create_emcy(uint8_t node_id, uint16_t error_code,
                                 uint8_t error_register, const uint8_t* msef);

    // LSS
    static CANFrame create_lss_inquiry(LSSCommand cmd);
    static CANFrame create_lss_response(const uint8_t* data, size_t len);

private:
    static void set_cobid(CANFrame& frame, uint32_t cobid);
    static uint32_t get_cobid(const CANFrame& frame);
};

}
```

**Deliverables:**
- [ ] `message_factory.hpp/cpp`

---

## Phase 3: CANopen Protocol Layer

### Task 3.1: Object Dictionary (OD)
**Mục tiêu:** Core data structure cho CANopen

```cpp
// co/object_dictionary/object.hpp
namespace canopen {

// Data types supported
enum class DataType : uint16_t {
    BOOLEAN = 0x01,
    INTEGER8 = 0x02,
    INTEGER16 = 0x03,
    INTEGER32 = 0x04,
    UNSIGNED8 = 0x05,
    UNSIGNED16 = 0x06,
    UNSIGNED32 = 0x07,
    REAL32 = 0x08,
    VISIBLE_STRING = 0x09,
    OCTET_STRING = 0x0A,
    UNICODE_STRING = 0x0B,
    TIME_OF_DAY = 0x0C,
    TIME_DIFFERENCE = 0x0D,
    DOMAIN = 0x0F,
    INTEGER64 = 0x10,
    UNSIGNED64 = 0x1B,
    REAL64 = 0x1C,
};

// Access types
enum class AccessType : uint8_t {
    RO = 0,
    WO = 1,
    RW = 2,
    CO = 3,  // Const
};

// Object entry
struct ObjectEntry {
    uint16_t index;
    uint8_t subindex;
    DataType data_type;
    AccessType access;
    uint8_t size;        // bytes, or 0 for variable
    const char* name;    // optional

    // Callbacks
    std::function<int(ObjectEntry&, const void*, size_t)> on_write;
    std::function<int(const ObjectEntry&, void*, size_t)> on_read;
    std::function<void(ObjectEntry&, uint16_t)> on_change;  // PDO trigger
};

// Object Dictionary
class ObjectDictionary {
public:
    ObjectDictionary() = default;

    // Add objects
    int add_object(const ObjectEntry& entry);
    int add_objects(const ObjectEntry* entries, size_t count);

    // Access objects
    ObjectEntry* get_object(uint16_t index, uint8_t subindex);
    const ObjectEntry* get_object(uint16_t index, uint8_t subindex) const;

    // Iterate
    void for_each_object(std::function<bool(const ObjectEntry&)> callback) const;
    std::vector<const ObjectEntry*> get_pdo_mappable(uint16_t index) const;

    // Load from EDS/DCF
    int load_eds(const std::string& filename);
    int load_dcf(const std::string& filename);

    // Default OD (CiA 301)
    void init_default_od(uint8_t node_id);

private:
    std::map<uint32_t, ObjectEntry> objects_;  // key = (index << 8) | subindex
};

}
```

**Deliverables:**
- [ ] `object.hpp/cpp` - Object entry
- [ ] `object_dictionary.hpp/cpp` - Full OD
- [ ] `object_dictionary_factory.cpp` - Default OD builder

---

### Task 3.2: NMT State Machine
**Mục tiêu:** Implement NMT protocol

```cpp
// co/nmt/nmt.hpp
namespace canopen {

enum class NMTState : uint8_t {
    INITIALISING = 0x00,
    PREOPERATIONAL = 0x7F,
    OPERATIONAL = 0x05,
    STOPPED = 0x04,
};

enum class NMTCommand : uint8_t {
    INITIALISE = 0x81,
    PREOPERATIONAL = 0x80,
    START = 0x01,
    STOP = 0x02,
    RESET_NODE = 0x81,
    RESET_COMMUNICATION = 0x82,
};

class NMTService : public ProtocolService {
public:
    NMTService(ObjectDictionary& od, BusInterface& bus);

    // Send commands (Master)
    void send_command(NMTCommand cmd, uint8_t node_id);
    void send_heartbeat_producer(uint16_t interval_ms);

    // State management
    NMTState get_state() const { return state_; }
    void set_state(NMTState state);

    // Callbacks
    std::function<void(NMTState old_state, NMTState new_state)> on_state_change;
    std::function<void(uint8_t node_id, NMTState state)> on_node_state_change;
    std::function<void()> on_bootup;

protected:
    void handle_frame(const CANFrame& frame) override;
    void on_timeout() override;

private:
    void send_heartbeat();
    void handle_heartbeat(uint8_t node_id, NMTState state);

    NMTState state_{NMTState::INITIALISING};
    std::atomic<NMTState> target_state_{NMTState::INITIALISING};

    // Heartbeat consumer monitoring
    struct NodeState {
        NMTState state;
        timespec last_heartbeat;
        bool timeout_occurred;
    };
    std::map<uint8_t, NodeState> node_states_;

    // Timer for heartbeat producer
    Timer heartbeat_timer_;
};

}
```

**Deliverables:**
- [ ] `nmt.hpp/cpp`
- [ ] `nmt_boot.hpp/cpp` - Boot-up sequence (CiA 302-2)

---

### Task 3.3: SDO Protocol (Client & Server)
**Mục tiêu:** SDO read/write implementation

```cpp
// co/sdo/sdo.hpp
namespace canopen {

enum class SDOCommand : uint8_t {
    DOWNLOAD_INIT = 0x20,
    DOWNLOAD_SEGMENT = 0x00,
    UPLOAD_INIT = 0x40,
    UPLOAD_SEGMENT = 0x60,
    ABORT = 0x80,
};

class SDOServer : public ProtocolService {
public:
    SDOServer(ObjectDictionary& od, BusInterface& bus);

    // Handle incoming SDO requests
    void handle_request(uint8_t node_id, uint16_t index, uint8_t subindex,
                        SDOCommand cmd, const void* data, size_t len);

protected:
    void handle_frame(const CANFrame& frame) override;

private:
    int handle_download(uint8_t node_id, uint16_t index, uint8_t subindex,
                        const void* data, size_t len, uint8_t cmd_byte);
    int handle_upload(uint8_t node_id, uint16_t index, uint8_t subindex,
                      uint8_t cmd_byte);

    ObjectDictionary& od_;
};

class SDOClient {
public:
    using UploadCallback = std::function<void(int error, const void* data, size_t len)>;
    using DownloadCallback = std::function<void(int error)>;

    SDOClient(BusInterface& bus, uint8_t node_id);

    // Synchronous API
    int upload_sync(uint16_t index, uint8_t subindex, void* data, size_t& len);
    int download_sync(uint16_t index, uint8_t subindex, const void* data, size_t len);

    // Asynchronous API
    void upload(uint16_t index, uint8_t subindex, UploadCallback callback);
    void download(uint16_t index, uint8_t subindex, const void* data,
                  size_t len, DownloadCallback callback);

    void abort(uint32_t abort_code);

private:
    void handle_response(const CANFrame& frame);

    BusInterface& bus_;
    uint8_t server_node_id_;

    // Pending request tracking
    struct PendingRequest {
        uint16_t index;
        uint8_t subindex;
        size_t data_size;
        uint8_t* buffer;
        size_t* result_len;
        UploadCallback upload_cb;
        DownloadCallback download_cb;
        timespec timeout;
        int retry_count;
    };
    std::map<uint8_t, PendingRequest> pending_;
    uint8_t next_toggle_{0};
};

}
```

**Deliverables:**
- [ ] `sdo.hpp/cpp`
- [ ] `sdo_client.hpp/cpp`
- [ ] `sdo_server.hpp/cpp`

---

### Task 3.4: PDO Protocol (TPDO & RPDO)
**Mục tiêu:** PDO mapping và transmission

```cpp
// co/pdo/pdo.hpp
namespace canopen {

struct PDOMapping {
    struct MappingEntry {
        uint16_t object_index;
        uint8_t subindex;
        uint8_t bit_length;
    };
    std::vector<MappingEntry> entries;
    size_t data_size() const;
};

enum class PDOTransmissionType : uint8_t {
    SYNCHRONOUS_CYCLIC = 0,      // 0: acyclic, 1-240: cyclic
    SYNCHRONOUS_ACYCLIC = 254,
    ASYNCHRONOUS_SPECIFIC = 252,
    ASYNCHRONOUS_RTR_SYNC = 253,
    ASYNCHRONOUS = 255,
};

class PDOConfiguration {
public:
    void set_transmission_type(PDOTransmissionType type);
    void set_inhibit_time(uint16_t hundred_us);
    void set_event_timer(uint16_t ms);
    void set_sync_start_value(uint8_t value);

    void add_mapping(const PDOMapping::MappingEntry& entry);
    void clear_mappings();

    uint32_t get_cob_id() const { return cob_id_; }
    void set_cob_id(uint32_t id) { cob_id_ = id; }

private:
    uint32_t cob_id_{0};
    PDOTransmissionType transmission_type_{PDOTransmissionType::SYNCHRONOUS_ACYCLIC};
    uint16_t inhibit_time_{0};
    uint16_t event_timer_{0};
    uint8_t sync_start_value_{0};
    PDOMapping mapping_;
};

class TPDO : public ProtocolService {
public:
    TPDO(ObjectDictionary& od, BusInterface& bus, uint8_t pdo_num);

    void configure(const PDOConfiguration& config);
    void start();
    void stop();

    // Trigger transmission
    void send();
    void send_sync(uint8_t sync_counter);

    // Update mapped values from OD
    void update_from_od();

protected:
    void handle_frame(const CANFrame& frame) override;

private:
    void build_frame();

    PDOConfiguration config_;
    uint8_t tx_data_[8]{};
    size_t tx_data_len_{0};
    uint8_t sync_counter_{0};
    bool pending_transmission_{false};
};

class RPDO : public ProtocolService {
public:
    RPDO(ObjectDictionary& od, BusInterface& bus, uint8_t pdo_num);

    void configure(const PDOConfiguration& config);
    void start();
    void stop();

    // Callbacks
    std::function<void(const void* data, size_t len)> on_receive;

protected:
    void handle_frame(const CANFrame& frame) override;
    void update_od();

private:
    PDOConfiguration config_;
    uint8_t rx_data_[8]{};
    std::vector<std::pair<uint16_t, uint8_t>> mapped_objects_;
};

}
```

**Deliverables:**
- [ ] `pdo.hpp/cpp`
- [ ] `pdo_config.hpp/cpp`

---

### Task 3.5: LSS (Layer Setting Services)
**Mục tiêu:** LSS Master/Slave cho cấu hình node

```cpp
// co/lss/lss.hpp
namespace canopen {

struct LSSAddress {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
};

class LSSMaster : public ProtocolService {
public:
    LSSMaster(BusInterface& bus);

    // Switch to config mode
    int switch_to_config(uint8_t node_id);
    int switch_to_config_global();

    // Configure
    int set_node_id(uint8_t node_id);
    int set_bit_timing(uint8_t bit_timing_index);

    // Store
    int store_configuration();

    // Inquire
    int inquire_lss_address(LSSAddress& address);
    int inquire_node_id(uint8_t& node_id);

    // Scan (Lely-specific)
    using ScanCallback = std::function<void(const LSSAddress&)>;
    int slowscan(const LSSAddress& lo, const LSSAddress& hi, ScanCallback callback);
    int fastscan(const LSSAddress& known_bits, uint32_t mask, ScanCallback callback);

    // Activate bit timing
    int activate_bit_timing(uint16_t delay_ms);

protected:
    void handle_frame(const CANFrame& frame) override;

private:
    void handle_switch_response(const CANFrame& frame);
    void handle_config_response(const CANFrame& frame);
    void handle_inquiry_response(const CANFrame& frame);

    LSSState state_{LSSState::IDLE};
    uint8_t selected_node_{0};
    std::vector<LSSAddress> discovered_nodes_;
};

}
```

**Deliverables:**
- [ ] `lss.hpp/cpp`
- [ ] `lss_master.hpp/cpp`

---

### Task 3.6: Emergency & Heartbeat
**Mục tiêu:** Error handling và heartbeat

```cpp
// co/emcy/emcy.hpp
namespace canopen {

struct EmergencyError {
    uint16_t error_code;
    uint8_t error_register;
    uint8_t manufacturer_specific[5];
};

class EMCYService : public ProtocolService {
public:
    EMCYService(ObjectDictionary& od, BusInterface& bus);

    void send_emergency(uint16_t error_code, uint8_t error_register,
                        const uint8_t* msef = nullptr);

    // Error register callback
    std::function<void(uint8_t register_bits)> on_error_register_change;

protected:
    void handle_frame(const CANFrame& frame) override;

private:
    uint16_t error_history_[8]{};
    uint8_t error_history_index_{0};
};

class HeartbeatConsumer : public ProtocolService {
public:
    HeartbeatConsumer(BusInterface& bus);

    void add_producer(uint8_t node_id, uint16_t producer_heartbeat_time);
    void remove_producer(uint8_t node_id);
    void set_consumer_heartbeat_time(uint16_t time_ms);

    struct ProducerStatus {
        NMTState state;
        timespec last_heartbeat;
        bool in_bootup;
        bool lost;
    };
    ProducerStatus get_producer_status(uint8_t node_id);

    std::function<void(uint8_t node_id, NMTState state)> on_producer_state;

protected:
    void handle_frame(const CANFrame& frame) override;
    void on_timeout() override;

private:
    struct ProducerInfo {
        uint16_t heartbeat_time;
        NMTState state;
        timespec last_heartbeat;
        bool in_bootup;
    };
    std::map<uint8_t, ProducerInfo> producers_;
};

}
```

**Deliverables:**
- [ ] `emcy.hpp/cpp`
- [ ] `heartbeat.hpp/cpp`

---

### Task 3.7: SYNC Producer/Consumer ⭐
**Mục tiêu:** Đồng bộ dữ liệu và thời gian

```cpp
// co/sync/sync.hpp
namespace canopen {

class SYNCService : public ProtocolService {
public:
    SYNCService(ObjectDictionary& od, BusInterface& bus);

    // Producer mode
    void start_producer(uint16_t cycle_period_us);
    void stop_producer();

    // Consumer mode
    void start_consumer();
    void stop_consumer();

    // Counter handling
    uint8_t get_counter() const { return counter_; }
    void set_counter(uint8_t counter) { counter_ = counter; }

    // Callbacks
    std::function<void(uint8_t counter)> on_sync;
    std::function<void()> on_sync_lost;

protected:
    void handle_frame(const CANFrame& frame) override;
    void on_timeout() override;

private:
    void send_sync();
    void handle_sync_frame(const CANFrame& frame);

    bool is_producer_{false};
    uint16_t cycle_period_us_{10000};  // 10ms default
    uint8_t counter_{0};
    bool counter_overflow_{false};

    Timer sync_timer_;
    std::chrono::steady_clock::time_point last_sync_time_;

    // Consumer monitoring
    std::chrono::milliseconds consumer_timeout_{100};
    bool sync_received_{false};
};

}
```

**Deliverables:**
- [ ] `sync.hpp/cpp`

---

### Task 3.8: CiA 402 Motor Control Profile ⭐⭐⭐
**Mục tiêu:** Điều khiển motor theo chuẩn CiA 402

```cpp
// co/cia402/cia402_drive.hpp
namespace canopen {

// CiA 402 State Machine
enum class CiA402State : uint16_t {
    NOT_READY_TO_SWITCH_ON   = 0x0000,
    SWITCH_ON_DISABLED       = 0x0001,
    READY_TO_SWITCH_ON       = 0x0002,
    SWITCHED_ON              = 0x0003,
    OPERATION_ENABLED        = 0x0004,
    QUICK_STOP_ACTIVE        = 0x0005,
    FAULT_REACTION_ACTIVE    = 0x0006,
    FAULT                    = 0x0007,
};

// CiA 402 Controlword bits
struct ControlWord402 {
    uint16_t switch_on : 1;           // Bit 0
    uint16_t enable_voltage : 1;       // Bit 1
    uint16_t quick_stop : 1;           // Bit 2
    uint16_t enable_operation : 1;      // Bit 3
    uint16_t operation_mode_specific : 1; // Bit 4
    uint16_t operation_mode_specific2 : 1; // Bit 5
    uint16_t fault_reset : 1;          // Bit 7
    uint16_t halt : 1;                 // Bit 8
    uint16_t operation_mode_specific3 : 1; // Bit 9
};

// CiA 402 Statusword bits
struct StatusWord402 {
    uint16_t ready_to_switch_on : 1;   // Bit 0
    uint16_t switched_on : 1;           // Bit 1
    uint16_t operation_enabled : 1;     // Bit 2
    uint16_t fault : 1;                 // Bit 3
    uint16_t voltage_enabled : 1;       // Bit 4
    uint16_t quick_stop : 1;            // Bit 5
    uint16_t switch_on_disabled : 1;    // Bit 6
    uint16_t warning : 1;               // Bit 7
    uint16_t manufacturer_specific : 1;  // Bit 8
    uint16_t remote : 1;                // Bit 9
    uint16_t target_reached : 1;        // Bit 10
    uint16_t internal_limit_active : 1; // Bit 11
    uint16_t operation_mode_specific : 2; // Bit 12-13
    uint16_t operation_mode_specific2 : 1; // Bit 14
    uint16_t manufacturer_specific2 : 1; // Bit 15
};

// Operation Modes
enum class OperationMode : int8_t {
    NO_MODE           = 0,
    PROFILED_POSITION = 1,
    VELOCITY         = 2,
    PROFILED_VELOCITY = 3,
    TORQUE           = 4,
    PROFILED_TORQUE  = 5,
    HOMING           = 6,
    CYCLIC_SYNC_POSITION = 8,
    CYCLIC_SYNC_VELOCITY = 9,
    CYCLIC_SYNC_TORQUE = 10,
};

class CiA402Drive : public CANopenDevice {
public:
    CiA402Drive(uint8_t node_id, BusInterface& bus);

    // Lifecycle
    void init() override;
    void start() override;
    void stop() override;
    void shutdown() override;

    // CiA 402 State Machine Control
    CiA402State get_state() const { return state_; }
    bool set_state(CiA402State target_state);

    // Quick shutdown (safety)
    void quick_stop();
    void fault_reset();

    // Controlword helpers
    void switch_on() { set_controlword(get_controlword() | 0x000F); }
    void enable_operation() { set_controlword(get_controlword() | 0x000F); }
    void disable_voltage() { set_controlword(get_controlword() & ~0x0002); }

    // Control/Status access
    uint16_t get_controlword() const;
    void set_controlword(uint16_t cw);
    uint16_t get_statusword() const;

    // Operation Mode
    void set_operation_mode(OperationMode mode);
    OperationMode get_operation_mode() const { return current_mode_; }

    // Position Control (Profiled Position Mode - PPM)
    void set_target_position(int32_t position);
    void set_position_profile(int32_t max_speed, uint32_t acceleration, uint32_t deceleration);
    void start_position_move(int32_t position, bool absolute = true);
    void start_relative_move(int32_t offset);
    void halt_position();
    void immediate_position(int32_t position);

    // Velocity Control (Profiled Velocity Mode - PVM)
    void set_target_velocity(int32_t velocity);
    void set_velocity_profile(int32_t max_acceleration, int32_t max_deceleration);
    void start_velocity_move(int32_t velocity);
    void halt_velocity();

    // Torque Control
    void set_target_torque(int16_t torque);
    void set_torque_slope(uint16_t slope);

    // Homing
    void start_homing(int16_t method);
    bool is_homing() const;

    // Cyclic Sync modes
    void set_target_position_cyclic(int32_t position);
    void set_target_velocity_cyclic(int32_t velocity);
    void set_target_torque_cyclic(int16_t torque);

    // Interpolation
    void set_interpolation_time_period(uint16_t period_us, uint8_t index);
    void set_interpolation_data_record(int32_t position, int16_t velocity, int16_t torque);

    // Safety: Timeout Monitoring
    void set_motion_timeout(uint32_t ms);
    void enable_motion_timeout(bool enable);
    bool check_motion_timeout();  // Returns true if timeout occurred

    // Follow error
    void set_following_error_window(uint32_t window);
    void set_following_error_time(uint16_t time_ms);

    // Position limits
    void set_position_range_limits(int32_t min, int32_t max);
    void set_software_position_limits(int32_t min, int32_t max);
    void set_max_motor_speed(uint32_t speed);

    // Callbacks
    std::function<void(CiA402State old_state, CiA402State new_state)> on_state_change;
    std::function<void()> on_target_reached;
    std::function<void(uint16_t error_code)> on_fault;
    std::function<void()> on_homing_complete;
    std::function<void()> on_motion_timeout;  // ⭐ SAFETY

protected:
    void on_nmt_state_change(NMTState old_state, NMTState new_state) override;

private:
    void update_state_from_statusword();
    void send_controlword();
    void monitor_motion();  // ⭐ Timeout monitoring thread

    BusInterface& bus_;
    ObjectDictionary od_;

    CiA402State state_{CiA402State::NOT_READY_TO_SWITCH_ON};
    CiA402State target_state_{CiA402State::SWITCH_ON_DISABLED};
    OperationMode current_mode_{OperationMode::NO_MODE};

    // Object values (cached)
    uint16_t controlword_{0};
    uint16_t statusword_{0};
    int32_t target_position_{0};
    int32_t target_velocity_{0};
    int16_t target_torque_{0};

    // Safety: Motion timeout
    std::atomic<bool> motion_timeout_enabled_{false};
    std::atomic<uint32_t> motion_timeout_ms_{1000};
    std::chrono::steady_clock::time_point last_motion_command_;
    std::thread motion_monitor_thread_;
    std::atomic<bool> motion_monitor_running_{false};

    // SDO for configuration
    SDOClient* sdo_client_{nullptr};
};

// ============================================================
// Timeout Safety Manager - Ngăn motor chạy vô hạn khi mất lệnh
// ============================================================

class TimeoutSafetyManager {
public:
    TimeoutSafetyManager();

    // Register drive for monitoring
    void register_drive(CiA402Drive* drive, uint32_t timeout_ms);
    void unregister_drive(CiA402Drive* drive);

    // Global settings
    void set_default_timeout(uint32_t ms);
    void set_fault_action(enum FaultAction { STOP, QUICK_STOP, HOLD });

    // Manual trigger
    void trigger_safe_stop();

    // Status
    bool is_all_safe() const;
    std::vector<uint8_t> get_timed_out_nodes() const;

    // Callbacks
    std::function<void(uint8_t node_id)> on_timeout;

private:
    void monitoring_loop();

    struct DriveMonitor {
        CiA402Drive* drive;
        uint32_t timeout_ms;
        std::chrono::steady_clock::time_point last_update;
        bool timeout_occurred;
    };

    std::map<uint8_t, DriveMonitor> monitored_drives_;
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};
    uint32_t default_timeout_ms_{1000};
    FaultAction fault_action_{FAULT_ACTION_QUICK_STOP};
};

}
```

**Deliverables:**
- [ ] `cia402_drive.hpp/cpp` - CiA 402 drive implementation
- [ ] `timeout_safety_manager.hpp/cpp` - Timeout safety ⭐

---

### Task 3.9: Object Dictionary Default Values (CiA 301 + CiA 402)
**Mục tiêu:** Predefined OD entries cho standard objects

```cpp
// co/object_dictionary/standard_objects.hpp
namespace canopen {

// CiA 301 Mandatory Objects
struct StandardObjects301 {
    // Device Identity (0x1000)
    static constexpr uint16_t DEVICE_TYPE = 0x1000;
    static constexpr uint16_t ERROR_REGISTER = 0x1001;
    static constexpr uint16_t STATUS_REGISTER = 0x1002;
    static constexpr uint16_t PREDEFINED_ERROR_FIELD = 0x1003;
    static constexpr uint16_t SYNC_COBID = 0x1005;
    static constexpr uint16_t SYNC_MANUFACTURER = 0x1006;
    static constexpr uint16_t ERROR_HISTORY = 0x1003;
    static constexpr uint16_t DEVICE_NAME = 0x1008;
    static constexpr uint16_t HARDWARE_VERSION = 0x1009;
    static constexpr uint16_t SOFTWARE_VERSION = 0x100A;
    static constexpr uint16_t IDENTITY = 0x1018;

    // RxPDO Parameters
    static constexpr uint16_t RXPDO_1_COBID = 0x1400;
    static constexpr uint16_t RXPDO_1_TRANSMISSION = 0x1401;
    static constexpr uint16_t RXPDO_2_COBID = 0x1402;
    static constexpr uint16_t RXPDO_2_TRANSMISSION = 0x1403;
    static constexpr uint16_t RXPDO_3_COBID = 0x1404;
    static constexpr uint16_t RXPDO_3_TRANSMISSION = 0x1405;
    static constexpr uint16_t RXPDO_4_COBID = 0x1406;
    static constexpr uint16_t RXPDO_4_TRANSMISSION = 0x1407;

    // RxPDO Mapping
    static constexpr uint16_t RXPDO_1_MAPPING = 0x1600;
    static constexpr uint16_t RXPDO_2_MAPPING = 0x1601;
    static constexpr uint16_t RXPDO_3_MAPPING = 0x1602;
    static constexpr uint16_t RXPDO_4_MAPPING = 0x1603;

    // TxPDO Parameters
    static constexpr uint16_t TXPDO_1_COBID = 0x1800;
    static constexpr uint16_t TXPDO_1_TRANSMISSION = 0x1801;
    static constexpr uint16_t TXPDO_2_COBID = 0x1802;
    static constexpr uint16_t TXPDO_2_TRANSMISSION = 0x1803;
    static constexpr uint16_t TXPDO_3_COBID = 0x1804;
    static constexpr uint16_t TXPDO_3_TRANSMISSION = 0x1805;
    static constexpr uint16_t TXPDO_4_COBID = 0x1806;
    static constexpr uint16_t TXPDO_4_TRANSMISSION = 0x1807;

    // TxPDO Mapping
    static constexpr uint16_t TXPDO_1_MAPPING = 0x1A00;
    static constexpr uint16_t TXPDO_2_MAPPING = 0x1A01;
    static constexpr uint16_t TXPDO_3_MAPPING = 0x1A02;
    static constexpr uint16_t TXPDO_4_MAPPING = 0x1A03;

    // NMT
    static constexpr uint16_t HEARTBEAT_CONSUMER = 0x1016;
    static constexpr uint16_t HEARTBEAT_PRODUCER = 0x1017;
    static constexpr uint16_t NMT_COBID = 0x1005;

    // SDO Server
    static constexpr uint16_t SDO_SERVER_PARAM = 0x1200;

    // LSS
    static constexpr uint16_t LSS_PARAM = 0x1019;
};

// CiA 402 Drive Objects
struct StandardObjects402 {
    // Controlword/Statusword
    static constexpr uint16_t CONTROLWORD = 0x6040;
    static constexpr uint16_t STATUSWORD = 0x6041;

    // Control modes
    static constexpr uint16_t MODES_OF_OPERATION = 0x6060;
    static constexpr uint16_t MODES_OF_OPERATION_DISPLAY = 0x6061;

    // Position
    static constexpr uint16_t TARGET_POSITION = 0x607A;
    static constexpr uint16_t POSITION_DEMAND = 0x6062;
    static constexpr uint16_t POSITION_ACTUAL = 0x6064;
    static constexpr uint16_t POSITION_WINDOW = 0x6067;
    static constexpr uint16_t POSITION_WINDOW_TIME = 0x6068;
    static constexpr uint16_t FOLLOWING_ERROR_WINDOW = 0x6065;
    static constexpr uint16_t FOLLOWING_ERROR_TIME = 0x6066;

    // Velocity
    static constexpr uint16_t TARGET_VELOCITY = 0x60FF;
    static constexpr uint16_t VELOCITY_DEMAND = 0x606B;
    static constexpr uint16_t VELOCITY_ACTUAL = 0x606C;
    static constexpr uint16_t VELOCITY_WINDOW = 0x606D;
    static constexpr uint16_t VELOCITY_WINDOW_TIME = 0x606E;

    // Torque
    static constexpr uint16_t TARGET_TORQUE = 0x6071;
    static constexpr uint16_t TORQUE_DEMAND = 0x6074;
    static constexpr uint16_t TORQUE_ACTUAL = 0x6077;
    static constexpr uint16_t TORQUE_MAX_DEMAND = 0x6072;

    // Profiles
    static constexpr uint16_t PROFILE_VELOCITY = 0x6081;
    static constexpr uint16_t PROFILE_ACCELERATION = 0x6083;
    static constexpr uint16_t PROFILE_DECELERATION = 0x6084;
    static constexpr uint16_t QUICK_STOP_DECELERATION = 0x6085;
    static constexpr uint16_t MOTOR_OFFSET = 0x6086;

    // Homing
    static constexpr uint16_t HOMING_METHOD = 0x6098;
    static constexpr uint16_t HOMING_SPEED_SWITCH = 0x6099;
    static constexpr uint16_t HOMING_SPEED_ZERO = 0x609A;
    static constexpr uint16_t HOMING_ACCELERATION = 0x609C;

    // Limits
    static constexpr uint16_t MAX_VELOCITY = 0x6080;
    static constexpr uint16_t MAX_MOTOR_SPEED = 0x6081;
    static constexpr uint16_t VELOCITY_LIMIT = 0x6081;
    static constexpr uint16_t TORQUE_LIMIT = 0x6081;
    static constexpr uint16_t POSITION_LIMIT_MIN = 0x607B;
    static constexpr uint16_t POSITION_LIMIT_MAX = 0x607B;
    static constexpr uint16_t SOFTWARE_LIMIT_MIN = 0x607D;
    static constexpr uint16_t SOFTWARE_LIMIT_MAX = 0x607D;

    // Interpolation
    static constexpr uint16_t INTERPOLATION_TIME_PERIOD = 0x60C2;
    static constexpr uint16_t INTERPOLATION_DATA_RECORD = 0x60C1;
    static constexpr uint16_t INTERPOLATION_SUBMODE_SELECT = 0x60C0;

    // Touch Probe
    static constexpr uint16_t TOUCH_PROBE_FUNCTION = 0x60B8;
    static constexpr uint16_t TOUCH_PROBE_STATUS = 0x60B9;
    static constexpr uint16_t TOUCH_PROBE_POS_1 = 0x60BA;
    static constexpr uint16_t TOUCH_PROBE_POS_2 = 0x60BC;

    // Digital I/O
    static constexpr uint16_t DIG_OUTPUTS = 0x60FE;
    static constexpr uint16_t DIG_INPUTS = 0x60FD;
    static constexpr uint16_t DIG_OUTPUT_PHYS = 0x60FE;
    static constexpr uint16_t DIG_INPUT_PHYS = 0x60FD;
};

// Build default OD with CiA 301 + CiA 402
ObjectDictionary build_default_od(uint8_t node_id, bool is_drive = false);

}
```

**Deliverables:**
- [ ] `standard_objects.hpp` - Standard object definitions
- [ ] `standard_objects.cpp` - Default OD builder

---

## Phase 4: Recovery & State Management

### Task 4.1: CAN Bus State Manager
**Mục tiêu:** Theo dõi và quản lý bus state với auto-recovery

```cpp
// recovery/bus_state_manager.hpp
namespace canopen {

enum class BusState : uint8_t {
    UNKNOWN = 0,
    ACTIVE = 1,
    WARNING = 2,
    PASSIVE = 3,
    BUS_OFF = 4,
    STOPPED = 5,
};

class BusStateManager {
public:
    BusStateManager(SocketCAN& can, BusMonitor& monitor);

    // State monitoring
    BusState get_state() const;
    bool is_operational() const;
    bool is_recovering() const { return recovering_.load(); }

    // Recovery control
    void set_auto_recovery(bool enable, uint32_t restart_delay_ms = 1000);
    void recovery_now();
    void stop_recovery();

    // State preservation
    void save_state();
    void restore_state();

    // Statistics
    struct RecoveryStats {
        uint32_t total_recoveries;
        uint32_t successful_recoveries;
        uint32_t failed_recoveries;
        uint64_t total_downtime_ms;
        timespec last_recovery_time;
        timespec last_bus_off_time;
    };
    RecoveryStats get_stats() const;

    // Callbacks
    std::function<void(BusState old_state, BusState new_state)> on_state_change;
    std::function<void()> on_recovery_start;
    std::function<void(bool success)> on_recovery_complete;
    std::function<void()> on_permanent_failure;

private:
    void monitor_loop();
    void perform_recovery();
    void check_recovery_timeout();

    SocketCAN& can_;
    BusMonitor& monitor_;

    std::atomic<BusState> state_{BusState::UNKNOWN};
    std::atomic<bool> recovering_{false};
    std::atomic<bool> auto_recovery_{true};

    uint32_t restart_delay_ms_{1000};
    std::thread monitor_thread_;
    std::atomic<bool> running_{false};

    RecoveryStats stats_{};
    mutable std::mutex stats_mutex_;

    // State preservation
    struct SavedState {
        NMTState nmt_state;
        std::map<uint16_t, std::vector<uint8_t>> od_values;
        std::vector<PendingTXFrame> tx_queue;
        timespec saved_at;
    };
    std::optional<SavedState> saved_state_;
    std::mutex state_mutex_;
};

}
```

**Deliverables:**
- [ ] `bus_state_manager.hpp/cpp`

---

### Task 4.2: TX Queue with State Preservation
**Mục tiêu:** Preserve TX queue during bus-off

```cpp
// recovery/tx_queue.hpp
namespace canopen {

struct PendingTXFrame {
    CANFrame frame;
    timespec timestamp;
    uint8_t retry_count;
    uint32_t can_id;

    bool is_high_priority() const {
        return (can_id & 0x700) == 0x000;  // NMT
    }
};

class TXQueueManager {
public:
    TXQueueManager(size_t max_queue_size = 1024);

    // Queue operations
    bool enqueue(const CANFrame& frame, bool high_priority = false);
    std::optional<CANFrame> dequeue();
    void clear();

    // During recovery
    void pause();
    void resume();

    // State preservation
    std::vector<PendingTXFrame> get_pending_frames() const;
    size_t restore_pending_frames(const std::vector<PendingTXFrame>& frames);
    void remove_acknowledged_frames(const std::vector<uint32_t>& acked_ids);

    // Monitoring
    size_t size() const;
    size_t high_priority_count() const;
    bool is_empty() const;

    // Priority ordering
    // 1. NMT (highest)
    // 2. SYNC
    // 3. EMCY
    // 4. SDO responses
    // 5. PDO
    // 6. LSS
    // 7. Heartbeat (lowest)

private:
    size_t get_priority(const CANFrame& frame) const;

    std::vector<PendingTXFrame> queue_;
    std::vector<PendingTXFrame> high_priority_queue_;
    mutable std::mutex mutex_;
    std::atomic<bool> paused_{false};
    size_t max_size_;
};

}
```

**Deliverables:**
- [ ] `tx_queue.hpp/cpp`

---

### Task 4.3: Node Health Monitor
**Mục tiêu:** Monitor individual nodes và tự phục hồi từng node

```cpp
// recovery/node_health_monitor.hpp
namespace canopen {

struct NodeHealth {
    uint8_t node_id;
    NMTState current_state;
    NMTState expected_state;
    uint8_t heartbeat_producer_time;
    timespec last_heartbeat;
    uint8_t consecutive_timeouts;
    uint8_t sdo_failures;
    bool configuration_complete;
    std::vector<uint16_t> pdo_errors;
};

class NodeHealthMonitor {
public:
    NodeHealthMonitor(BusInterface& bus, NMTService& nmt);

    // Register nodes
    void register_node(uint8_t node_id, uint16_t heartbeat_time);
    void unregister_node(uint8_t node_id);

    // Update expectations
    void set_expected_state(uint8_t node_id, NMTState state);

    // Health queries
    NodeHealth get_health(uint8_t node_id) const;
    std::vector<uint8_t> get_unhealthy_nodes() const;

    // Callbacks
    std::function<void(uint8_t node_id, const std::string& error)> on_node_error;
    std::function<void(uint8_t node_id)> on_node_recovered;
    std::function<void(uint8_t node_id, NMTState old_state, NMTState new_state)> on_state_change;

    // Recovery actions
    void reset_node(uint8_t node_id);
    void reconfigure_node(uint8_t node_id);
    void request_boot(uint8_t node_id);

protected:
    void check_heartbeats();
    void on_heartbeat_timeout(uint8_t node_id);

private:
    std::map<uint8_t, NodeHealth> nodes_;
    BusInterface& bus_;
    NMTService& nmt_;
    Timer heartbeat_check_timer_;
};

}
```

**Deliverables:**
- [ ] `node_health_monitor.hpp/cpp`

---

### Task 4.4: Complete Recovery Orchestrator
**Mục tiêu:** Coordinate tất cả recovery mechanisms

```cpp
// recovery/recovery_orchestrator.hpp
namespace canopen {

struct RecoveryPolicy {
    bool auto_recover_bus{true};
    uint32_t bus_recovery_delay_ms{1000};
    uint32_t max_recovery_attempts{5};
    uint32_t node_timeout_ms{5000};
    bool preserve_tx_queue{true};
    bool restore_nmt_state{true};

    // Exponential backoff
    bool use_exponential_backoff{true};
    uint32_t initial_backoff_ms{1000};
    uint32_t max_backoff_ms{30000};
};

class RecoveryOrchestrator {
public:
    RecoveryOrchestrator(BusStateManager& bus_manager,
                         TXQueueManager& tx_queue,
                         NodeHealthMonitor& node_monitor,
                         NMTService& nmt);

    void set_policy(const RecoveryPolicy& policy);
    const RecoveryPolicy& get_policy() const { return policy_; }

    // Automatic mode
    void enable_auto_recovery();
    void disable_auto_recovery();

    // Manual recovery
    RecoveryResult recover();
    RecoveryResult recover_node(uint8_t node_id);

    // Status
    bool is_recovering() const;
    RecoveryPhase get_current_phase() const;
    float get_recovery_progress() const;

    enum class RecoveryPhase : uint8_t {
        IDLE,
        DETECTING_FAILURE,
        PRESERVING_STATE,
        WAITING_FOR_BUS,
        RECONNECTING,
        RESTORING_STATE,
        VERIFYING,
        RECOVERING_NODES,
        COMPLETE
    };

    struct RecoveryResult {
        bool success;
        RecoveryPhase final_phase;
        std::string error_message;
        timespec duration;
        std::vector<uint8_t> recovered_nodes;
    };

    // Events
    std::function<void(RecoveryPhase phase, float progress)> on_phase_change;
    std::function<void(const RecoveryResult& result)> on_recovery_complete;

private:
    void recovery_loop();
    bool check_bus_available();
    bool verify_can_interface();
    bool restore_application_state();
    bool recover_nodes();

    RecoveryPolicy policy_;
    BusStateManager& bus_manager_;
    TXQueueManager& tx_queue_;
    NodeHealthMonitor& node_monitor_;
    NMTService& nmt_;

    std::atomic<RecoveryPhase> phase_{RecoveryPhase::IDLE};
    std::atomic<float> progress_{0.0f};
    std::atomic<bool> recovery_active_{false};
    std::thread recovery_thread_;

    std::atomic<uint32_t> recovery_attempts_{0};
    uint32_t backoff_ms_{1000};
};

}
```

**Deliverables:**
- [ ] `recovery_orchestrator.hpp/cpp`

---

## Phase 5: Device Management

### Task 5.1: CANopen Device Base Class
**Mục tiêu:** Base class cho Master/Slave devices

```cpp
// device/canopen_device.hpp
namespace canopen {

class CANopenDevice {
public:
    CANopenDevice(uint8_t node_id);
    virtual ~CANopenDevice() = default;

    // Lifecycle
    virtual void init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void shutdown() = 0;

    // NMT
    NMTState get_nmt_state() const { return nmt_state_; }
    void set_nmt_state(NMTState state);

    // Object Dictionary
    ObjectDictionary& get_od() { return od_; }
    const ObjectDictionary& get_od() const { return od_; }

    // Read/Write OD
    template<typename T>
    int read_od(uint16_t index, uint8_t subindex, T& value);
    template<typename T>
    int write_od(uint16_t index, uint8_t subindex, const T& value);

protected:
    uint8_t node_id_;
    ObjectDictionary od_;
    NMTState nmt_state_{NMTState::INITIALISING};

    // Subclasses override
    virtual void on_nmt_state_change(NMTState old_state, NMTState new_state) {}
    virtual void on_od_value_change(uint16_t index, uint8_t subindex) {}
};

}
```

**Deliverables:**
- [ ] `canopen_device.hpp`

---

### Task 5.2: CANopen Master
**Mục tiêu:** Master device với full control

```cpp
// device/canopen_master.hpp
namespace canopen {

class CANopenMaster : public CANopenDevice {
public:
    CANopenMaster(uint8_t node_id, BusInterface& bus);

    // From CANopenDevice
    void init() override;
    void start() override;
    void stop() override;
    void shutdown() override;

    // Node management
    void add_slave(uint8_t node_id, uint16_t heartbeat_time);
    void remove_slave(uint8_t node_id);

    // Boot-up sequence (CiA 302-2)
    void boot_slave(uint8_t node_id);
    void boot_all_slaves();

    // SDO access
    SDOClient& get_sdo_client() { return sdo_client_; }

    // PDO
    TPDO* get_tpdo(uint8_t num);
    RPDO* get_rpdo(uint8_t num);

    // LSS
    LSSMaster& get_lss_master() { return lss_master_; }

    // Configuration
    int configure_slave_from_dcf(uint8_t node_id, const std::string& dcf_path);

protected:
    void on_nmt_state_change(NMTState old_state, NMTState new_state) override;

private:
    void handle_slave_bootup(uint8_t node_id);
    void configure_slave_objects(uint8_t node_id);

    BusInterface& bus_;
    SDOServer sdo_server_;
    SDOClient sdo_client_;
    LSSMaster lss_master_;

    std::array<std::unique_ptr<TPDO>, 4> tpdos_;
    std::array<std::unique_ptr<RPDO>, 4> rpdos_;

    struct SlaveInfo {
        uint16_t heartbeat_time;
        NMTState expected_state;
        bool configured;
        std::string dcf_path;
    };
    std::map<uint8_t, SlaveInfo> slaves_;
};

}
```

**Deliverables:**
- [ ] `canopen_master.hpp/cpp`

---

### Task 5.3: CANopen Slave
**Mục tiêu:** Slave device implementation

```cpp
// device/canopen_slave.hpp
namespace canopen {

class CANopenSlave : public CANopenDevice {
public:
    CANopenSlave(uint8_t node_id, BusInterface& bus);

    // From CANopenDevice
    void init() override;
    void start() override;
    void stop() override;
    void shutdown() override;

    // Send data
    void send_pdo(uint8_t pdo_num);
    void send_emcy(uint16_t error_code, uint8_t error_register,
                   const uint8_t* msef = nullptr);

    // Configuration
    void set_heartbeat_producer_time(uint16_t time_ms);
    void set_identity(uint32_t vendor_id, uint32_t product_code,
                     uint32_t revision, uint32_t serial);

protected:
    void on_nmt_state_change(NMTState old_state, NMTState new_state) override;

private:
    BusInterface& bus_;
    SDOServer sdo_server_;
    HeartbeatConsumer heartbeat_consumer_;
    EMCYService emcy_;

    std::array<std::unique_ptr<RPDO>, 4> rpdos_;
    std::array<std::unique_ptr<TPDO>, 4> tpdos_;

    bool bootup_message_sent_{false};
};

}
```

**Deliverables:**
- [ ] `canopen_slave.hpp/cpp`

---

### Task 5.4: CiA 402 Motor Drive Device ⭐⭐⭐
**Mục tiêu:** Motor drive device với đầy đủ CiA 402 features

```cpp
// device/cia402_motor_drive.hpp
namespace canopen {

class CiA402MotorDrive : public CiA402Drive {
public:
    CiA402MotorDrive(uint8_t node_id, BusInterface& bus);

    // Convenience methods
    void home_and_enable(int16_t homing_method = -1);
    void enable_and_run();
    void safe_stop();

    // Diagnostics
    bool is_enabled() const;
    bool is_moving() const;
    int32_t get_actual_position() const;
    int32_t get_actual_velocity() const;
    int16_t get_actual_torque() const;

    // Error handling
    bool has_fault() const;
    uint16_t get_fault_code() const;
    void clear_faults();
};

}
```

**Deliverables:**
- [ ] `cia402_motor_drive.hpp/cpp`

---

### Task 5.5: Timeout Safety Controller ⭐⭐⭐
**Mục tiêu:** Global timeout safety - ngăn motor chạy vô hạn khi mất lệnh

```cpp
// device/timeout_safety_controller.hpp
namespace canopen {

class TimeoutSafetyController {
public:
    TimeoutSafetyController();

    // Singleton accessor
    static TimeoutSafetyController& instance();

    // Register drives for safety monitoring
    void register_drive(uint8_t node_id, CiA402Drive* drive);
    void unregister_drive(uint8_t node_id);
    void set_drive_timeout(uint8_t node_id, uint32_t timeout_ms);

    // Global safety
    void enable_safety(bool enable);
    bool is_safety_enabled() const { return safety_enabled_; }

    // Manual emergency stop
    void emergency_stop_all();
    void resume_all();

    // Status
    struct SafetyStatus {
        bool safety_enabled;
        std::vector<uint8_t> monitored_nodes;
        std::vector<uint8_t> timed_out_nodes;
        uint32_t last_command_timestamp;
    };
    SafetyStatus get_status() const;

    // Configuration
    void set_default_timeout(uint32_t ms);
    void set_stop_mode(enum StopMode { STOP_MOTOR, QUICK_STOP, HOLD_POSITION });
    void set_recovery_mode(enum RecoveryMode { AUTO_RECOVERY, MANUAL_RECOVERY });

    // Callbacks
    std::function<void(uint8_t node_id, uint32_t timeout_ms)> on_drive_timeout;
    std::function<void()> on_emergency_stop;

private:
    void safety_monitor_loop();

    std::atomic<bool> safety_enabled_{true};
    std::atomic<bool> running_{false};
    std::thread safety_thread_;

    struct DriveSafetyInfo {
        CiA402Drive* drive;
        uint32_t timeout_ms;
        std::chrono::steady_clock::time_point last_command;
        std::atomic<bool> timed_out{false};
    };

    std::map<uint8_t, DriveSafetyInfo> monitored_drives_;
    mutable std::mutex drives_mutex_;

    StopMode stop_mode_{QUICK_STOP};
    RecoveryMode recovery_mode_{AUTO_RECOVERY};
    uint32_t default_timeout_ms_{500};
};

}
```

**Deliverables:**
- [ ] `timeout_safety_controller.hpp/cpp` - Global timeout safety ⭐

---

## Phase 6: Event Loop & Async

### Task 6.1: Event Loop Framework
**Mục tiêu:** Async event handling giống Lely fiber

```cpp
// ev/event_loop.hpp
namespace canopen {

class EventLoop {
public:
    EventLoop();
    ~EventLoop();

    // Timer operations
    TimerHandle add_timer(std::chrono::milliseconds interval,
                          std::function<void()> callback, bool repeating = true);
    void cancel_timer(TimerHandle handle);

    // File descriptor monitoring
    WatchHandle add_watch(int fd, uint32_t events,
                          std::function<void(uint32_t)> callback);
    void remove_watch(WatchHandle handle);

    // Delayed execution
    void post_delayed(std::chrono::milliseconds delay,
                      std::function<void()> callback);

    // Execution control
    void run();
    void stop();
    bool is_running() const;

    // Integration with CAN
    void register_can_socket(SocketCAN& can);
    void process_can_frame(const CANFrame& frame);

private:
    void process_events(int timeout_ms);

    int epoll_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread loop_thread_;

    std::map<TimerHandle, TimerInfo> timers_;
    std::map<WatchHandle, WatchInfo> watches_;
    TimerHandle next_timer_handle_{1};
    WatchHandle next_watch_handle_{1};
};

}
```

**Deliverables:**
- [ ] `event_loop.hpp/cpp`

---

## Phase 7: Gateway (Optional but Lely-compatible)

### Task 7.1: ASCII Gateway (CiA 309)
**Mục tiêu:** ASCII protocol cho debugging

```cpp
// gw/ascii_gateway.hpp
namespace canopen {

// ASCII Gateway Protocol (CiA 309-1)
class ASCIIGateway {
public:
    ASCIIGateway(CANopenMaster& master, std::istream& input,
                 std::ostream& output);

    void process_command(const std::string& command);
    std::string get_prompt();

private:
    std::string handle_sdo_upload(const std::vector<std::string>& args);
    std::string handle_sdo_download(const std::vector<std::string>& args);
    std::string handle_nmt_command(const std::vector<std::string>& args);
    std::string handle_pdo_read(const std::vector<std::string>& args);
    std::string handle_pdo_write(const std::vector<std::string>& args);
    std::string handle_status();
    std::string handle_help();

    CANopenMaster& master_;
    std::ostream& output_;
};

}
```

**Deliverables:**
- [ ] `ascii_gateway.hpp/cpp`

---

## File Structure

```
canopen-lib/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── include/
│   └── canopen/
│       ├── can/
│       │   ├── raw/
│       │   │   ├── socket_can.hpp
│       │   │   ├── filter_manager.hpp
│       │   │   └── bus_monitor.hpp
│       │   ├── frame/
│       │   │   └── frame.hpp
│       │   └── msg/
│       │       └── message_factory.hpp
│       ├── co/
│       │   ├── object_dictionary/
│       │   │   ├── object.hpp
│       │   │   ├── object_dictionary.hpp
│       │   │   └── standard_objects.hpp
│       │   ├── nmt/
│       │   │   └── nmt.hpp
│       │   ├── sdo/
│       │   │   ├── sdo.hpp
│       │   │   ├── sdo_client.hpp
│       │   │   └── sdo_server.hpp
│       │   ├── pdo/
│       │   │   └── pdo.hpp
│       │   ├── lss/
│       │   │   └── lss.hpp
│       │   ├── emcy/
│       │   │   └── emcy.hpp
│       │   ├── heartbeat/
│       │   │   └── heartbeat.hpp
│       │   ├── sync/
│       │   │   └── sync.hpp
│       │   └── cia402/
│       │       └── cia402_drive.hpp
│       ├── recovery/
│       │   ├── bus_state_manager.hpp
│       │   ├── tx_queue.hpp
│       │   ├── node_health_monitor.hpp
│       │   └── recovery_orchestrator.hpp
│       ├── device/
│       │   ├── canopen_device.hpp
│       │   ├── canopen_master.hpp
│       │   ├── canopen_slave.hpp
│       │   ├── cia402_motor_drive.hpp
│       │   └── timeout_safety_controller.hpp
│       ├── ev/
│       │   └── event_loop.hpp
│       └── gw/
│           └── ascii_gateway.hpp
├── src/
│   ├── can/
│   │   ├── raw/
│   │   │   └── socket_can.cpp
│   │   ├── frame/
│   │   │   └── frame.cpp
│   │   └── msg/
│   │       └── message_factory.cpp
│   ├── co/
│   │   ├── object_dictionary/
│   │   │   ├── object.cpp
│   │   │   ├── object_dictionary.cpp
│   │   │   └── standard_objects.cpp
│   │   ├── nmt/
│   │   │   └── nmt.cpp
│   │   ├── sdo/
│   │   │   ├── sdo.cpp
│   │   │   ├── sdo_client.cpp
│   │   │   └── sdo_server.cpp
│   │   ├── pdo/
│   │   │   └── pdo.cpp
│   │   ├── lss/
│   │   │   └── lss.cpp
│   │   ├── emcy/
│   │   │   └── emcy.cpp
│   │   ├── heartbeat/
│   │   │   └── heartbeat.cpp
│   │   ├── sync/
│   │   │   └── sync.cpp
│   │   └── cia402/
│   │       └── cia402_drive.cpp
│   ├── recovery/
│   │   ├── bus_state_manager.cpp
│   │   ├── tx_queue.cpp
│   │   ├── node_health_monitor.cpp
│   │   └── recovery_orchestrator.cpp
│   ├── device/
│   │   ├── canopen_master.cpp
│   │   ├── canopen_slave.cpp
│   │   ├── cia402_motor_drive.cpp
│   │   └── timeout_safety_controller.cpp
│   ├── ev/
│   │   └── event_loop.cpp
│   └── gw/
│       └── ascii_gateway.cpp
├── test/
│   ├── can/
│   ├── co/
│   ├── recovery/
│   └── device/
└── examples/
    ├── master_example.cpp
    ├── slave_example.cpp
    ├── recovery_demo.cpp
    └── **motor_control_example.cpp**  // CiA 402 demo
```

---

## PHẦN 8: TEST & DEMO - ĐIỀU KHIỂN ĐỘNG CƠ

### 8.1 Cấu trúc Test Project

```
test/
├── CMakeLists.txt
├── include/
│   └── test_motor/
│       ├── motor_controller.hpp       # Điều khiển động cơ chính
│       ├── motor_monitor.hpp          # Theo dõi trạng thái
│       ├── motor_config.hpp           # Cấu hình từ EDS
│       └── motor_dashboard.hpp        # Hiển thị dashboard
├── config/
│   ├── motor_config.yaml              # Cấu hình chung
│   └── eds/
│       ├── elmo_gold_whistle.eds     # Elmo Gold Whistle servo
│       ├── maxon_epos2.eds            # Maxon EPOS2
│       ├── servo_drive_generic.eds    # Generic CANopen servo
│       └── drive_default.eds           # Default drive config
├── src/
│   ├── motor_controller.cpp
│   ├── motor_monitor.cpp
│   ├── motor_config.cpp
│   └── main_test.cpp                  # Test entry point
└── README_TEST.md
```

---

### 8.2 Motor Controller - Include

```cpp
// test/include/test_motor/motor_controller.hpp
#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <canopen/device/cia402_motor_drive.hpp>
#include <canopen/device/timeout_safety_controller.hpp>
#include <canopen/co/object_dictionary/standard_objects.hpp>
#include <memory>
#include <vector>
#include <chrono>

namespace canopen {
namespace test {

// Motor configuration from EDS
struct MotorConfig {
    uint8_t node_id;
    std::string name;
    std::string eds_file;

    // Hardware limits
    int32_t max_position;           // 0x607D sub 2
    int32_t min_position;          // 0x607D sub 1
    uint32_t max_motor_speed;      // 0x6080
    uint16_t max_torque;           // 0x6081

    // Profile settings
    uint32_t profile_velocity;      // 0x6081
    uint32_t profile_acceleration;  // 0x6083
    uint32_t profile_deceleration;  // 0x6084
    uint32_t quick_stop_decel;      // 0x6085

    // PDO settings
    bool use_pdo_position;          // Enable position PDO
    bool use_pdo_velocity;          // Enable velocity PDO
    uint16_t pdo_cycle_time_ms;    // PDO update rate

    // Timeout settings
    uint32_t command_timeout_ms;    // Safety timeout
};

// Control modes available
enum class ControlMode {
    POSITION_ABSOLUTE,     // Profiled Position - Absolute
    POSITION_RELATIVE,     // Profiled Position - Relative
    VELOCITY,              // Profiled Velocity
    TORQUE,                // Torque mode
    HOMING,                // Homing mode
    CYCLIC_SYNC_POSITION,  // CSP mode (CiA 402)
    CYCLIC_SYNC_VELOCITY,  // CSV mode
    CYCLIC_SYNC_TORQUE,    // CST mode
};

// Motion profile parameters
struct MotionProfile {
    // Velocity settings
    int32_t target_velocity;        // Target velocity (units/s)
    uint32_t max_velocity;          // Maximum velocity limit

    // Acceleration/Deceleration
    uint32_t acceleration;           // Acceleration (units/s²)
    uint32_t deceleration;           // Deceleration (units/s²)
    uint32_t quick_stop_decel;      // Quick stop deceleration

    // Position-specific
    int32_t target_position;        // Target position (units)
    bool absolute;                   // True = absolute, False = relative

    // Torque
    int16_t target_torque;          // Target torque (mNm or %)

    // Homing
    int16_t homing_method;          // Homing method (CiA 402 table)
    int32_t homing_speed_switch;     // Speed during search for switch
    int32_t homing_speed_zero;       // Speed during search for zero
    uint32_t homing_acceleration;    // Homing acceleration
};

// Motion status
struct MotionStatus {
    // Current state
    CiA402State drive_state;
    OperationMode mode;
    bool is_moving;
    bool is_enabled;
    bool has_fault;

    // Position feedback
    int32_t actual_position;         // Current position
    int32_t position_demand;        // Position demand value
    int32_t position_error;         // Following error

    // Velocity feedback
    int32_t actual_velocity;         // Current velocity
    int32_t velocity_demand;        // Velocity demand value

    // Torque feedback
    int16_t actual_torque;          // Current torque
    int16_t torque_demand;          // Torque demand value

    // Status bits
    bool target_reached;
    bool setpoint_acknowledge;
    bool speed_zero;
    bool torque_max;
    bool following_error;

    // Timing
    uint64_t last_update_ms;
    uint32_t time_since_command_ms;
};

// Main Motor Controller
class MotorController {
public:
    MotorController();
    ~MotorController();

    // Initialization
    void initialize(const std::string& can_interface);
    void add_motor(const MotorConfig& config);
    void load_from_eds(const std::string& eds_file, uint8_t node_id);
    void load_from_json(const std::string& json_file);

    // Startup sequence
    void boot_all_motors();
    void boot_motor(uint8_t node_id);
    void enable_all_motors();
    void enable_motor(uint8_t node_id);

    // Shutdown
    void disable_all_motors();
    void disable_motor(uint8_t node_id);
    void shutdown_all();

    // Motion Control
    void set_control_mode(uint8_t node_id, ControlMode mode);

    // Position control
    void move_to_position(uint8_t node_id, int32_t position,
                         bool absolute = true,
                         const MotionProfile* profile = nullptr);
    void move_relative(uint8_t node_id, int32_t offset,
                       const MotionProfile* profile = nullptr);
    void stop_motion(uint8_t node_id);
    void emergency_stop(uint8_t node_id);

    // Velocity control
    void set_velocity(uint8_t node_id, int32_t velocity);
    void set_velocity_with_profile(uint8_t node_id, int32_t velocity,
                                   uint32_t acceleration, uint32_t deceleration);
    void halt_velocity(uint8_t node_id);

    // Torque control
    void set_torque(uint8_t node_id, int16_t torque);
    void set_torque_limit(uint8_t node_id, uint16_t limit_percent);

    // Homing
    void home_motor(uint8_t node_id, int16_t method = -1);
    void home_all_motors(int16_t method = -1);
    bool is_homed(uint8_t node_id) const;

    // Motion profile customization
    void set_motion_profile(uint8_t node_id, const MotionProfile& profile);
    void set_velocity_limit(uint8_t node_id, uint32_t max_velocity);
    void set_acceleration(uint8_t node_id, uint32_t acceleration);
    void set_deceleration(uint8_t node_id, uint32_t deceleration);

    // Position limits
    void set_position_limits(uint8_t node_id, int32_t min_pos, int32_t max_pos);
    void enable_software_limits(uint8_t node_id, bool enable);

    // Monitoring
    MotionStatus get_motor_status(uint8_t node_id) const;
    std::map<uint8_t, MotionStatus> get_all_motor_status() const;
    bool are_all_motors_ready() const;
    bool has_any_fault() const;
    std::vector<uint8_t> get_faulted_motors() const;

    // Fault handling
    void clear_faults(uint8_t node_id);
    void clear_all_faults();
    void reset_motor(uint8_t node_id);

    // Real-time update (call in loop)
    void update();  // Update all motor status

    // Direct SDO access
    template<typename T>
    int read_sdo(uint8_t node_id, uint16_t index, uint8_t subindex, T& value);

    template<typename T>
    int write_sdo(uint8_t node_id, uint16_t index, uint8_t subindex, const T& value);

private:
    void update_motor_status(uint8_t node_id);
    void apply_motion_profile(uint8_t node_id, const MotionProfile& profile);

    std::map<uint8_t, std::unique_ptr<CiA402MotorDrive>> motors_;
    std::map<uint8_t, MotorConfig> motor_configs_;
    std::map<uint8_t, MotionStatus> motor_statuses_;
    std::map<uint8_t, MotionProfile> motion_profiles_;

    std::shared_ptr<SocketCAN> can_bus_;
    std::unique_ptr<CANopenMaster> master_;
    TimeoutSafetyController& safety_;

    std::thread update_thread_;
    std::atomic<bool> running_{false};
};

} // namespace test
} // namespace canopen

#endif // MOTOR_CONTROLLER_HPP
```

---

### 8.3 Motor Monitor - Theo dõi trạng thái

```cpp
// test/include/test_motor/motor_monitor.hpp
#ifndef MOTOR_MONITOR_HPP
#define MOTOR_MONITOR_HPP

#include "motor_controller.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

namespace canopen {
namespace test {

class MotorDashboard {
public:
    MotorDashboard(MotorController& controller)
        : controller_(controller) {}

    // Print single motor status
    void print_motor_status(uint8_t node_id) {
        auto status = controller_.get_motor_status(node_id);

        std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║  MOTOR " << static_cast<int>(node_id) << " STATUS                                      ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════╣\n";

        // State
        std::cout << "║ State: " << std::left << std::setw(20)
                  << get_state_string(status.drive_state)
                  << " Mode: " << std::setw(15)
                  << get_mode_string(status.mode) << "       ║\n";

        // Position
        std::cout << "╠──────────────────────────────────────────────────────────────╣\n";
        std::cout << "║ POSITION               ║ VELOCITY                ║\n";
        std::cout << "║  Target: " << std::setw(12) << status.position_demand
                  << "  ║  Target: " << std::setw(12) << status.velocity_demand << "  ║\n";
        std::cout << "║  Actual: " << std::setw(12) << status.actual_position
                  << "  ║  Actual: " << std::setw(12) << status.actual_velocity << "  ║\n";
        std::cout << "║  Error:  " << std::setw(12) << status.position_error
                  << "  ║                        ║\n";

        // Torque
        std::cout << "╠──────────────────────────────────────────────────────────────╣\n";
        std::cout << "║ TORQUE                                                     ║\n";
        std::cout << "║  Target: " << std::setw(6) << status.target_torque << " %"
                  << "  Actual: " << std::setw(6) << status.actual_torque << " %"
                  << "              ║\n";

        // Status flags
        std::cout << "╠──────────────────────────────────────────────────────────────╣\n";
        std::cout << "║ FLAGS:                                                     ║\n";
        std::cout << "║  ";
        std::cout << (status.is_enabled ? "[ENABLED] " : "[DISABLED] ");
        std::cout << (status.is_moving ? "[MOVING] " : "[STOPPED] ");
        std::cout << (status.target_reached ? "[AT_TARGET] " : "[IN_MOTION] ");
        std::cout << (status.has_fault ? "[FAULT!] " : "[OK] ") << "        ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    }

    // Print all motors
    void print_all_status() {
        std::cout << "\033[2J\033[H";  // Clear screen
        std::cout << "═══════════════════════════════════════════════════════════════\n";
        std::cout << "           CANOPEN MOTOR CONTROL MONITOR                     \n";
        std::cout << "═══════════════════════════════════════════════════════════════\n";

        auto all_status = controller_.get_all_motor_status();
        for (const auto& [node_id, status] : all_status) {
            print_motor_status(node_id);
        }

        std::cout << "\n───────────────────────────────────────────────────────────────\n";
        std::cout << "Commands: pos <id> <pos> | vel <id> <vel> | stop <id> | q=quit\n";
        std::cout << "───────────────────────────────────────────────────────────────\n";
    }

    // ASCII position plotter
    void plot_position(uint8_t node_id) {
        auto status = controller_.get_motor_status(node_id);

        const int width = 60;
        const int height = 10;

        // Simple bar representation
        int32_t min_pos = -10000;
        int32_t max_pos = 10000;
        double normalized = (double)(status.actual_position - min_pos) /
                          (max_pos - min_pos);
        normalized = std::max(0.0, std::min(1.0, normalized));

        int bar_pos = static_cast<int>(normalized * width);

        std::cout << "Position: " << std::setw(10) << status.actual_position << " [";
        for (int i = 0; i < width; ++i) {
            if (i == bar_pos) std::cout << "*";
            else std::cout << "─";
        }
        std::cout << "] " << std::setw(10) << status.target_position << "\n";

        // Velocity bar
        int32_t max_vel = 5000;
        double vel_normalized = (double)std::abs(status.actual_velocity) / max_vel;
        vel_normalized = std::max(0.0, std::min(1.0, vel_normalized));
        int vel_bar = static_cast<int>(vel_normalized * width);

        std::cout << "Velocity: " << std::setw(10) << status.actual_velocity << " [";
        for (int i = 0; i < width; ++i) {
            if (i == vel_bar) std::cout << "|";
            else std::cout << " ";
        }
        std::cout << "]\n";
    }

private:
    MotorController& controller_;

    std::string get_state_string(CiA402State state) {
        switch (state) {
            case CiA402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY";
            case CiA402State::SWITCH_ON_DISABLED: return "DISABLED";
            case CiA402State::READY_TO_SWITCH_ON: return "READY";
            case CiA402State::SWITCHED_ON: return "SWITCHED_ON";
            case CiA402State::OPERATION_ENABLED: return "ENABLED";
            case CiA402State::QUICK_STOP_ACTIVE: return "QUICK_STOP";
            case CiA402State::FAULT_REACTION_ACTIVE: return "FAULT_RXN";
            case CiA402State::FAULT: return "FAULT";
            default: return "UNKNOWN";
        }
    }

    std::string get_mode_string(OperationMode mode) {
        switch (mode) {
            case OperationMode::PROFILED_POSITION: return "PP";
            case OperationMode::VELOCITY: return "VL";
            case OperationMode::PROFILED_VELOCITY: return "PV";
            case OperationMode::TORQUE: return "TQ";
            case OperationMode::HOMING: return "HM";
            case OperationMode::CYCLIC_SYNC_POSITION: return "CSP";
            case OperationMode::CYCLIC_SYNC_VELOCITY: return "CSV";
            case OperationMode::CYCLIC_SYNC_TORQUE: return "CST";
            default: return "NONE";
        }
    }
};

// Real-time plotter for oscilloscope-like view
class RealTimePlotter {
public:
    struct DataPoint {
        double time;
        double position;
        double velocity;
        double torque;
    };

    void add_point(uint8_t node_id, const DataPoint& point) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_[node_id].push_back(point);
        if (data_[node_id].size() > max_points_) {
            data_[node_id].erase(data_[node_id].begin());
        }
    }

    void print_plot(uint8_t node_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& points = data_[node_id];
        if (points.empty()) return;

        std::cout << "\nPosition | Velocity | Torque\n";
        std::cout << "─────────┼──────────┼────────\n";

        for (const auto& p : points) {
            int pos_bar = (int)((p.position + 10000) / 200 * 30);
            int vel_bar = (int)(std::abs(p.velocity) / 5000 * 30);
            int tor_bar = (int)(std::abs(p.torque) / 100 * 30);

            std::cout << std::string(pos_bar, '#')
                      << " " << std::string(vel_bar, '*')
                      << " " << std::string(tor_bar, '+')
                      << "\n";
        }
    }

private:
    std::map<uint8_t, std::vector<DataPoint>> data_;
    std::mutex mutex_;
    const size_t max_points_ = 50;
};

}
}
#endif // MOTOR_MONITOR_HPP
```

---

### 8.4 EDS File Generator - Cấu hình động cơ

```python
# test/config/eds/generate_eds.py
"""
EDS File Generator for CANopen Servo Drives
Generates Electronic Data Sheet (EDS) files for standard servo drives
"""

EDS_TEMPLATE = '''[FileInfo]
FileName={filename}
FileVersion=4.0
FileRevision=2
CreatedBy=MotorTestSuite
CreationDate=09-14-2026
CreationTime=12:00:00
ModifiedBy=
ModifiedDate=
ModifiedTime=

[DeviceInfo]
DeviceName={device_name}
DeviceType=0x201
VendorName={vendor_name}
VendorNumber={vendor_number}
ProductName={product_name}
ProductNumber={product_number}
RevisionNumber={revision_number}
OrderCode={order_code}

[Comments]
Lines=0

[DeviceComissioning]
NodeID={node_id}
Baudrate=500
BaudrateValid=0x0F

[MandatoryObjects]
0x1000=0x0007,2,"Device Type",0x{device_type:08X}
0x1001=0x07,1,"Error Register"
0x1005=0x{heartbeat_cobid:06X},4,"SYNC COB-ID"
0x1006=0x{heartbeat_cycle:08X},4,"Communication cycle period"
0x1008=0,"Manufacturer device name"
0x1009=0,"Manufacturer hardware version"
0x100A=0,"Manufacturer software version"
0x1010=0x01,4,"Store parameters"
0x1011=0x01,4,"Restore default parameters"
0x1017=0x{heartbeat_producer:04X},2,"Heartbeat producer"

[OptionalObjects]
# RxPDO1 - Controlword
0x1400=0x00000201,6,"RxPDO1 Communication Parameter"
0x1401=0x01,2,"RxPDO1 Transmission Type"
0x1600=0x02,8,"RxPDO1 Mapping"
0x1600sub1=0x60400010,"Controlword"
0x1600sub2=0x607A0020,"Target Position"

# TxPDO1 - Statusword
0x1800=0x00000181,6,"TxPDO1 Communication Parameter"
0x1801=0x01,2,"TxPDO1 Transmission Type"
0x1A00=0x02,8,"TxPDO1 Mapping"
0x1A00sub1=0x60410010,"Statusword"
0x1A00sub2=0x60640020,"Position Actual Value"

# CiA 402 Objects
0x6040=0x0010,2,"Controlword"
0x6041=0x0010,2,"Statusword"
0x6060=0x0008,1,"Modes of operation"
0x6061=0x0008,1,"Modes of operation display"
0x6062=0x0020,4,"Position demand value"
0x6064=0x0020,4,"Position actual value"
0x606C=0x0024,4,"Velocity actual value"
0x607A=0x0024,4,"Target position"
0x607D=0x02,8,"Software position limit"
0x6081=0x0024,4,"Profile velocity"
0x6083=0x0024,4,"Profile acceleration"
0x6084=0x0024,4,"Profile deceleration"
0x6085=0x0024,4,"Quick stop deceleration"
0x6086=0x0024,4,"Motion profile type"
0x6098=0x0008,1,"Homing method"
0x6099=0x02,4,"Homing speeds"
0x609A=0x0024,4,"Homing acceleration"
0x60C5=0x0024,4,"Max deceleration"
0x60FF=0x0024,4,"Target velocity"

[ManufacturerObjects]
# Custom manufacturer objects here
'''

def generate_eds(
    filename="servo_drive.eds",
    device_name="CANopen Servo Drive",
    vendor_name="Test Vendor",
    vendor_number=0x00000001,
    product_name="Test Servo",
    product_number=0x00000001,
    revision_number=0x00010001,
    node_id=0x01,
    heartbeat_cobid=0x701,  # Heartbeat from node
    heartbeat_producer=0,   # 0 = disabled
    heartbeat_cycle=10000   # 10000 us = 10ms
):
    """Generate an EDS file for CANopen servo drive testing."""

    content = EDS_TEMPLATE.format(
        filename=filename,
        device_name=device_name,
        vendor_name=vendor_name,
        vendor_number=vendor_number,
        product_name=product_name,
        product_number=product_number,
        revision_number=revision_number,
        order_code="TEST-001",
        node_id=node_id,
        device_type=0x0192,  # Drive with integrated safety
        heartbeat_cobid=heartbeat_cobid,
        heartbeat_producer=heartbeat_producer,
        heartbeat_cycle=heartbeat_cycle
    )

    return content

if __name__ == "__main__":
    # Generate default drive EDS
    eds = generate_eds(
        filename="servo_drive_generic.eds",
        device_name="Generic CANopen Servo Drive"
    )

    with open("servo_drive_generic.eds", "w") as f:
        f.write(eds)

    # Generate Elmo Gold Whistle EDS
    elmo_eds = generate_eds(
        filename="elmo_gold_whistle.eds",
        device_name="Elmo Gold Whistle",
        vendor_name="Elmo Motion Control",
        vendor_number=0x00000039,
        product_name="Gold Whistle",
        product_number=0x00391001,
        heartbeat_cobid=0x739
    )

    with open("elmo_gold_whistle.eds", "w") as f:
        f.write(elmo_eds)

    # Generate Maxon EPOS2 EDS
    epos_eds = generate_eds(
        filename="maxon_epos2.eds",
        device_name="EPOS2",
        vendor_name="Maxon Motor",
        vendor_number=0x00000001,
        product_name="EPOS2 50/5",
        product_number=0x60301001,
        heartbeat_cobid=0x701
    )

    with open("maxon_epos2.eds", "w") as f:
        f.write(epos_eds)

    print("EDS files generated successfully!")
```

---

### 8.5 Main Test Program

```cpp
// test/src/main_test.cpp
#include "test_motor/motor_controller.hpp"
#include "test_motor/motor_monitor.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <readline/readline.h>
#include <readline/history.h>

using namespace canopen::test;
using namespace std::chrono_literals;

// Global controller
std::unique_ptr<MotorController> g_controller;
std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    std::cout << "\nShutting down...\n";
    g_running = false;
    if (g_controller) {
        g_controller->emergency_stop(0);  // Emergency stop all
        g_controller->disable_all_motors();
    }
    exit(0);
}

// Test sequence: Position control with custom profile
void test_position_control(MotorController& ctrl) {
    std::cout << "\n=== TEST: Position Control ===\n";

    MotionProfile profile{};
    profile.target_position = 10000;
    profile.absolute = true;
    profile.max_velocity = 5000;      // 5000 units/s
    profile.acceleration = 1000;      // 1000 units/s²
    profile.deceleration = 1000;      // 1000 units/s²

    std::cout << "Moving to position 10000 with:\n";
    std::cout << "  - Max velocity: 5000 units/s\n";
    std::cout << "  - Acceleration: 1000 units/s²\n";
    std::cout << "  - Deceleration: 1000 units/s²\n";

    ctrl.move_to_position(1, 10000, true, &profile);

    // Wait for target reached
    auto status = ctrl.get_motor_status(1);
    while (!status.target_reached && g_running) {
        ctrl.update();
        status = ctrl.get_motor_status(1);
        std::cout << "\rPosition: " << status.actual_position
                  << " | Velocity: " << status.actual_velocity
                  << " | State: " << (status.is_moving ? "MOVING" : "STOPPED")
                  << "     " << std::flush;
        std::chrono::milliseconds(50).sleep();
    }
    std::cout << "\nTarget reached!\n";
}

// Test sequence: Velocity control with custom profile
void test_velocity_control(MotorController& ctrl) {
    std::cout << "\n=== TEST: Velocity Control ===\n";

    MotionProfile profile{};
    profile.target_velocity = 2000;    // 2000 units/s
    profile.acceleration = 500;       // 500 units/s²
    profile.deceleration = 500;       // 500 units/s²

    std::cout << "Running at velocity 2000 with:\n";
    std::cout << "  - Acceleration: 500 units/s²\n";
    std::cout << "  - Deceleration: 500 units/s²\n";

    ctrl.set_velocity_with_profile(1, 2000, 500, 500);

    std::this_thread::sleep_for(3s);

    std::cout << "Stopping...\n";
    ctrl.halt_velocity(1);
}

// Test sequence: Homing
void test_homing(MotorController& ctrl) {
    std::cout << "\n=== TEST: Homing ===\n";
    std::cout << "Homing motor 1 using method 35 (Limit switch + Index)...\n";

    MotionProfile homing{};
    homing.homing_method = 35;
    homing.homing_speed_switch = 500;
    homing.homing_speed_zero = 100;
    homing.homing_acceleration = 500;

    ctrl.set_motion_profile(1, homing);
    ctrl.home_motor(1, 35);

    auto status = ctrl.get_motor_status(1);
    while (status.mode == OperationMode::HOMING && g_running) {
        ctrl.update();
        status = ctrl.get_motor_status(1);
        std::cout << "\rHoming... Position: " << status.actual_position << "     " << std::flush;
        std::this_thread::sleep_for(100ms);
    }

    std::cout << "\nHoming complete! Position: " << status.actual_position << "\n";
}

// Test sequence: Profile customization
void test_custom_profiles(MotorController& ctrl) {
    std::cout << "\n=== TEST: Custom Profiles ===\n";

    // Fast profile
    std::cout << "1. Fast profile (high acceleration)\n";
    MotionProfile fast_profile{};
    fast_profile.max_velocity = 10000;
    fast_profile.acceleration = 5000;
    fast_profile.deceleration = 5000;
    ctrl.set_motion_profile(1, fast_profile);
    ctrl.move_to_position(1, 20000);
    std::this_thread::sleep_for(2s);

    // Slow profile
    std::cout << "2. Slow profile (low acceleration)\n";
    MotionProfile slow_profile{};
    slow_profile.max_velocity = 1000;
    slow_profile.acceleration = 100;
    slow_profile.deceleration = 100;
    ctrl.set_motion_profile(1, slow_profile);
    ctrl.move_to_position(1, 0);
    std::this_thread::sleep_for(5s);

    std::cout << "Profile test complete!\n";
}

// Test sequence: Emergency stop
void test_emergency_stop(MotorController& ctrl) {
    std::cout << "\n=== TEST: Emergency Stop ===\n";
    std::cout << "Starting motion...\n";

    ctrl.set_velocity(1, 5000);
    std::this_thread::sleep_for(1s);

    std::cout << "EMERGENCY STOP!\n";
    ctrl.emergency_stop(1);

    auto status = ctrl.get_motor_status(1);
    std::cout << "Velocity after emergency stop: " << status.actual_velocity << "\n";
}

// Interactive command parser
void interactive_mode(MotorController& ctrl, MotorDashboard& dashboard) {
    std::cout << "\n=== INTERACTIVE MODE ===\n";
    std::cout << "Commands:\n";
    std::cout << "  pos <id> <position> [vel] [acc] [dec]  - Move to position\n";
    std::cout << "  vel <id> <velocity> [acc] [dec]         - Set velocity\n";
    std::cout << "  home <id> [method]                     - Home motor\n";
    std::cout << "  stop <id>                              - Stop motion\n";
    std::cout << "  estop <id>                             - Emergency stop\n";
    std::cout << "  status                                 - Show all status\n";
    std::cout << "  monitor                                - Real-time monitor\n";
    std::cout << "  fault <id>                             - Clear faults\n";
    std::cout << "  quit                                   - Exit\n";
    std::cout << "\n";

    char* line;
    while (g_running && (line = readline("CANopen> ")) != nullptr) {
        std::string cmd(line);
        free(line);

        if (cmd.empty()) continue;
        add_history(cmd.c_str());

        std::istringstream iss(cmd);
        std::string action;
        iss >> action;

        try {
            if (action == "pos") {
                int id, pos;
                iss >> id >> pos;
                MotionProfile profile{};
                if (iss.good()) iss >> profile.max_velocity;
                else profile.max_velocity = 5000;
                if (iss.good()) iss >> profile.acceleration;
                else profile.acceleration = 1000;
                if (iss.good()) iss >> profile.deceleration;
                else profile.deceleration = 1000;
                ctrl.move_to_position(id, pos, true, &profile);
                std::cout << "Moving motor " << id << " to " << pos << "\n";
            }
            else if (action == "vel") {
                int id, vel;
                iss >> id >> vel;
                uint32_t acc = 1000, dec = 1000;
                if (iss.good()) iss >> acc;
                if (iss.good()) iss >> dec;
                ctrl.set_velocity_with_profile(id, vel, acc, dec);
                std::cout << "Setting motor " << id << " velocity to " << vel << "\n";
            }
            else if (action == "home") {
                int id;
                iss >> id;
                int method = -1;
                if (iss.good()) iss >> method;
                ctrl.home_motor(id, method);
                std::cout << "Homing motor " << id << "\n";
            }
            else if (action == "stop") {
                int id;
                iss >> id;
                ctrl.stop_motion(id);
                std::cout << "Stopping motor " << id << "\n";
            }
            else if (action == "estop") {
                int id;
                iss >> id;
                ctrl.emergency_stop(id);
                std::cout << "Emergency stop motor " << id << "\n";
            }
            else if (action == "status") {
                dashboard.print_all_status();
            }
            else if (action == "monitor") {
                while (g_running) {
                    dashboard.print_all_status();
                    ctrl.update();
                    std::this_thread::sleep_for(100ms);
                }
            }
            else if (action == "fault") {
                int id;
                iss >> id;
                ctrl.clear_faults(id);
                std::cout << "Clearing faults on motor " << id << "\n";
            }
            else if (action == "quit" || action == "exit") {
                break;
            }
            else {
                std::cout << "Unknown command: " << action << "\n";
            }
        }
        catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << "\n";
        }
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::cout << "═══════════════════════════════════════════════════════════════\n";
    std::cout << "         CANOPEN MOTOR CONTROL TEST SUITE                    \n";
    std::cout << "═══════════════════════════════════════════════════════════════\n";

    // Parse arguments
    std::string can_interface = (argc > 1) ? argv[1] : "can0";

    // Create controller
    g_controller = std::make_unique<MotorController>();

    std::cout << "\nInitializing CAN interface: " << can_interface << "\n";
    g_controller->initialize(can_interface);

    // Add motor configuration
    MotorConfig config{};
    config.node_id = 1;
    config.name = "Test Motor 1";
    config.eds_file = "config/eds/servo_drive_generic.eds";

    // Profile settings
    config.profile_velocity = 5000;
    config.profile_acceleration = 1000;
    config.profile_deceleration = 1000;
    config.quick_stop_decel = 5000;

    // Limits
    config.max_motor_speed = 10000;
    config.max_position = 50000;
    config.min_position = -50000;
    config.max_torque = 1000;  // mNm

    // Safety
    config.command_timeout_ms = 500;

    g_controller->add_motor(config);

    std::cout << "Motor configured:\n";
    std::cout << "  - Node ID: " << (int)config.node_id << "\n";
    std::cout << "  - Max Velocity: " << config.profile_velocity << "\n";
    std::cout << "  - Acceleration: " << config.profile_acceleration << "\n";
    std::cout << "  - Timeout: " << config.command_timeout_ms << " ms\n";

    // Boot motors
    std::cout << "\nBooting motors...\n";
    g_controller->boot_all_motors();
    std::this_thread::sleep_for(1s);

    std::cout << "Enabling motors...\n";
    g_controller->enable_all_motors();
    std::this_thread::sleep_for(500ms);

    // Create dashboard
    MotorDashboard dashboard(*g_controller);

    // Run automated tests
    if (argc > 2 && std::string(argv[2]) == "--test") {
        std::cout << "\nRunning automated test sequence...\n";
        test_position_control(*g_controller);
        test_velocity_control(*g_controller);
        test_homing(*g_controller);
        test_custom_profiles(*g_controller);
        test_emergency_stop(*g_controller);
    }

    // Interactive mode
    interactive_mode(*g_controller, dashboard);

    // Cleanup
    std::cout << "\nShutting down motors...\n";
    g_controller->disable_all_motors();
    std::this_thread::sleep_for(500ms);

    std::cout << "Test complete!\n";
    return 0;
}
```

---

### 8.6 CMakeLists.txt cho Test Project

```cmake
# test/CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(canopen_motor_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find CANopen library
find_path(CANOPEN_INCLUDE_DIR canopen/device/canopen_master.hpp
    HINTS ${CMAKE_SOURCE_DIR}/../include
)

find_library(CANOPEN_LIBRARY NAMES canopen
    HINTS ${CMAKE_SOURCE_DIR}/../lib
)

if(NOT CANOPEN_LIBRARY)
    message(WARNING "CANopen library not found, building tests only")
    set(BUILD_CANOPEN OFF)
else()
    set(BUILD_CANOPEN ON)
endif()

# Dependencies
find_package(Threads REQUIRED)
find_package(PkgConfig)

# Optional: Readline for interactive mode
find_package(Readline QUIET)

# Source files
set(SOURCES
    src/motor_controller.cpp
    src/motor_monitor.cpp
    src/motor_config.cpp
    src/main_test.cpp
)

# Include directories
set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CANOPEN_INCLUDE_DIR}
)

# Create executable
add_executable(canopen_motor_test ${SOURCES})
target_include_directories(canopen_motor_test PRIVATE ${INCLUDES})

# Libraries
target_link_libraries(canopen_motor_test
    Threads::Threads
    $<$<BOOL:${BUILD_CANOPEN}>:${CANOPEN_LIBRARY}>
)

# Readline
if(Readline_FOUND)
    target_link_libraries(canopen_motor_test Readline::Readline)
    target_compile_definitions(canopen_motor_test HAVE_READLINE=1)
else()
    target_compile_definitions(canopen_motor_test HAVE_READLINE=0)
    message(STATUS "Readline not found, using basic input")
endif()

# Copy EDS files to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/eds/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/config/eds)

# Copy config files
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/motor_config.yaml
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/config/)

# Install
install(TARGETS canopen_motor_test DESTINATION bin)
install(DIRECTORY config/ DESTINATION etc/canopen_motor)
```

---

### 8.7 YAML Configuration File

```yaml
# test/config/motor_config.yaml
can:
  interface: can0
  bitrate: 500000

motors:
  - node_id: 1
    name: X-Axis
    eds_file: config/eds/servo_drive_generic.eds
            "type": "CiA402",
            "profile": {
                "velocity": 5000,
                "acceleration": 1000,
                "deceleration": 1000,
                "quick_stop_deceleration": 5000
            },
            "limits": {
                "max_velocity": 10000,
                "max_position": 50000,
                "min_position": -50000,
                "max_torque_percent": 100
            },
            "safety": {
                "command_timeout_ms": 500,
                "enable_timeout": true,
                "stop_mode": "quick_stop"
            },
            "homing": {
                "method": 35,
                "switch_speed": 500,
                "zero_speed": 100,
                "acceleration": 500
            },
            "pdo": {
                "enable_rx_pdo1": true,
                "enable_tx_pdo1": true,
                "cycle_time_ms": 10
            }
        },
        {
            "node_id": 2,
            "name": "Y-Axis",
            "eds_file": "config/eds/servo_drive_generic.eds",
            "type": "CiA402",
            "profile": {
                "velocity": 3000,
                "acceleration": 800,
                "deceleration": 800,
                "quick_stop_deceleration": 3000
            },
            "limits": {
                "max_velocity": 8000,
                "max_position": 30000,
                "min_position": 0,
                "max_torque_percent": 80
            },
            "safety": {
                "command_timeout_ms": 500,
                "enable_timeout": true,
                "stop_mode": "quick_stop"
            }
        },
        {
            "node_id": 3,
            "name": "Z-Axis",
            "eds_file": "config/eds/servo_drive_generic.eds",
            "type": "CiA402",
            "profile": {
                "velocity": 2000,
                "acceleration": 500,
                "deceleration": 500,
                "quick_stop_deceleration": 2000
            },
            "limits": {
                "max_velocity": 5000,
                "max_position": 20000,
                "min_position": 0,
                "max_torque_percent": 50
            },
            "safety": {
                "command_timeout_ms": 500,
                "enable_timeout": true,
                "stop_mode": "quick_stop"
            }
        }
    ],

    "sync": {
        "enable": false,
        "cycle_time_us": 10000,
        "cobid": 0x80
    },

    "recovery": {
        "auto_recovery": true,
        "bus_off_delay_ms": 1000,
        "max_attempts": 5
    },

    "logging": {
        "enable": true,
        "level": "info",
        "file": "/var/log/canopen_motor.log"
    }
}
```

---

### 8.8 Build và Run

```bash
# Build instructions
cd test
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run basic test
./canopen_motor_test can0 --test

# Run interactive mode
./canopen_motor_test can0

# Example commands in interactive mode:
# pos 1 10000 5000 1000 1000    # Move motor 1 to position 10000
# vel 1 2000 500 500             # Set motor 1 velocity to 2000
# home 1 35                       # Home motor 1 with method 35
# status                          # Show all motor status
# monitor                         # Real-time monitoring
# estop 1                         # Emergency stop motor 1
```

---

### 8.9 Demo Script - Tự động hóa các test

```bash
#!/bin/bash
# test/run_demo.sh - Automated motor control demo

CAN_INTERFACE="${1:-can0}"

echo "═══════════════════════════════════════════════════════════════"
echo "         CANOPEN MOTOR CONTROL DEMO"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Check if can interface exists
if ! ip link show $CAN_INTERFACE > /dev/null 2>&1; then
    echo "ERROR: CAN interface $CAN_INTERFACE not found"
    echo "Please setup CAN interface first:"
    echo "  sudo ip link add $CAN_INTERFACE type can"
    echo "  sudo ip link set $CAN_INTERFACE up bitrate 500000"
    exit 1
fi

echo "CAN Interface: $CAN_INTERFACE"
echo ""

# Run motor test
./canopen_motor_test $CAN_INTERFACE --test << EOF
pos 1 5000 5000 1000 1000
wait 3
vel 1 2000 500 500
wait 2
stop 1
home 1 35
wait 5
status
quit
EOF

echo ""
echo "Demo complete!"
```

---

### 8.10 Features Matrix

| Feature | Implemented | File | Description |
|---------|------------|------|-------------|
| Position Control | ✅ | motor_controller.hpp | Profiled position mode |
| Velocity Control | ✅ | motor_controller.hpp | Profiled velocity mode |
| Torque Control | ✅ | motor_controller.hpp | Torque mode |
| Homing | ✅ | motor_controller.hpp | Multiple homing methods |
| Custom Acceleration | ✅ | MotionProfile | User-defined accel/decel |
| Custom Velocity | ✅ | MotionProfile | User-defined max velocity |
| Safety Timeout | ✅ | TimeoutSafetyController | Auto-stop on command loss |
| Real-time Monitor | ✅ | motor_monitor.hpp | ASCII dashboard |
| Position Plotter | ✅ | motor_monitor.hpp | Oscilloscope view |
| EDS Loading | ✅ | motor_config.cpp | Load from EDS file |
| YAML Config | ✅ | motor_config.yaml | YAML configuration |
| Interactive CLI | ✅ | main_test.cpp | Command-line interface |
| Emergency Stop | ✅ | motor_controller.hpp | Quick stop all motors |

---

## ⭐ Example: Motor Control với Timeout Safety

```cpp
// examples/motor_control_example.cpp
#include <canopen/device/canopen_master.hpp>
#include <canopen/device/cia402_motor_drive.hpp>
#include <canopen/device/timeout_safety_controller.hpp>

int main() {
    // Setup CAN
    auto can = std::make_shared<SocketCAN>("can0");
    can->open();

    // Create Master
    CANopenMaster master(0x00, can);

    // Create Safety Controller
    auto& safety = TimeoutSafetyController::instance();
    safety.set_default_timeout(500);  // 500ms
    safety.set_stop_mode(QUICK_STOP);

    // Create Motor Drives
    CiA402MotorDrive drive1(0x01, can);
    CiA402MotorDrive drive2(0x02, can);

    // Register drives for safety monitoring
    safety.register_drive(0x01, &drive1);
    safety.register_drive(0x02, &drive2);

    // Initialize
    master.init();
    master.start();

    // Boot slaves
    master.boot_slave(0x01);
    master.boot_slave(0x02);

    // Homing
    drive1.home_and_enable(35);  // Method 35: Limit switch + index
    drive2.home_and_enable(35);

    // Position move example
    while (true) {
        // Send position commands - must be within 500ms
        drive1.set_target_position(10000);  // 10000 encoder counts
        drive2.set_target_velocity(5000);  // 5000 rpm

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // If no command for 500ms, motor will quick stop automatically!
    }

    return 0;
}
```

---

## ⭐ Timeout Safety Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                     TIMEOUT SAFETY FLOW                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Normal Operation:                                               │
│  ┌──────────┐     Command      ┌──────────┐                     │
│  │  App     │ ───────────────►│  Drive   │                     │
│  └──────────┘                 └──────────┘                     │
│         │                            │                          │
│         │    last_command = now     │                          │
│         │◄──────────────────────────┘                          │
│                                                                  │
│  Command Lost (timeout):                                         │
│  ┌──────────┐     [no cmd]     ┌──────────┐                     │
│  │  Safety  │ ───────────────►│  Drive   │                     │
│  │Controller│   after 500ms   └──────────┘                     │
│  └──────────┘                 │                                 │
│         │                     │ QUICK STOP!                     │
│         │                     ▼                                 │
│         │              ┌──────────┐                            │
│         │              │ Safe     │                            │
│         │              │ State    │                            │
│         │              └──────────┘                            │
│         │                                                        │
│  Recovery:                                                       │
│         │◄────────────── Resume command ─────────────────────── │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Implementation Order

| Phase | Task | Priority | CiA Spec | Estimated LOC |
|-------|------|----------|----------|---------------|
| **CORE LIBRARY** |||||
| 1.1 | SocketCAN Interface | **HIGH** | - | 500 |
| 1.2 | Filter Manager | HIGH | - | 300 |
| 1.3 | Bus Monitor | HIGH | - | 400 |
| 2.1 | CAN Frame | HIGH | - | 300 |
| 2.2 | Message Factory | HIGH | - | 500 |
| 3.1 | Object Dictionary | **CRITICAL** | CiA 301 | 800 |
| 3.2 | NMT State Machine | **CRITICAL** | CiA 301 | 600 |
| 3.3 | SDO Protocol | **CRITICAL** | CiA 301 | 800 |
| 3.4 | PDO Protocol | **HIGH** | CiA 301 | 700 |
| 3.5 | LSS | MEDIUM | CiA 305 | 600 |
| 3.6 | EMCY/Heartbeat | **HIGH** | CiA 301 | 500 |
| 3.7 | SYNC Producer/Consumer | **HIGH** | CiA 301 | 400 |
| 3.8 | **CiA 402 Motor Control** | **CRITICAL** | **CiA 402** | 1200 |
| 3.9 | Standard Objects | **HIGH** | CiA 301+402 | 600 |
| 4.1 | Bus State Manager | **CRITICAL** | - | 600 |
| 4.2 | TX Queue | HIGH | - | 400 |
| 4.3 | Node Health Monitor | HIGH | - | 500 |
| 4.4 | Recovery Orchestrator | **CRITICAL** | - | 700 |
| 5.1 | Device Base | HIGH | CiA 301 | 300 |
| 5.2 | CANopen Master | HIGH | CiA 301 | 800 |
| 5.3 | CANopen Slave | MEDIUM | CiA 301 | 600 |
| 5.4 | **CiA 402 Motor Device** | **CRITICAL** | **CiA 402** | 600 |
| 5.5 | **Timeout Safety Controller** | **CRITICAL** | **CiA 402** | 500 |
| 6.1 | Event Loop | MEDIUM | - | 600 |
| 7.1 | ASCII Gateway | LOW | CiA 309 | 400 |
| **TEST & DEMO** |||||
| 8.1 | MotorController | **HIGH** | - | 600 |
| 8.2 | MotorMonitor | **HIGH** | - | 400 |
| 8.3 | MotorDashboard | HIGH | - | 300 |
| 8.4 | EDS File Generator | MEDIUM | - | 200 |
| 8.5 | YAML Config Parser | MEDIUM | - | 300 |
| 8.6 | Main Test Program | HIGH | - | 500 |

---

## Verification

### Build Verification
```bash
# Compile
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Unit tests
ctest --output-on-failure

# Examples
./examples/master_example can0
./examples/slave_example can0 0x10
```

### Recovery Testing
```bash
# 1. Start master
./examples/master_example can0

# 2. In another terminal, force bus-off
ip link set can0 down

# 3. Wait 5 seconds, bring interface back up
ip link set can0 up

# 4. Verify:
#    - Application still running
#    - No restart needed
#    - TX queue restored
#    - Nodes re-joined
```

---

## Dependencies

| Dependency | Purpose | License |
|-----------|---------|---------|
| libsocketcan (optional) | CAN bus control | BSD |
| pthread | Multithreading | POSIX |
| Boost (optional) | Shared pointers | Boost |
| CMake | Build system | BSD |

---

## Success Criteria

### CiA Compliance
✅ **CiA 301** - CANopen application layer (mandatory)
✅ **CiA 402** - Motion control profile (mandatory)
✅ **CiA 305** - LSS (Layer Setting Services) (recommended)

### Protocol Features
✅ PDO (TPDO/RPDO) - Fast cyclic data exchange
✅ SDO (Client/Server) - Parameter configuration
✅ NMT State Machine - Node state management
✅ Heartbeat Consumer/Producer - Connection monitoring
✅ EMCY (Emergency) - Error reporting
✅ SYNC Producer/Consumer - Data synchronization

### Safety Features
✅ **Timeout Safety** - Prevent motor runaway when commands lost ⭐
✅ Follow error monitoring (CiA 402)
✅ Software position limits (CiA 402)
✅ Quick stop function (CiA 402)
✅ Fault reaction handling (CiA 402)

### Driver/Recovery
✅ CAN_RAW deep intervention (filters, error frames, bus state)
✅ CAN_ADMIN operations (bus state control, restart)
✅ Self-recovery from bus-off without restart
✅ State preservation (OD values, TX queue)
✅ Node health monitoring

### Test & Demo
✅ MotorController - Main motor control class
✅ MotorMonitor - Real-time status monitoring
✅ ASCII Dashboard - Terminal-based UI
✅ Position Plotter - Oscilloscope view
✅ EDS File Generator - Standard drive configs
✅ JSON Configuration - Multi-motor setup
✅ Interactive CLI - Command-line interface

### General
✅ Multi-node support
✅ EDS/DCF file support
✅ C++17/20 API
✅ Cross-platform (Linux, embedded)

---

## PHẦN 9: YAML CẤU HÌNH & CẤU TRÚC BUILD

### 9.1 YAML Configuration Files (motor_config.yaml)

```yaml
# config/motor_config.yaml - Cấu hình chính cho hệ thống CANopen

# CAN Interface
can:
  interface: "can0"           # Tên interface CAN
  bitrate: 500000             # Bitrate (bps)
  loopback: false             # Loopback mode
  fd_enabled: false           # CAN FD enable
  restart_ms: 1000            # Auto restart delay

# Recovery Settings
recovery:
  enabled: true
  auto_recover: true
  bus_off_delay_ms: 1000
  max_attempts: 5
  preserve_tx_queue: true
  use_exponential_backoff: true
  initial_backoff_ms: 1000
  max_backoff_ms: 30000

# Safety Settings
safety:
  enabled: true
  default_timeout_ms: 500
  stop_mode: "quick_stop"      # stop_motor, quick_stop, hold_position
  recovery_mode: "auto"       # auto, manual

# Motors
motors:
  - node_id: 1
    name: "X-Axis"
    eds_file: "config/eds/servo_drive_generic.eds"
    profile:
      velocity: 5000
      acceleration: 1000
      deceleration: 1000
      quick_stop: 5000
    limits:
      max_velocity: 10000
      max_position: 50000
      min_position: -50000
      max_torque_percent: 100
    pdo:
      enable_rx_pdo1: true
      enable_tx_pdo1: true
      cycle_time_ms: 10
    safety:
      command_timeout_ms: 500
      enable_timeout: true
    homing:
      method: 35
      switch_speed: 500
      zero_speed: 100
      acceleration: 500

  - node_id: 2
    name: "Y-Axis"
    eds_file: "config/eds/servo_drive_generic.eds"
    profile:
      velocity: 3000
      acceleration: 800
      deceleration: 800
      quick_stop: 3000
    limits:
      max_velocity: 8000
      max_position: 30000
      min_position: 0
      max_torque_percent: 80
    safety:
      command_timeout_ms: 500
      enable_timeout: true
    homing:
      method: 35

  - node_id: 3
    name: "Z-Axis"
    eds_file: "config/eds/servo_drive_generic.eds"
    profile:
      velocity: 2000
      acceleration: 500
      deceleration: 500
      quick_stop: 2000
    limits:
      max_velocity: 5000
      max_position: 20000
      min_position: 0
      max_torque_percent: 50
    safety:
      command_timeout_ms: 500
      enable_timeout: true
    homing:
      method: 35

# Logging
logging:
  enabled: true
  level: "info"
  file: "/var/log/canopen_motor.log"
  console: true
```

### 9.2 CMakeLists.txt Templates

```cmake
# CMakeLists.txt - Root build file
cmake_minimum_required(VERSION 3.15)
project(canopen VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_TESTING "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)

find_package(Threads REQUIRED)

add_subdirectory(src)

if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

include(GNUInstallDirs)
install(DIRECTORY include/canopen/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
```

```cmake
# src/CMakeLists.txt
add_library(canopen_lib)

target_include_directories(canopen_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_sources(canopen_lib PRIVATE
    can/raw/socket_can.cpp
    can/frame/frame.cpp
    can/msg/message_factory.cpp
    co/object_dictionary/object.cpp
    co/object_dictionary/object_dictionary.cpp
    co/object_dictionary/standard_objects.cpp
    co/nmt/nmt.cpp
    co/sdo/sdo.cpp
    co/sdo/sdo_client.cpp
    co/sdo/sdo_server.cpp
    co/pdo/pdo.cpp
    co/lss/lss.cpp
    co/emcy/emcy.cpp
    co/heartbeat/heartbeat.cpp
    co/sync/sync.cpp
    co/cia402/cia402_drive.cpp
    recovery/bus_state_manager.cpp
    recovery/tx_queue.cpp
    recovery/node_health_monitor.cpp
    recovery/recovery_orchestrator.cpp
    device/canopen_device.cpp
    device/canopen_master.cpp
    device/canopen_slave.cpp
    device/cia402_motor_drive.cpp
    device/timeout_safety_controller.cpp
    ev/event_loop.cpp
    gw/ascii_gateway.cpp
)

target_link_libraries(canopen_lib PRIVATE Threads::Threads)

install(TARGETS canopen_lib EXPORT canopenTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
```

### 9.3 YAML Config Parser (include/canopen/config/yaml_config.hpp)

```cpp
#ifndef YAML_CONFIG_HPP
#define YAML_CONFIG_HPP

#include <string>
#include <vector>
#include <map>
#include <variant>

namespace canopen {

struct CANConfig {
    std::string interface{"can0"};
    int bitrate{500000};
    bool loopback{false};
    bool fd_enabled{false};
    int restart_ms{1000};
};

struct RecoveryConfig {
    bool enabled{true};
    bool auto_recover{true};
    int bus_off_delay_ms{1000};
    int max_attempts{5};
};

struct SafetyConfig {
    bool enabled{true};
    int default_timeout_ms{500};
    std::string stop_mode{"quick_stop"};
};

struct MotorProfile {
    int velocity{5000};
    int acceleration{1000};
    int deceleration{1000};
    int quick_stop{5000};
};

struct MotorLimits {
    int max_velocity{10000};
    int max_position{50000};
    int min_position{-50000};
    int max_torque_percent{100};
};

struct MotorHoming {
    int method{35};
    int switch_speed{500};
    int zero_speed{100};
    int acceleration{500};
};

struct MotorConfig {
    int node_id{1};
    std::string name{"Motor"};
    std::string eds_file;
    MotorProfile profile;
    MotorLimits limits;
    MotorHoming homing;
    int command_timeout_ms{500};
    bool enable_timeout{true};
};

struct SystemConfig {
    CANConfig can;
    RecoveryConfig recovery;
    SafetyConfig safety;
    std::vector<MotorConfig> motors;
};

class YAMLConfigParser {
public:
    bool load(const std::string& filename);
    SystemConfig get_system_config() const;
    std::string get_error() const { return error_; }

private:
    std::string error_;
    SystemConfig config_;
};

} // namespace canopen
#endif // YAML_CONFIG_HPP
```

### 9.4 Project Directory Structure

```
base-canopen/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── include/canopen/
│   ├── can/raw/          # CAN Driver Layer
│   ├── co/               # CANopen Protocol Layer
│   ├── recovery/         # Recovery Layer
│   ├── device/           # Device Layer
│   ├── ev/               # Event Loop
│   ├── gw/               # Gateway
│   └── config/           # YAML Config Parser
├── src/
│   ├── can/raw/
│   ├── co/
│   ├── recovery/
│   ├── device/
│   ├── ev/
│   ├── gw/
│   └── config/
├── test/
│   ├── include/test_motor/
│   │   ├── motor_controller.hpp
│   │   ├── motor_monitor.hpp
│   │   └── motor_config.hpp
│   ├── config/
│   │   ├── motor_config.yaml     # YAML config ⭐
│   │   └── eds/                  # EDS files
│   └── src/
├── examples/
└── scripts/
    └── generate_eds.py
```

---

## PHẦN 10: BUILD & RUN

### Build Instructions

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

# Build
make -j$(nproc)

# Run tests
ctest --output-on-failure

# Run motor control example
./examples/motor_control_example can0
```

### Implementation Priority

| Priority | Phase | Tasks | Estimated Time |
|----------|-------|-------|----------------|
| **CRITICAL** | Phase 1 | SocketCAN, Filter, Bus Monitor | 1 week |
| **CRITICAL** | Phase 3.1-3.3 | OD, NMT, SDO | 1 week |
| **CRITICAL** | Phase 3.8 | CiA 402 Motor Control | 1 week |
| **CRITICAL** | Phase 4 | Recovery System | 1 week |
| **HIGH** | Phase 5 | Device Layer + Timeout Safety | 1 week |
| **MEDIUM** | Phase 3.4-3.7 | PDO, LSS, EMCY, SYNC | 1 week |
| **HIGH** | Phase 8-9 | Test & Demo + YAML Config | 1 week |
