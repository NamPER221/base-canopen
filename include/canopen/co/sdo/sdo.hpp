/**
 * @file sdo.hpp
 * @brief SDO (Service Data Object) Protocol — expedited transfers
 *
 * SDOClient: master-side, sends requests on 0x600+node, waits for the
 * response on 0x580+node (single outstanding request, condition-variable
 * based with timeout).
 *
 * SDOServer: slave-side, serves requests from its own Object Dictionary
 * and answers on 0x580+node.
 *
 * Note: only expedited (<= 4 byte) transfers are supported; larger
 * objects abort with 0x06070010 (object length mismatch).
 */

#ifndef CANOPEN_CO_SDO_SDO_HPP
#define CANOPEN_CO_SDO_SDO_HPP

#include <canopen/co/object_dictionary/object_dictionary.hpp>
#include <canopen/can/raw/bus_interface.hpp>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>

namespace canopen {

class CANFrame;

/**
 * @brief SDO error codes
 */
enum class SDOError {
    OK = 0,
    TIMEOUT = -1,
    ABORT = -2,
    BUS_ERROR = -3,
    INVALID_RESPONSE = -4,
    NO_BUS = -5,
};

/**
 * @brief Map errno-style result from ObjectDictionary to SDO abort code
 */
uint32_t sdo_abort_from_errno(int result);

/**
 * @brief SDO Server (slave side)
 */
class SDOServer {
public:
    /**
     * @brief Construct SDO Server
     * @param od Object Dictionary to serve
     * @param bus Bus to answer on (may be set later via set_bus/attach)
     */
    explicit SDOServer(ObjectDictionary& od, BusInterface* bus = nullptr);

    ~SDOServer();

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void set_node_id(uint8_t id) { node_id_ = id; }
    uint8_t get_node_id() const { return node_id_; }

    /**
     * @brief Register for automatic dispatch of SDO requests on this bus
     */
    void attach(BusInterface& bus);

    /**
     * @brief Unregister from the bus
     */
    void detach(BusInterface& bus);

    /**
     * @brief Handle an incoming SDO request frame (0x600+node)
     */
    void handle_frame(const CANFrame& frame);

private:
    void handle_download_request(const CANFrame& frame);
    void handle_upload_request(const CANFrame& frame);
    bool respond(const CANFrame& frame);

    ObjectDictionary& od_;
    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};
    uint8_t node_id_{0};
};

/**
 * @brief SDO Client (master side) — single outstanding request
 */
class SDOClient {
public:
    using UploadCallback = std::function<void(SDOError, const void*, size_t)>;
    using DownloadCallback = std::function<void(SDOError)>;

    /**
     * @brief Construct SDO Client
     * @param bus Bus to send on (may be set later via set_bus/attach)
     * @param server_node_id Target server node ID
     */
    explicit SDOClient(BusInterface* bus = nullptr, uint8_t server_node_id = 0);

    ~SDOClient();

    // ==================== Configuration ====================

    void set_bus(BusInterface* bus) { bus_ = bus; }
    void set_server_node_id(uint8_t id) { server_node_id_ = id; }
    void set_timeout(uint32_t ms) { timeout_ms_ = ms; }
    uint32_t get_timeout() const { return timeout_ms_; }
    uint8_t get_server_node_id() const { return server_node_id_; }

    /**
     * @brief Register for automatic dispatch of SDO responses on this bus
     */
    void attach(BusInterface& bus);

    /**
     * @brief Unregister from the bus
     */
    void detach(BusInterface& bus);

    // ==================== Synchronous API ====================

    /**
     * @brief Upload (read) a value — blocks until response/timeout
     * @param size in: buffer capacity, out: actual size read
     */
    SDOError upload_sync(uint16_t index, uint8_t subindex, void* data, size_t& size);

    /**
     * @brief Download (write) a value — blocks until response/timeout
     */
    SDOError download_sync(uint16_t index, uint8_t subindex, const void* data, size_t size);

    // ==================== Template versions ====================

    template<typename T>
    SDOError upload(uint16_t index, uint8_t subindex, T& value) {
        size_t size = sizeof(T);
        SDOError err = upload_sync(index, subindex, &value, size);
        if (err != SDOError::OK) return err;
        if (size != sizeof(T)) return SDOError::INVALID_RESPONSE;
        return SDOError::OK;
    }

    template<typename T>
    SDOError download(uint16_t index, uint8_t subindex, const T& value) {
        return download_sync(index, subindex, &value, sizeof(T));
    }

    // ==================== Asynchronous API ====================

    /**
     * @brief Upload (read) asynchronously (runs upload_sync in a worker thread)
     */
    void upload(uint16_t index, uint8_t subindex, UploadCallback callback);

    /**
     * @brief Download (write) asynchronously (runs download_sync in a worker thread)
     */
    void download(uint16_t index, uint8_t subindex, const void* data,
                  size_t size, DownloadCallback callback);

    // ==================== Control ====================

    /**
     * @brief Send SDO abort to the server / unblock pending request
     */
    void abort(uint32_t abort_code = 0x08000000);

    /**
     * @brief Handle an incoming SDO response frame (0x580+node)
     */
    void handle_frame(const CANFrame& frame);

    /**
     * @brief Last abort code received (valid when result was ABORT)
     */
    uint32_t last_abort_code() const { return abort_code_.load(); }

    /**
     * @brief Verbose trace: in mọi SDO request/response (stderr)
     *        Dùng để debug giao tiếp với drive thật
     */
    void set_verbose(bool on) { verbose_ = on; }

private:
    // Single outstanding request state
    struct PendingState {
        std::mutex mutex;
        std::condition_variable cv;
        bool response_ready{false};
        SDOError result{SDOError::OK};
        uint16_t index{0};
        uint8_t subindex{0};
        std::vector<uint8_t> data;
    };

    void complete_locked(PendingState& p, SDOError result,
                         const uint8_t* data, size_t len);

    BusInterface* bus_{nullptr};
    BusInterface::RouteHandle route_{0};
    uint8_t server_node_id_{0};
    uint32_t timeout_ms_{1000};
    std::atomic<uint32_t> abort_code_{0};
    bool verbose_{false};

    PendingState pending_;
};

} // namespace canopen

#endif // CANOPEN_CO_SDO_SDO_HPP
