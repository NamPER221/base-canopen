/**
 * @file sdo.cpp
 * @brief SDO Protocol implementation (expedited transfers)
 */

#include <canopen/co/sdo/sdo.hpp>
#include <canopen/can/msg/message_factory.hpp>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace canopen {

uint32_t sdo_abort_from_errno(int result) {
    switch (result) {
        case -ENOENT: return static_cast<uint32_t>(SDOAbortCode::OBJECT_NOT_EXIST);
        case -EACCES: return static_cast<uint32_t>(SDOAbortCode::ACCESS_FAILED);
        case -EINVAL: return static_cast<uint32_t>(SDOAbortCode::OBJECT_LENGTH_MISMATCH);
        default:      return 0x08000000;  // general error
    }
}

// ==================== SDO Server ====================

SDOServer::SDOServer(ObjectDictionary& od, BusInterface* bus)
    : od_(od), bus_(bus) {}

SDOServer::~SDOServer() {
    if (bus_ && route_) {
        bus_->remove_route(route_);
    }
}

void SDOServer::attach(BusInterface& bus) {
    bus_ = &bus;
    route_ = routes::rsdo(bus, node_id_, [this](const CANFrame& f) {
        handle_frame(f);
    });
}

void SDOServer::detach(BusInterface& bus) {
    if (route_) bus.remove_route(route_);
    route_ = 0;
    bus_ = nullptr;
}

void SDOServer::handle_frame(const CANFrame& frame) {
    if (frame.len() < 4) return;

    const uint32_t cob_id = frame.id() & 0x7FF;

    // Requests arrive on RSDO: 0x600 + node_id
    if ((cob_id & 0x780) != 0x600) return;

    // If a specific node ID is configured, only serve that node
    if (node_id_ != 0 && (cob_id & 0x7F) != node_id_) return;

    const uint8_t cmd = frame.get_u8(0);
    const uint8_t ccs = cmd & 0xE0;

    switch (ccs) {
        case 0x20:  // initiate download
        case 0x00:  // download segment
            handle_download_request(frame);
            break;
        case 0x40:  // initiate upload
        case 0x60:  // upload segment
            handle_upload_request(frame);
            break;
        case 0x80:  // client abort — nothing to answer
        default:
            break;
    }
}

void SDOServer::handle_download_request(const CANFrame& frame) {
    const uint8_t cmd = frame.get_u8(0);
    const uint16_t index = frame.get_u16_le(1);
    const uint8_t subindex = frame.get_u8(3);
    const uint8_t node = frame.id() & 0x7F;

    const uint8_t* data = nullptr;
    size_t data_size = 0;

    if (cmd & 0x02) {
        // Expedited: e=1. s bit (bit 0) = size indicated in n (bits 2-3)
        if (cmd & 0x01) {
            const uint8_t n = (cmd >> 2) & 0x03;
            data_size = 4 - n;
        } else {
            data_size = 4;  // size unknown, assume full payload
        }
        data = frame.data() + 4;
    } else {
        // Segmented transfer not supported — abort
        respond(MessageFactory::create_sdo_abort(
            node, index, subindex,
            static_cast<uint32_t>(SDOAbortCode::OBJECT_LENGTH_MISMATCH)));
        return;
    }

    const int result = od_.write(index, subindex, data, data_size);

    if (result == 0) {
        // Download initiate response: scs=011, n=0, e=0, s=1 -> 0x60
        respond(MessageFactory::create_sdo_download_response(
            node, index, subindex, nullptr, 0));
    } else {
        respond(MessageFactory::create_sdo_abort(
            node, index, subindex, sdo_abort_from_errno(result)));
    }
}

void SDOServer::handle_upload_request(const CANFrame& frame) {
    const uint16_t index = frame.get_u16_le(1);
    const uint8_t subindex = frame.get_u8(3);
    const uint8_t node = frame.id() & 0x7F;

    uint8_t buffer[4];
    size_t size = sizeof(buffer);

    const int result = od_.read(index, subindex, buffer, size);

    if (result == 0 && size <= 4) {
        respond(MessageFactory::create_sdo_upload_response(
            node, index, subindex, buffer, size));
    } else if (result == 0) {
        // Object too large for expedited transfer
        respond(MessageFactory::create_sdo_abort(
            node, index, subindex,
            static_cast<uint32_t>(SDOAbortCode::OBJECT_LENGTH_MISMATCH)));
    } else {
        respond(MessageFactory::create_sdo_abort(
            node, index, subindex, sdo_abort_from_errno(result)));
    }
}

bool SDOServer::respond(const CANFrame& frame) {
    if (!bus_) return false;
    return bus_->send(frame);
}

// ==================== SDO Client ====================

SDOClient::SDOClient(BusInterface* bus, uint8_t server_node_id)
    : bus_(bus), server_node_id_(server_node_id) {}

SDOClient::~SDOClient() {
    if (bus_ && route_) {
        bus_->remove_route(route_);
    }
}

void SDOClient::attach(BusInterface& bus) {
    bus_ = &bus;
    route_ = routes::tsdo(bus, server_node_id_, [this](const CANFrame& f) {
        handle_frame(f);
    });
}

void SDOClient::detach(BusInterface& bus) {
    if (route_) bus.remove_route(route_);
    route_ = 0;
    bus_ = nullptr;
}

SDOError SDOClient::upload_sync(uint16_t index, uint8_t subindex,
                                void* data, size_t& size) {
    if (!bus_) return SDOError::NO_BUS;

    {
        std::lock_guard<std::mutex> lock(pending_.mutex);
        pending_.response_ready = false;
        pending_.result = SDOError::OK;
        pending_.index = index;
        pending_.subindex = subindex;
        pending_.data.clear();
    }

    CANFrame request = MessageFactory::create_sdo_upload_request(
        server_node_id_, index, subindex);
    if (verbose_) {
        std::cerr << "[sdo] TX upload req  0x" << std::hex << request.id()
                  << "  " << std::dec << request.get_u8(0) << " "
                  << request.get_u16_le(1) << ":" << static_cast<int>(request.get_u8(3))
                  << std::endl;
    }
    if (!bus_->send(request)) {
        return SDOError::BUS_ERROR;
    }

    // Wait for response (handle_frame runs on RX thread or test thread)
    {
        std::unique_lock<std::mutex> lock(pending_.mutex);
        bool got = pending_.cv.wait_for(lock, std::chrono::milliseconds(timeout_ms_),
                                        [this] { return pending_.response_ready; });
        if (!got) {
            pending_.response_ready = true;  // consume slot
            if (verbose_) {
                std::cerr << "[sdo] TIMEOUT sau " << timeout_ms_
                          << "ms đọc 0x" << std::hex << index << std::dec
                          << ":" << static_cast<int>(subindex) << std::endl;
            }
            return SDOError::TIMEOUT;
        }
        if (verbose_) {
            std::cerr << "[sdo] RX response, " << pending_.data.size()
                      << " byte data" << std::endl;
        }
        if (pending_.result != SDOError::OK) {
            return pending_.result;
        }

        // Copy received data
        const size_t n = std::min(size, pending_.data.size());
        std::memcpy(data, pending_.data.data(), n);
        size = n;
    }
    return SDOError::OK;
}

SDOError SDOClient::download_sync(uint16_t index, uint8_t subindex,
                                  const void* data, size_t size) {
    if (size > 4) {
        return SDOError::INVALID_RESPONSE;  // segmented not supported
    }
    if (!bus_) return SDOError::NO_BUS;

    {
        std::lock_guard<std::mutex> lock(pending_.mutex);
        pending_.response_ready = false;
        pending_.result = SDOError::OK;
        pending_.index = index;
        pending_.subindex = subindex;
        pending_.data.clear();
    }

    CANFrame request = MessageFactory::create_sdo_download_request(
        server_node_id_, index, subindex, data, size);
    if (!bus_->send(request)) {
        return SDOError::BUS_ERROR;
    }

    {
        std::unique_lock<std::mutex> lock(pending_.mutex);
        bool got = pending_.cv.wait_for(lock, std::chrono::milliseconds(timeout_ms_),
                                        [this] { return pending_.response_ready; });
        if (!got) {
            pending_.response_ready = true;  // consume slot
            return SDOError::TIMEOUT;
        }
        return pending_.result;
    }
}

void SDOClient::upload(uint16_t index, uint8_t subindex, UploadCallback callback) {
    std::thread([this, index, subindex, cb = std::move(callback)]() {
        uint8_t buf[256];
        size_t size = sizeof(buf);
        SDOError err = upload_sync(index, subindex, buf, size);
        if (cb) {
            cb(err, err == SDOError::OK ? buf : nullptr,
               err == SDOError::OK ? size : 0);
        }
    }).detach();
}

void SDOClient::download(uint16_t index, uint8_t subindex, const void* data,
                         size_t size, DownloadCallback callback) {
    std::thread([this, index, subindex, data, size, cb = std::move(callback)]() {
        SDOError err = download_sync(index, subindex, data, size);
        if (cb) {
            cb(err);
        }
    }).detach();
}

void SDOClient::abort(uint32_t abort_code) {
    abort_code_.store(abort_code);

    {
        std::lock_guard<std::mutex> lock(pending_.mutex);
        if (!pending_.response_ready) {
            pending_.result = SDOError::ABORT;
            pending_.response_ready = true;
        }
    }
    pending_.cv.notify_all();

    if (bus_) {
        CANFrame frame = MessageFactory::create_sdo_abort(
            server_node_id_, 0, 0, abort_code);
        bus_->send(frame);
    }
}

void SDOClient::handle_frame(const CANFrame& frame) {
    if (frame.len() < 4) return;

    const uint32_t cob_id = frame.id() & 0x7FF;

    // Responses arrive on TSDO: 0x580 + node_id
    if ((cob_id & 0x780) != 0x580) return;
    if (server_node_id_ != 0 && (cob_id & 0x7F) != server_node_id_) return;

    const uint8_t cmd = frame.get_u8(0);
    const uint16_t index = frame.get_u16_le(1);
    const uint8_t subindex = frame.get_u8(3);

    std::lock_guard<std::mutex> lock(pending_.mutex);

    // Ignore stale responses (no pending request or different object)
    if (pending_.response_ready) return;
    if (index != pending_.index || subindex != pending_.subindex) return;

    if (cmd == 0x80) {
        // SDO abort
        abort_code_.store(frame.get_u32_le(4));
        pending_.result = SDOError::ABORT;
    } else {
        const uint8_t scs = cmd & 0xE0;
        if (scs != 0x40 && scs != 0x60 && scs != 0x20) return;

        // Extract expedited data
        size_t data_len = 0;
        const uint8_t* data_ptr = nullptr;
        if (cmd & 0x02) {
            if (cmd & 0x01) {
                const uint8_t n = (cmd >> 2) & 0x03;
                data_len = 4 - n;
            } else {
                data_len = 4;
            }
            data_ptr = frame.data() + 4;
        }

        if (data_ptr && data_len > 0) {
            pending_.data.assign(data_ptr, data_ptr + data_len);
        } else {
            pending_.data.clear();
        }
        pending_.result = SDOError::OK;
    }

    pending_.response_ready = true;
    pending_.cv.notify_all();
}

} // namespace canopen
