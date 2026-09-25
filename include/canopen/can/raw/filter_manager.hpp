/**
 * @file filter_manager.hpp
 * @brief CAN Filter Manager for multiple node filtering
 */

#ifndef CANOPEN_CAN_RAW_FILTER_MANAGER_HPP
#define CANOPEN_CAN_RAW_FILTER_MANAGER_HPP

#include <linux/can.h>
#include <vector>
#include <functional>
#include <mutex>
#include <string>

namespace canopen {

/**
 * @brief Filter Manager for CANopen nodes
 *
 * Manages CAN filters for multiple CANopen nodes with support for:
 * - Individual filter management
 * - Predefined filter sets (NMT, PDO, etc.)
 * - Dynamic filter updates
 */
class FilterManager {
public:
    FilterManager() = default;
    ~FilterManager() = default;

    /**
     * @brief Attach to socket
     * @param sock_fd Socket file descriptor
     */
    void attach(int sock_fd);

    /**
     * @brief Detach from socket
     */
    void detach();

    // ==================== Individual Filters ====================

    /**
     * @brief Add single filter
     * @param can_id CAN ID
     * @param can_mask CAN mask
     * @return 0 on success, negative errno on error
     */
    int add_filter(uint32_t can_id, uint32_t can_mask);

    /**
     * @brief Remove filter by CAN ID
     * @param can_id CAN ID to remove
     * @return 0 on success, negative errno on error
     */
    int remove_filter(uint32_t can_id);

    /**
     * @brief Add extended ID filter
     * @param can_id Extended CAN ID
     * @param can_mask Extended mask
     * @return 0 on success
     */
    int add_extended_filter(uint32_t can_id, uint32_t can_mask);

    // ==================== Bulk Operations ====================

    /**
     * @brief Set all filters at once
     * @param filters Vector of filters
     * @return 0 on success
     */
    int set_filters(const std::vector<struct can_filter>& filters);

    /**
     * @brief Clear all filters
     * @return 0 on success
     */
    int clear_all_filters();

    // ==================== Predefined Filters ====================

    /**
     * @brief Set NMT broadcast filter (0x000)
     */
    int set_nmt_filter();

    /**
     * @brief Set SYNC filter (0x080)
     */
    int set_sync_filter();

    /**
     * @brief Set PDO filters for specific node
     * @param node_id Node ID (1-127)
     * @param pdo_mask Which PDOs (bitmask: bit 0=TPDO1, bit 1=RPDO1, etc.)
     */
    int set_pdo_filter(uint8_t node_id, uint8_t pdo_mask = 0xFF);

    /**
     * @brief Set SDO filters for specific node
     * @param node_id Node ID (1-127)
     */
    int set_sdo_filter(uint8_t node_id);

    /**
     * @brief Set heartbeat filter
     */
    int set_heartbeat_filter();

    /**
     * @brief Set EMCY filter for specific node
     * @param node_id Node ID (0xFF for all)
     */
    int set_emcy_filter(uint8_t node_id = 0xFF);

    /**
     * @brief Set all nodes filter (accept all CANopen frames)
     */
    int set_all_nodes_filter();

    /**
     * @brief Set master-only filter (accept commands to master)
     */
    int set_master_filter();

    // ==================== Accessors ====================

    /**
     * @brief Get current active filters
     */
    std::vector<struct can_filter> get_active_filters() const;

    /**
     * @brief Get filter count
     */
    size_t get_filter_count() const;

    /**
     * @brief Check if filter is active
     */
    bool has_filter(uint32_t can_id) const;

private:
    int apply_filters();
    uint32_t make_pdo_cobid(uint8_t node_id, uint8_t base_id);

    int sock_fd_{-1};
    std::vector<struct can_filter> filters_;
    mutable std::mutex mutex_;
};

} // namespace canopen

#endif // CANOPEN_CAN_RAW_FILTER_MANAGER_HPP
