#!/bin/bash
# =============================================================================
# Script khởi động điều khiển động cơ ZLAC8015D
# =============================================================================
#
# Usage:
#   ./run_motor_control.sh              # Chạy với config mặc định
#   ./run_motor_control.sh can0 1       # Chạy với interface và node_id
#   ./run_motor_control.sh --config     # Chọn config file
#
# =============================================================================

# Directory chứa config
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(dirname "$SCRIPT_DIR")/config"

# Binary path
BINARY="$(dirname "$SCRIPT_DIR")/build/examples/dual_motor_keyboard"

# Default config
CONFIG_FILE="$CONFIG_DIR/zlac8015_config.yaml"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running as root (required for CAN)
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}Warning: Not running as root. CAN access may fail.${NC}"
    echo "Consider running with: sudo $0 $@"
fi

# Parse arguments
INTERFACE=""
NODE_ID=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -c, --config <file>   Config file (default: $CONFIG_FILE)"
            echo "  -h, --help            Show this help"
            echo ""
            echo "Arguments:"
            echo "  <interface> [node_id]  CAN interface and node ID"
            echo ""
            echo "Examples:"
            echo "  $0                    # Run with default config"
            echo "  $0 can0 1            # CAN interface can0, Node ID 1"
            echo "  $0 can0 2            # CAN interface can0, Node ID 2"
            echo "  $0 -c my_config.yaml # Run with custom config"
            exit 0
            ;;
        *)
            if [[ "$1" == can* ]] || [[ "$1" == vcan* ]]; then
                INTERFACE="$1"
            elif [[ "$1" =~ ^[0-9]+$ ]]; then
                NODE_ID="$1"
            fi
            shift
            ;;
    esac
done

# Check binary exists
if [ ! -f "$BINARY" ]; then
    echo -e "${RED}Error: Binary not found at $BINARY${NC}"
    echo "Please build the project first:"
    echo "  cd build && cmake .. && make dual_motor_keyboard"
    exit 1
fi

# Check CAN interface exists
if [ -n "$INTERFACE" ]; then
    if ! ip link show "$INTERFACE" &>/dev/null; then
        echo -e "${RED}Error: CAN interface '$INTERFACE' not found${NC}"
        echo "Available interfaces:"
        ip -br link show | grep -E "(can|vcan)"
        exit 1
    fi
    echo -e "${GREEN}Using CAN interface: $INTERFACE${NC}"
fi

# Print config
echo ""
echo "========================================"
echo "  ZLAC8015D Motor Control"
echo "========================================"
echo "Binary:    $BINARY"
echo "Interface:  ${INTERFACE:-can0 (default)}"
echo "Node ID:   ${NODE_ID:-1 (default)}"
echo "Config:    $CONFIG_FILE"
echo "========================================"
echo ""

# Build command
CMD="sudo $BINARY"

if [ -n "$INTERFACE" ]; then
    CMD="$CMD $INTERFACE"
    if [ -n "$NODE_ID" ]; then
        CMD="$CMD $NODE_ID"
    fi
elif [ -f "$CONFIG_FILE" ]; then
    CMD="$CMD --config $CONFIG_FILE"
fi

# Run
echo -e "${GREEN}Starting motor control...${NC}"
echo "Press Ctrl+C to stop safely"
echo ""
eval "$CMD"
EXIT_CODE=$?

echo ""
echo "========================================"
echo "Motor control exited with code: $EXIT_CODE"
echo "========================================"

exit $EXIT_CODE
