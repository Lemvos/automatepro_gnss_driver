#!/bin/bash

# Lifecycle Node Transition Test Script
# This script triggers a series of lifecycle transitions for testing the GNSS driver

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Node name - adjust if different
NODE_NAME="ublox_gps_base_node"

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to execute transition and check result
execute_transition() {
    local transition=$1
    local expected_state=$2
    
    print_status "Executing transition: $transition"
    
    # Execute the transition
    result=$(ros2 lifecycle set $NODE_NAME $transition 2>&1)
    
    if [ $? -eq 0 ]; then
        print_success "Transition '$transition' completed successfully"
        
        # Get current state
        current_state=$(ros2 lifecycle get $NODE_NAME 2>&1)
        if [[ $current_state == *"$expected_state"* ]]; then
            print_success "Node is now in state: $expected_state"
        else
            print_warning "Expected state '$expected_state', but got: $current_state"
        fi
    else
        print_error "Transition '$transition' failed: $result"
        return 1
    fi
    
    # Wait a bit between transitions
    sleep 2
    return 0
}

# Function to get current node state
get_current_state() {
    local state=$(ros2 lifecycle get $NODE_NAME 2>&1)
    echo "$state"
}

# Main execution
main() {
    print_status "Starting lifecycle transition test for node: $NODE_NAME"
    print_status "========================================================="
    
    # Check if node exists
    if ! ros2 lifecycle nodes | grep -q "$NODE_NAME"; then
        print_error "Lifecycle node '$NODE_NAME' not found!"
        print_status "Available lifecycle nodes:"
        ros2 lifecycle nodes
        exit 1
    fi
    
    # Get initial state
    initial_state=$(get_current_state)
    print_status "Initial node state: $initial_state"
    
    # Transition sequence as requested
    transitions=(
        "deactivate:inactive"
        "cleanup:unconfigured"
        "configure:inactive"
        "cleanup:unconfigured"
        "configure:inactive"
        "activate:active"
        "deactivate:inactive"
        "activate:active"
    )
    
    print_status "Executing transition sequence..."
    echo ""
    
    for i in "${!transitions[@]}"; do
        IFS=':' read -r transition expected_state <<< "${transitions[$i]}"
        
        echo "----------------------------------------"
        print_status "Step $((i+1))/8: $transition -> $expected_state"
        
        if ! execute_transition "$transition" "$expected_state"; then
            print_error "Transition sequence failed at step $((i+1))"
            exit 1
        fi
        
        echo ""
    done
    
    # Final state check
    final_state=$(get_current_state)
    print_status "========================================================="
    print_success "All transitions completed successfully!"
    print_status "Final node state: $final_state"
}

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Lifecycle Node Transition Test Script"
    echo ""
    echo "Options:"
    echo "  -n, --node NODE_NAME    Specify the lifecycle node name (default: $NODE_NAME)"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "This script executes the following transition sequence:"
    echo "  1. deactivate"
    echo "  2. cleanup"
    echo "  3. configure"
    echo "  4. cleanup"
    echo "  5. configure"
    echo "  6. activate"
    echo "  7. deactivate"
    echo "  8. activate"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--node)
            NODE_NAME="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Execute main function
main