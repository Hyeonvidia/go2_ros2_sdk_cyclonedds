#!/usr/bin/env bash
# setup_network.sh — Add static secondary IP for Go2 CycloneDDS communication
#
# Adds 192.168.123.18/24 as a secondary address on enP8p1s0
# alongside the existing DHCP configuration. Persists across reboots.
#
# Usage:
#   sudo bash setup_network.sh              # Add static IP
#   sudo bash setup_network.sh --check-only # Check current status
#   sudo bash setup_network.sh --remove     # Remove static IP

set -euo pipefail

IFACE="${IFACE:-enP8p1s0}"
STATIC_IP="${STATIC_IP:-192.168.123.18}"
CIDR="${STATIC_IP}/24"
GO2_IP="${GO2_IP:-192.168.123.161}"

# --- helpers ---
info()  { echo -e "\033[1;32m[INFO]\033[0m  $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m  $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; }

# --- find NM connection name by interface ---
find_connection() {
    local conn
    conn=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | grep ":${IFACE}$" | cut -d: -f1)
    if [[ -z "$conn" ]]; then
        conn=$(nmcli -t -f NAME,DEVICE con show 2>/dev/null | grep ":${IFACE}$" | cut -d: -f1)
    fi
    if [[ -z "$conn" ]]; then
        error "No NetworkManager connection found for interface ${IFACE}"
        exit 1
    fi
    echo "$conn"
}

# --- check if static IP is already configured in NM profile ---
is_configured() {
    local conn="$1"
    nmcli -t -f ipv4.addresses con show "$conn" 2>/dev/null | grep -q "${STATIC_IP}"
}

# --- check if static IP is live on the interface ---
is_live() {
    ip addr show "$IFACE" 2>/dev/null | grep -q "inet ${STATIC_IP}/"
}

# --- check for IP conflict using arping ---
check_conflict() {
    if ! command -v arping &>/dev/null; then
        warn "arping not installed, skipping IP conflict check"
        return 0
    fi
    info "Checking for IP conflict on ${STATIC_IP}..."
    if arping -D -c 2 -w 3 -I "$IFACE" "$STATIC_IP" 2>/dev/null | grep -q "Received"; then
        error "IP ${STATIC_IP} is already in use on the network!"
        error "Try a different IP (e.g., 192.168.123.100)"
        exit 1
    fi
    info "No IP conflict detected"
}

# --- verify Go2 connectivity ---
verify_connectivity() {
    info "Pinging Go2 at ${GO2_IP}..."
    if ping -c 3 -W 2 "$GO2_IP" &>/dev/null; then
        info "Go2 (${GO2_IP}) is reachable!"
        return 0
    else
        warn "Go2 (${GO2_IP}) is not responding to ping"
        warn "Check that the Go2 is powered on and ethernet cable is connected"
        return 1
    fi
}

# --- show current status ---
show_status() {
    local conn="$1"
    echo ""
    info "=== Network Status ==="
    echo ""

    if is_configured "$conn"; then
        info "NM profile: ${CIDR} is configured (persistent)"
    else
        warn "NM profile: ${CIDR} is NOT configured"
    fi

    if is_live; then
        info "Interface:  ${CIDR} is active on ${IFACE}"
    else
        warn "Interface:  ${CIDR} is NOT active on ${IFACE}"
    fi

    echo ""
    info "Current IPs on ${IFACE}:"
    ip -4 addr show "$IFACE" | grep inet | sed 's/^/  /'
    echo ""

    verify_connectivity || true
}

# --- add static IP ---
add_static_ip() {
    local conn="$1"

    if is_configured "$conn"; then
        info "Static IP ${CIDR} is already configured in NM profile"
        if is_live; then
            info "IP is already active. Nothing to do."
            verify_connectivity || true
            return 0
        fi
        info "Reactivating connection to apply..."
        nmcli con up "$conn" >/dev/null 2>&1
        sleep 2
        verify_connectivity || true
        return 0
    fi

    check_conflict

    info "Adding ${CIDR} to connection '${conn}'..."
    nmcli con mod "$conn" +ipv4.addresses "$CIDR"

    info "Reactivating connection to apply..."
    nmcli con up "$conn" >/dev/null 2>&1
    sleep 3

    if is_live; then
        info "Static IP ${CIDR} is now active on ${IFACE}"
    else
        error "Failed to apply static IP. Check NetworkManager logs."
        exit 1
    fi

    echo ""
    info "Current IPs on ${IFACE}:"
    ip -4 addr show "$IFACE" | grep inet | sed 's/^/  /'
    echo ""

    verify_connectivity || true
    info "Configuration is persistent and will survive reboots."
}

# --- remove static IP ---
remove_static_ip() {
    local conn="$1"

    if ! is_configured "$conn"; then
        info "Static IP ${CIDR} is not configured. Nothing to remove."
        return 0
    fi

    info "Removing ${CIDR} from connection '${conn}'..."
    nmcli con mod "$conn" -ipv4.addresses "$CIDR"

    info "Reactivating connection to apply..."
    nmcli con up "$conn" >/dev/null 2>&1
    sleep 2

    info "Static IP removed."
}

# --- main ---
main() {
    if [[ $EUID -ne 0 ]]; then
        error "This script must be run as root (use sudo)"
        exit 1
    fi

    local conn
    conn=$(find_connection)
    info "Using NM connection: '${conn}' on ${IFACE}"

    case "${1:-}" in
        --check-only)
            show_status "$conn"
            ;;
        --remove)
            remove_static_ip "$conn"
            ;;
        *)
            add_static_ip "$conn"
            ;;
    esac
}

main "$@"
