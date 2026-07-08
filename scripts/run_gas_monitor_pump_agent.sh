#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-${DEFAULT_WORKSPACE_ROOT}}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-${WORKSPACE_ROOT}/install/setup.bash}"
AGENT_BIN="${AGENT_BIN:-${WORKSPACE_ROOT}/install/gas_monitor/lib/gas_monitor/gas_monitor_pump_agent}"

INSTALL_CONFIG_FILE="${WORKSPACE_ROOT}/install/gas_monitor/share/gas_monitor/config/gas_params_default.yaml"
SOURCE_CONFIG_FILE="${WORKSPACE_ROOT}/src/gas_monitor/config/gas_params_default.yaml"
if [[ -f "${INSTALL_CONFIG_FILE}" ]]; then
  CONFIG_FILE="${CONFIG_FILE:-${INSTALL_CONFIG_FILE}}"
else
  CONFIG_FILE="${CONFIG_FILE:-${SOURCE_CONFIG_FILE}}"
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Workspace setup file not found: ${WS_SETUP}" >&2
  exit 1
fi
if [[ ! -x "${AGENT_BIN}" ]]; then
  echo "Agent binary not found or not executable: ${AGENT_BIN}" >&2
  exit 1
fi
if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

exec "${AGENT_BIN}" "${CONFIG_FILE}"
