#!/usr/bin/env bash

mtt_read_env_value() {
  local repo_root="$1"
  local key="$2"
  local env_file
  local value

  for env_file in "${repo_root}/.env.local" "${repo_root}/.env"; do
    [[ -f "${env_file}" ]] || continue
    value="$(awk -F= -v lookup="${key}" '$1 == lookup {print substr($0, index($0, "=") + 1)}' "${env_file}" | tail -n 1)"
    if [[ -n "${value}" ]]; then
      printf '%s\n' "${value}"
      return 0
    fi
  done

  return 1
}

mtt_env_or_file_or_default() {
  local repo_root="$1"
  local key="$2"
  local default_value="${3-}"
  local file_value

  if [[ -n "${!key-}" ]]; then
    printf '%s\n' "${!key}"
    return 0
  fi

  file_value="$(mtt_read_env_value "${repo_root}" "${key}" || true)"
  if [[ -n "${file_value}" ]]; then
    printf '%s\n' "${file_value}"
    return 0
  fi

  printf '%s\n' "${default_value}"
}

mtt_target_user() {
  local target="$1"

  [[ "${target}" == *"@"* ]] || return 1
  printf '%s\n' "${target%@*}"
}

mtt_target_host() {
  local target="$1"

  if [[ "${target}" == *"@"* ]]; then
    printf '%s\n' "${target#*@}"
    return 0
  fi

  [[ -n "${target}" ]] || return 1
  printf '%s\n' "${target}"
}

mtt_default_robot_workspace() {
  local robot_user="${1:-robot}"
  printf '/home/%s/Project/mtt_ws\n' "${robot_user}"
}
