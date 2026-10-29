#!/usr/bin/env bash
set -e

# Keep Docker's working directory and run the command as the requested host
# UID/GID. Never recursively chown bind mounts or copy root's SSH credentials.
if [ "$(id -u)" = 0 ] &&
   [ -n "${DOCKER_USER_NAME:-}" ] && [ -n "${DOCKER_USER_GROUP_NAME:-}" ] &&
   [[ "${DOCKER_USER_ID:-}" =~ ^[0-9]+$ ]] &&
   [[ "${DOCKER_USER_GROUP_ID:-}" =~ ^[0-9]+$ ]]; then
    piper_user="${DOCKER_USER_NAME}"
    piper_group="${DOCKER_USER_GROUP_NAME}"
    piper_home="/home/${piper_user}"

    if ! getent group "${piper_group}" > /dev/null; then
        groupadd --non-unique --gid "${DOCKER_USER_GROUP_ID}" "${piper_group}"
    fi
    if ! id "${piper_user}" > /dev/null 2>&1; then
        useradd --non-unique --uid "${DOCKER_USER_ID}" --gid "${piper_group}" \
            --create-home --home-dir "${piper_home}" --shell /bin/bash "${piper_user}"
    fi
    # Match the existing UID/GID remapping without usermod's recursive home
    # ownership changes, which could otherwise modify mounted teaching data.
    python3 - "${piper_user}" "${DOCKER_USER_ID}" "${piper_group}" "${DOCKER_USER_GROUP_ID}" <<'PY'
from pathlib import Path
import sys
user, uid, group, gid = sys.argv[1:]
for path, name, updates in (
    (Path('/etc/passwd'), user, {2: uid, 3: gid}),
    (Path('/etc/group'), group, {2: gid}),
):
    rows = []
    for line in path.read_text().splitlines():
        fields = line.split(':')
        if fields[0] == name:
            for index, value in updates.items():
                fields[index] = value
        rows.append(':'.join(fields))
    path.write_text('\n'.join(rows) + '\n')
PY
    mkdir -p "${piper_home}/.ros" "${piper_home}/.cache" "${piper_home}/.local" "${piper_home}/.config"
    chown "${DOCKER_USER_ID}:${DOCKER_USER_GROUP_ID}" "${piper_home}" \
        "${piper_home}/.ros" "${piper_home}/.cache" "${piper_home}/.local" "${piper_home}/.config"
    if getent group video > /dev/null; then
        usermod --append --groups video "${piper_user}"
    fi
    printf '%s ALL=(ALL) NOPASSWD: ALL\n' "${piper_user}" > /etc/sudoers.d/piper-user
    chmod 440 /etc/sudoers.d/piper-user
    # Only identified generated outputs are repaired. Source/data trees and
    # symlinks are never traversed. The actual build still runs through gosu.
    piper_workspace="${PIPER_WORKSPACE:-${piper_home}/docker_piper_ctrl/double_PiPER}"
    if [ "${piper_workspace}" = "${piper_home}/double_PiPER" ]; then
        piper_workspace="${piper_home}/docker_piper_ctrl/double_PiPER"
    fi
    export PIPER_WORKSPACE="${piper_workspace}"
    /usr/bin/python3 /piper_generated_permissions.py "${piper_workspace}" \
        --uid "${DOCKER_USER_ID}" --gid "${DOCKER_USER_GROUP_ID}"
    exec gosu "${piper_user}" "$@"
fi
exec "$@"
