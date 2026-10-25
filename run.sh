export USR_NAME=dnp218

PIPER_CTRL_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd) || exit 1

if [ -z "${DISPLAY:-}" ]; then
	echo 'DISPLAY is not set. Run this script from an X11 desktop or forwarded X11 session.' >&2
	exit 1
fi

# Reuse the desktop session's cookie, including GDM/Wayland Xwayland paths.
# If there is no cookie file, authorize the actual runtime user via X11.
PIPER_XAUTHORITY=${XAUTHORITY:-"$HOME/.Xauthority"}
set --
if [ -r "${PIPER_XAUTHORITY}" ] && command -v xauth >/dev/null 2>&1; then
	PIPER_XAUTH_COPY=$(mktemp) || exit 1
	PIPER_DOCKER_XAUTH=$(mktemp) || { rm -f "${PIPER_XAUTH_COPY}"; exit 1; }
	trap 'rm -f "${PIPER_XAUTH_COPY}" "${PIPER_DOCKER_XAUTH}"' 0
	cp "${PIPER_XAUTHORITY}" "${PIPER_XAUTH_COPY}" || exit 1
	PIPER_XAUTH_RECORDS=$(xauth -f "${PIPER_XAUTH_COPY}" nlist "${DISPLAY}") || exit 1
	if [ -z "${PIPER_XAUTH_RECORDS}" ]; then
		echo "No Xauthority cookie found for DISPLAY=${DISPLAY}." >&2
		exit 1
	fi
	# FamilyWild avoids matching the host's name against Docker's hostname.
	printf '%s\n' "${PIPER_XAUTH_RECORDS}" | sed 's/^..../ffff/' | \
		xauth -f "${PIPER_DOCKER_XAUTH}" nmerge - || exit 1
	set -- --volume "${PIPER_DOCKER_XAUTH}:/home/$(id -un)/.Xauthority:ro" \
		-e "XAUTHORITY=/home/$(id -un)/.Xauthority"
else
	xhost +si:localuser:"$(id -un)" || exit 1
fi

# Pass every camera currently present on the host through to the container.
# V4L device numbering is not stable; the VLA defaults use /dev/video6 and
# /dev/video0, while a fixed /dev/video0--3 list can hide the front camera.
PIPER_CAMERA_DEVICE_ARGS=""
for piper_camera_device in /dev/video*; do
	[ -e "${piper_camera_device}" ] || continue
	PIPER_CAMERA_DEVICE_ARGS="${PIPER_CAMERA_DEVICE_ARGS} --device=${piper_camera_device}"
done

docker run --rm -it \
	--name piper-control-dev \
	-w /home/${USR_NAME}/docker_piper_ctrl \
	--volume "${PIPER_CTRL_DIR}:/home/${USR_NAME}/docker_piper_ctrl" \
	--volume "${PIPER_CTRL_DIR}/double_PiPER:/home/${USR_NAME}/double_PiPER" \
	--volume "${PIPER_CTRL_DIR}/piper_dataset:/home/${USR_NAME}/piper_dataset" \
	--volume "$HOME/.config/piper:/home/${USR_NAME}/.config/piper" \
	"$@" \
	--volume /tmp/.X11-unix:/tmp/.X11-unix -e "DISPLAY=${DISPLAY}" \
	--privileged --net=host --ipc=host \
	${PIPER_CAMERA_DEVICE_ARGS} \
	-e DOCKER_USER_NAME=$(id -un) \
	-e DOCKER_USER_ID=$(id -u) \
	-e DOCKER_USER_GROUP_NAME=$(id -gn) \
	-e DOCKER_USER_GROUP_ID=$(id -g) \
	piper-control:noetic
	# --gpus all \
	# --device /dev/input/js1:/dev/input/js1 \
	# --device /dev/input/js2:/dev/input/js2 \
	# --device /dev/ttyUSB0:/dev/ttyUSB0 \
	# --device /dev/ttyACM0:/dev/ttyACM0 \
