#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="${SCRIPT_DIR}/SimMachine"
ENV_FILE="${SCRIPT_DIR}/.env"
OFFICIAL_DOWNLOAD_PAGE="${FAIRINO_DOWNLOAD_PAGE:-https://manual.fairino.support/latest/download.html}"
DRIVE_URL_OVERRIDE="${FAIRINO_DRIVE_URL:-}"
FALLBACK_DRIVE_URL="https://drive.google.com/file/d/143PtR2eQj9tIgff_sCAVMdZrRZvnZ_ro/view"

say() {
  printf '[FAIRINO] %s\n' "$*"
}

die() {
  printf '[FAIRINO] ERROR: %s\n' "$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command '$1' is not installed."
}

compose() {
  docker compose --project-directory "${SCRIPT_DIR}" --env-file "${ENV_FILE}" "$@"
}

resolve_drive_url() {
  local page section discovered

  if [[ -n "${DRIVE_URL_OVERRIDE}" ]]; then
    printf '%s\n' "${DRIVE_URL_OVERRIDE}"
    return
  fi

  if page="$(curl --fail --location --silent --show-error "${OFFICIAL_DOWNLOAD_PAGE}")"; then
    section="$(sed -n '/id="fairino-simmachine"/,/id="cpp-sdk"/p' <<<"${page}")"
    discovered="$(sed -n '/<span class="pre">Docker<\/span>/s/.*href="\([^"]*\)".*/\1/p' <<<"${section}" | head -n 1)"
    if [[ -n "${discovered}" ]]; then
      printf '%s\n' "${discovered//&amp;/&}"
      return
    fi
  fi

  say "The official download page is currently unavailable; using the last known Drive link." >&2
  printf '%s\n' "${FALLBACK_DRIVE_URL}"
}

drive_id_from_url() {
  local url="$1"

  if [[ "${url}" =~ /d/([^/?]+) ]]; then
    printf '%s\n' "${BASH_REMATCH[1]}"
  else
    die "Could not extract a Google Drive ID from: ${url}"
  fi
}

read_remote_metadata() {
  local drive_id="$1" headers disposition

  headers="$(curl --fail --location --silent --show-error --head \
    "https://drive.usercontent.google.com/download?id=${drive_id}&export=download&confirm=t")"
  disposition="$(sed -n 's/^[Cc]ontent-[Dd]isposition:.*filename="\([^"]*\)".*/\1/p' <<<"${headers}" | tr -d '\r' | head -n 1)"
  [[ -n "${disposition}" ]] || die "Google Drive did not return a SimMachine package name."

  REMOTE_FILENAME="${disposition}"
  if [[ "${REMOTE_FILENAME}" =~ [vV]([0-9]+([.][0-9]+)+) ]]; then
    SIMMACHINE_VERSION="${BASH_REMATCH[1]}"
  else
    die "No version was found in '${REMOTE_FILENAME}'."
  fi
}

download_archive() {
  local drive_id="$1" archive="$2" partial="${archive}.part"

  say "Downloading ${REMOTE_FILENAME} to SimMachine/ (approximately 650 MB)..."
  curl --fail --location --retry 3 --progress-bar \
    "https://drive.usercontent.google.com/download?id=${drive_id}&export=download&confirm=t" \
    --output "${partial}"
  mv -- "${partial}" "${archive}"
}

extract_image_tar() {
  local archive="$1" version_dir="$2" image_tar="$3" entry partial

  entry="$(unzip -Z1 "${archive}" | sed -n '/\(^\|\/\)FAIRINOSimMachine\.tar$/p' | head -n 1)"
  [[ -n "${entry}" ]] || die "FAIRINOSimMachine.tar was not found in ${archive}."

  mkdir -p -- "${version_dir}"
  partial="${image_tar}.part"
  say "Extracting ${entry}..."
  unzip -p "${archive}" "${entry}" >"${partial}"
  mv -- "${partial}" "${image_tar}"
}

load_source_image() {
  local image_tar="$1" output loaded_image

  say "Loading FAIRINO Docker image v${SIMMACHINE_VERSION}..."
  output="$(docker load --input "${image_tar}")"
  printf '%s\n' "${output}"
  loaded_image="$(sed -n 's/^Loaded image: //p' <<<"${output}" | tail -n 1)"

  if [[ -z "${loaded_image}" ]]; then
    for loaded_image in fairino_simmachine:latest fairno_simmachine:latest; do
      if docker image inspect "${loaded_image}" >/dev/null 2>&1; then
        break
      fi
      loaded_image=""
    done
  fi

  [[ -n "${loaded_image}" ]] || die "The loaded FAIRINO image does not have an expected tag."
  SIMMACHINE_BASE_IMAGE="${loaded_image}"
}

write_env_file() {
  local temporary="${ENV_FILE}.tmp"

  {
    printf 'SIMMACHINE_VERSION=%s\n' "${SIMMACHINE_VERSION}"
    printf 'SIMMACHINE_IMAGE=fairino-simmachine:%s\n' "${SIMMACHINE_VERSION}"
    printf 'SIMMACHINE_IP=192.168.58.2\n'
  } >"${temporary}"
  mv -- "${temporary}" "${ENV_FILE}"
}

delete_all() {
  local container_ids
  local -a image_refs=()

  say "Removing the FAIRINO SimMachine and ROS 2 stack..."
  if [[ -f "${ENV_FILE}" ]]; then
    compose down --volumes || true
  fi

  say "Removing all FAIRINO stack containers..."
  container_ids="$(docker ps --all --quiet --filter 'name=^/fairino-simmachine-v')"
  if [[ -n "${container_ids}" ]]; then
    # The IDs come directly from Docker's exact SimMachine name filter.
    docker container rm --force ${container_ids}
  fi

  if docker container inspect fairino-ros2-humble >/dev/null 2>&1; then
    docker container rm --force fairino-ros2-humble
  fi

  if docker network inspect fairino-net >/dev/null 2>&1; then
    say "Removing fairino-net..."
    docker network rm fairino-net
  fi

  mapfile -t image_refs < <(
    docker image ls --format '{{.Repository}}:{{.Tag}}' |
      sed -n '/^fairino-simmachine:/p; /^fairino_simmachine:/p; /^fairno_simmachine:/p' |
      sort -u
  )
  if ((${#image_refs[@]} > 0)); then
    say "Removing FAIRINO SimMachine images..."
    docker image rm --force "${image_refs[@]}"
  fi

  if docker image inspect fairino-ros2-humble:local >/dev/null 2>&1; then
    say "Removing the ROS 2 Humble image..."
    docker image rm --force fairino-ros2-humble:local
  fi

  for volume in fairino-ros2-build fairino-ros2-install fairino-ros2-log; do
    if docker volume inspect "${volume}" >/dev/null 2>&1; then
      docker volume rm "${volume}"
    fi
  done

  if [[ -d "${SIM_DIR}" ]]; then
    say "Removing cached SimMachine archives..."
    find "${SIM_DIR}" -mindepth 1 ! -name '.gitkeep' -delete
  fi
  rm -f -- "${ENV_FILE}" "${ENV_FILE}.tmp"
  say "All generated FAIRINO SimMachine and ROS 2 resources have been removed."
}

prepare() {
  local drive_url drive_id archive version_dir image_tar final_image

  require_command curl
  require_command unzip
  require_command docker
  docker compose version >/dev/null 2>&1 || die "The Docker Compose plugin is unavailable."
  docker info >/dev/null 2>&1 || die "The Docker daemon is unavailable or this user lacks permission."

  mkdir -p -- "${SIM_DIR}"
  drive_url="$(resolve_drive_url)"
  drive_id="$(drive_id_from_url "${drive_url}")"
  read_remote_metadata "${drive_id}"

  archive="${SIM_DIR}/${REMOTE_FILENAME}"
  version_dir="${SIM_DIR}/v${SIMMACHINE_VERSION}"
  image_tar="${version_dir}/FAIRINOSimMachine.tar"
  final_image="fairino-simmachine:${SIMMACHINE_VERSION}"

  if docker image inspect "${final_image}" >/dev/null 2>&1; then
    say "Docker image ${final_image} already exists."
  else
    if [[ -s "${image_tar}" ]]; then
      say "Reusing ${image_tar}."
    else
      download_archive "${drive_id}" "${archive}"
      extract_image_tar "${archive}" "${version_dir}" "${image_tar}"
    fi

    load_source_image "${image_tar}"
    say "Tagging ${SIMMACHINE_BASE_IMAGE} as ${final_image}..."
    docker image tag "${SIMMACHINE_BASE_IMAGE}" "${final_image}"
  fi

  if [[ -f "${archive}" ]]; then
    rm -- "${archive}"
    say "Deleted the ZIP archive; the extracted TAR remains cached."
  fi

  write_env_file
}

usage() {
  cat <<'EOF'
Usage: ./simmachine.sh [up|update|down|logs|status|delete]

  up      check the latest version, prepare the image, and start the container
  update  same as up; explicit command for future upgrades
  down    stop and remove the container (the cached TAR and images remain)
  logs    follow the container logs
  status  show the Compose status
  delete  remove the complete stack, images, volumes, network, and cached files
EOF
}

main() {
  local command="${1:-up}"

  cd -- "${SCRIPT_DIR}"
  case "${command}" in
    up|update)
      prepare
      compose up --detach
      say "fairino-simmachine-v${SIMMACHINE_VERSION} is running."
      say "Web interface: http://192.168.58.2 (admin / 123)"
      ;;
    down)
      [[ -f "${ENV_FILE}" ]] || die ".env does not exist; SimMachine has not been prepared yet."
      compose down
      ;;
    logs)
      [[ -f "${ENV_FILE}" ]] || die ".env does not exist; run './simmachine.sh up' first."
      compose logs --follow simmachine
      ;;
    status)
      [[ -f "${ENV_FILE}" ]] || die ".env does not exist; run './simmachine.sh up' first."
      compose ps
      ;;
    delete)
      require_command docker
      docker info >/dev/null 2>&1 || die "The Docker daemon is unavailable or this user lacks permission."
      delete_all
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
}

main "$@"
